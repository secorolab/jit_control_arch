#include <signal.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/framevel.hpp>
#include <kdl/framevel_io.hpp>

#include "robif2b/functions/kinova_gen3.h"

#include "jit_control_arch/controllers.h"


#define LOG_INFO(node, msg, ...) RCLCPP_INFO(node->get_logger(), msg, ##__VA_ARGS__)
#define LOG_INFO_S(node, msg, ...) RCLCPP_INFO_STREAM(node->get_logger(), msg, ##__VA_ARGS__)
#define LOG_ERROR(node, msg, ...) RCLCPP_ERROR(node->get_logger(), msg, ##__VA_ARGS__)

#define DEG2RAD(angle_deg) ((angle_deg) * M_PI / 180.0)
#define RAD2DEG(angle_rad) ((angle_rad) * 180.0 / M_PI)

#define NUM_JOINTS 7


struct motion_state_t {
    bool active;
    KDL::Rotation start_ee_rot;
};

struct motionspec_t {
    double rotation_angle;    // [deg]
    double rotation_dir;      // [+1 cw, -1 ccw]
    double rotation_tol_ulim; // [deg]
    double rotation_tol_llim; // [deg]
    double torque_ulim;       // [Nm]
    double torque_llim;       // [Nm]
    // state
    struct motion_state_t state;
};

struct wait_state_t {
    bool active;
    struct timespec start_time;
};

struct waitspec_t {
    double wait_time; // [s]
    // state
    struct wait_state_t state;
};

enum task_type_e {
    MOTION,
    WAIT
};

struct taskitem_t {
    enum task_type_e type;
    union {
        struct motionspec_t motion;
        struct waitspec_t   wait;
    } spec;
};

struct taskspec_t {
    struct taskitem_t *items;
    size_t count;
    size_t capacity;
};

bool motion_done(struct motionspec_t *motion_spec,
                    double current_rotation,
                    double current_torque) {
    double target_angle = motion_spec->rotation_angle * motion_spec->rotation_dir;
    double angle_llim = target_angle - motion_spec->rotation_tol_llim;
    double angle_ulim = target_angle + motion_spec->rotation_tol_ulim;

    bool angle_reached = (current_rotation >= angle_llim) && (current_rotation <= angle_ulim);

    double torque_with_dir = current_torque * motion_spec->rotation_dir;
    bool torque_exceeded = (torque_with_dir >= motion_spec->torque_ulim);

    return angle_reached || torque_exceeded;
}

bool wait_done(double wait_time_sec, struct timespec start_time, struct timespec current_time) {
    const int NSEC_IN_SEC = 1000000000;
    double elapsed_time = (current_time.tv_sec - start_time.tv_sec) +
                            (current_time.tv_nsec - start_time.tv_nsec) / (double)NSEC_IN_SEC;
   
    return elapsed_time >= wait_time_sec;
}

volatile sig_atomic_t stop_signal_received = 0;

void handle_stop_signal(int signal) {
    stop_signal_received = 1;
}

static long timespec_to_usec(const struct timespec *t) {
    const int NSEC_IN_USEC = 1000;
    const int USEC_IN_SEC  = 1000000;

    return t->tv_sec * USEC_IN_SEC + t->tv_nsec / NSEC_IN_USEC;
}

struct CycleTimeState {
    struct timespec cycle_start;
    struct timespec cycle_end;
    long cycle_time_exp; // [us]
    long cycle_time_msr; // [us]
};

enum task_state_e {
    TASK_STATE_IDLE,
    TASK_STATE_START,
    TASK_STATE_STOP
};

volatile enum task_state_e g_task_state = TASK_STATE_IDLE;

void task_control_cb(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "start") {
        g_task_state = TASK_STATE_START;
    } else if (msg->data == "stop") {
        g_task_state = TASK_STATE_STOP;
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, handle_stop_signal);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kinova_test_node");

    auto task_control_sub = node->create_subscription<std_msgs::msg::String>(
        "task_control", 10,
        task_control_cb
    );

    // ---- task specification ----
    taskspec_t task_spec;
    task_spec.count = 3;
    task_spec.capacity = 5;
    task_spec.items = (taskitem_t *)malloc(sizeof(taskitem_t) * task_spec.capacity);

    // first item: find motion
    task_spec.items[0].type = MOTION;
    task_spec.items[0].spec.motion.rotation_angle     = 120.0; // [deg]
    task_spec.items[0].spec.motion.rotation_dir       = 1.0;   // clockwise
    task_spec.items[0].spec.motion.rotation_tol_ulim  = 0.5;   // [deg]
    task_spec.items[0].spec.motion.rotation_tol_llim  = 0.5;   // [deg]
    task_spec.items[0].spec.motion.torque_ulim        = 4.0;   // [Nm]
    task_spec.items[0].spec.motion.torque_llim        = 0.0;   // [Nm]
    task_spec.items[0].spec.motion.state              = { .active = false, .start_ee_rot = KDL::Rotation::Identity() };

    // second item: wait
    task_spec.items[1].type = WAIT;
    task_spec.items[1].spec.wait.wait_time = 5.0; // [s]
    task_spec.items[1].spec.wait.state     = { .active = false, .start_time = {0, 0} };

    // third item: unscrew motion
    task_spec.items[2].type = MOTION;
    task_spec.items[2].spec.motion.rotation_angle     = 90.0;  // [deg]
    task_spec.items[2].spec.motion.rotation_dir       = -1.0;  // counter-clockwise
    task_spec.items[2].spec.motion.rotation_tol_ulim  = 5.0;   // [deg]
    task_spec.items[2].spec.motion.rotation_tol_llim  = 5.0;   // [deg]
    task_spec.items[2].spec.motion.torque_ulim        = 4.0;   // [Nm]
    task_spec.items[2].spec.motion.torque_llim        = 0.05;   // [Nm]
    task_spec.items[2].spec.motion.state              = { .active = false, .start_ee_rot = KDL::Rotation::Identity() };

    bool success = false;
    
    enum robif2b_hl_ctrl_mode ctrl_mode = ROBIF2B_HL_CTRL_MODE_WRENCH;
    enum robif2b_kinova_cart_ref_frame ref_frame = ROBIF2B_KINOVA_CART_REF_FRAME_TOOL;
    enum robif2b_kinova_cart_wrench_mode wrench_mode = ROBIF2B_KINOVA_CART_WRENCH_MODE_NORMAL;
    double pos_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double imu_ang_vel_msr[] = { 0.0, 0.0, 0.0 };
    double imu_lin_acc_msr[] = { 0.0, 0.0, 0.0 };
    double tool_ext_wrench_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; 

    double wrench_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
   
    struct robif2b_kinova_gen3_hl_nbx rob = {
        // Configuration
        .conf = {
            .ip_address         = "192.168.1.10",
            .port               = 10000,
            .port_real_time     = 10001,
            .user               = "admin",
            .password           = "admin",
            .session_timeout    = 60000,
            .connection_timeout = 2000,
        },

        // Connections
        .ctrl_mode           = &ctrl_mode,
        .reference_frame     = &ref_frame,
        .wrench_mode         = &wrench_mode,
        .jnt_pos_msr         = pos_msr,
        .jnt_vel_msr         = vel_msr,
        .jnt_trq_msr         = eff_msr,
        .act_cur_msr         = cur_msr,
        .imu_ang_vel_msr     = imu_ang_vel_msr,
        .imu_lin_acc_msr     = imu_lin_acc_msr,
        .tool_ext_wrench_msr = tool_ext_wrench_msr,
        .twist_cmd           = NULL,
        .wrench_cmd          = wrench_cmd,
        .success             = &success
    };

    struct CycleTimeState cycle_time_state = {
        .cycle_time_exp = 1000 // [us]
    };

    // load urdf
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("jit_control_arch");
    std::string urdf_path = package_share_directory + "/urdf/gen3_robotiq_2f_85.urdf";
    
    KDL::Tree tree;
    KDL::Chain chain;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        LOG_ERROR(node, "Failed to construct KDL tree from URDF file: %s", urdf_path.c_str());
        return -1;
    }
    if (!tree.getChain("base_link", "end_effector_link", chain)) {
        LOG_ERROR(node, "Failed to get KDL chain from tree.");
        return -1;
    }

    int num_joints = chain.getNrOfJoints();
    assert(num_joints == NUM_JOINTS && "Number of joints does not match expected value.");

    KDL::JntArrayVel jnt_array_vel(num_joints);
    jnt_array_vel.q = KDL::JntArray(num_joints);
    jnt_array_vel.qdot = KDL::JntArray(num_joints);

    KDL::FrameVel fvel;
    KDL::ChainFkSolverVel_recursive fk_solver_vel(chain);

    fk_solver_vel.JntToCart(jnt_array_vel, fvel, -1);
    auto ee_p = fvel.GetFrame().p;
    double ee_rpy[3];
    fvel.GetFrame().M.GetRPY(ee_rpy[0], ee_rpy[1], ee_rpy[2]);
    LOG_INFO_S(node, "ZeroConfig EE: " << ee_p << " RPY: [" 
                    << RAD2DEG(ee_rpy[0]) << ", "
                    << RAD2DEG(ee_rpy[1]) << ", "
                    << RAD2DEG(ee_rpy[2]) << "]");

    robif2b_kinova_gen3_hl_configure(&rob);
    if (!success) {
        LOG_ERROR(node, "Failed to configure Kinova Gen3 robot.");
        return -1;
    }

    robif2b_kinova_gen3_hl_recover(&rob);
    if (!success) {
        LOG_ERROR(node, "Failed to recover Kinova Gen3 robot.");
        return -1;
    }

    LOG_INFO(node, "Starting Kinova Gen3 robot control loop.");
    robif2b_kinova_gen3_hl_start(&rob);
    if (!success) {
        LOG_ERROR(node, "Failed to start Kinova Gen3 robot.");
        return -1;
    }
    
    const int buffer_size = 20;
    double wrench_msr_bias[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < buffer_size; i++) {
        robif2b_kinova_gen3_hl_update(&rob);
        if (!success) {
            LOG_ERROR(node, "Failed to update Kinova Gen3 robot during wrench bias initialization.");
            return -1;
        }
        for (int j = 0; j < 6; j++) {
            wrench_msr_bias[j] += tool_ext_wrench_msr[j];
        }
    }
    for (int j = 0; j < 6; j++) {
        wrench_msr_bias[j] /= buffer_size;
    }
    LOG_INFO(node, "Wrench bias initialized: [%f, %f, %f, %f, %f, %f]",
             wrench_msr_bias[0], wrench_msr_bias[1], wrench_msr_bias[2],
             wrench_msr_bias[3], wrench_msr_bias[4], wrench_msr_bias[5]);

    // wait for start command
    while (rclcpp::ok() && g_task_state != TASK_STATE_START && !stop_signal_received) {
        rclcpp::spin_some(node);
    }
    LOG_INFO(node, "Received start command. Beginning task execution.");
    
    size_t current_task_index = 0;
    struct taskitem_t *current_task = &task_spec.items[current_task_index];

    while (rclcpp::ok() && !stop_signal_received) {
        clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_start);

        if (current_task_index >= task_spec.count) {
            LOG_INFO(node, "All tasks completed. Exiting control loop.");
            break;
        }

        // compute measured wrench with bias compensation
        double wrench_msr[6];
        for (int i = 0; i < 6; i++) {
            wrench_msr[i] = tool_ext_wrench_msr[i] - wrench_msr_bias[i];
        }
        // compute end-effector pose and twist
        for (int i = 0; i < NUM_JOINTS; i++) {
            jnt_array_vel.q(i)    = pos_msr[i];
            jnt_array_vel.qdot(i) = vel_msr[i];
        }
        fk_solver_vel.JntToCart(jnt_array_vel, fvel, -1);
        auto ee_p = fvel.GetFrame().p;
        double ee_rpy[3];
        fvel.GetFrame().M.GetRPY(ee_rpy[0], ee_rpy[1], ee_rpy[2]);

        // ---- task execution ----
        switch (current_task->type) {
            case WAIT:
            {
                if (!current_task->spec.wait.state.active)
                {
                    current_task->spec.wait.state.active = true;
                    clock_gettime(CLOCK_MONOTONIC, &current_task->spec.wait.state.start_time);
                }
                
                struct timespec current_time;
                clock_gettime(CLOCK_MONOTONIC, &current_time);
                if (wait_done(current_task->spec.wait.wait_time,
                              current_task->spec.wait.state.start_time,
                              current_time)) {
                    LOG_INFO(node, "Wait task completed.");
                    current_task_index++;
                    if (current_task_index < task_spec.count) {
                        current_task = &task_spec.items[current_task_index];
                    }
                }
                break;
            }

            case MOTION:
            {
                if (!current_task->spec.motion.state.active)
                {
                    current_task->spec.motion.state.active = true;
                    current_task->spec.motion.state.start_ee_rot = fvel.GetFrame().M;
                }
                // compute relative rotation angle
                KDL::Rotation rel_rot = current_task->spec.motion.state.start_ee_rot.Inverse() * fvel.GetFrame().M;
                double rxyz[3];
                rel_rot.GetRPY(rxyz[0], rxyz[1], rxyz[2]);
                LOG_INFO_S(node, "Relative Rotation RPY: [" 
                                << RAD2DEG(rxyz[0]) << ", "
                                << RAD2DEG(rxyz[1]) << ", "
                                << RAD2DEG(rxyz[2]) << "]");

                const int desired_axis = 2; // Z-axis
                
                double current_rotation_deg = RAD2DEG(rxyz[desired_axis]);
                double current_torque = wrench_msr[desired_axis + 3];
                
                if (motion_done(&current_task->spec.motion,
                                current_rotation_deg,
                                current_torque)) {
                    LOG_INFO(node, "Motion task completed. Rotation: %f deg, Torque: %f Nm",
                             current_rotation_deg, current_torque);
                    // stop motion
                    for (int i = 0; i < 6; i++) {
                        wrench_cmd[i] = 0.0;
                    }
                    current_task_index++;
                    if (current_task_index < task_spec.count) {
                        current_task = &task_spec.items[current_task_index];
                    }
                } else {
                    // continue motion: apply wrench command around Z-axis
                    double wrench_magnitude = 5.0 * current_task->spec.motion.rotation_dir;
                    wrench_cmd[3 + desired_axis] = wrench_magnitude;
                }
                break;
            }
        }

        // ---- update robot ----
        robif2b_kinova_gen3_hl_update(&rob);
        if (!success) {
            LOG_ERROR(node, "Failed to update Kinova Gen3 robot.");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_end);
        cycle_time_state.cycle_time_msr = timespec_to_usec(&cycle_time_state.cycle_end)
                                  - timespec_to_usec(&cycle_time_state.cycle_start);

        // threshold for cycle time to be >= 0
        if (cycle_time_state.cycle_time_msr < cycle_time_state.cycle_time_exp) {
            struct timespec sleep_time = {
                .tv_sec  = 0,
                .tv_nsec = (cycle_time_state.cycle_time_exp - cycle_time_state.cycle_time_msr) * 1000
            };
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL);
        }
    }

    LOG_INFO(node, "Stopping Kinova Gen3 robot control.");
    robif2b_kinova_gen3_hl_stop(&rob);
    LOG_INFO(node, "Kinova Gen3 robot control loop stopped.");

    robif2b_kinova_gen3_hl_shutdown(&rob);
    LOG_INFO(node, "Kinova Gen3 robot shutdown completed.");

    rclcpp::shutdown();
    return 0;
}

