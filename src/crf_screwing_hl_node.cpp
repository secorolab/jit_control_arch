#include <signal.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>

#include <rclcpp/rclcpp.hpp>
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

#define NUM_JOINTS 7


struct motionspec_t {
    double rotation_angle;    // [deg]
    double rotation_dir;      // [+1 cw, -1 ccw]
    double rotation_tol_ulim; // [deg]
    double rotation_tol_llim; // [deg]
    double torque_ulim;       // [Nm]
    double torque_llim;       // [Nm]
};

struct waitspec_t {
    double wait_time; // [s]
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

int main(int argc, char **argv) {
    signal(SIGINT, handle_stop_signal);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kinova_test_node");

    // ---- task specification ----
    taskspec_t task_spec;
    task_spec.count = 3;
    task_spec.capacity = 3;
    task_spec.items = (taskitem_t *)malloc(sizeof(taskitem_t) * task_spec.capacity);

    // first item: find motion
    task_spec.items[0].type = MOTION;
    task_spec.items[0].spec.motion.rotation_angle     = 120.0; // [deg]
    task_spec.items[0].spec.motion.rotation_dir       = 1.0;   // clockwise
    task_spec.items[0].spec.motion.rotation_tol_ulim  = 0.0;   // [deg]
    task_spec.items[0].spec.motion.rotation_tol_llim  = 0.0;   // [deg]
    task_spec.items[0].spec.motion.torque_ulim        = 4.0;   // [Nm]
    task_spec.items[0].spec.motion.torque_llim        = 0.0;   // [Nm]

    // second item: wait
    task_spec.items[1].type = WAIT;
    task_spec.items[1].spec.wait.wait_time = 1.0; // [s]

    // third item: unscrew motion
    task_spec.items[2].type = MOTION;
    task_spec.items[2].spec.motion.rotation_angle     = 90.0;  // [deg]
    task_spec.items[2].spec.motion.rotation_dir       = -1.0;  // counter-clockwise
    task_spec.items[2].spec.motion.rotation_tol_ulim  = 5.0;   // [deg]
    task_spec.items[2].spec.motion.rotation_tol_llim  = 5.0;   // [deg]
    task_spec.items[2].spec.motion.torque_ulim        = 8.4;   // [Nm]
    task_spec.items[2].spec.motion.torque_llim        = 0.05;   // [Nm]

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
    LOG_INFO_S(node, "ZeroConfig EE: " << fvel);

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
    
    bool wrench_bias_initialized = false;
    const int buffer_size = 20;
    double wrench_msr_bias[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    int lc = 0;
    struct taskitem_t current_task = task_spec.items[0];

    while (rclcpp::ok() && !stop_signal_received) {
        clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_start);

        // add wrench measurement to buffer
        if (!wrench_bias_initialized) {
            for (int i = 0; i < 6; i++) {
                wrench_msr_bias[i] += tool_ext_wrench_msr[i];
            }
            lc++;

            if (lc >= buffer_size) {
                for (int i = 0; i < 6; i++) wrench_msr_bias[i] /= buffer_size;

                wrench_bias_initialized = true;
                LOG_INFO(node, "Wrench bias initialized: [%f, %f, %f, %f, %f, %f]",
                         wrench_msr_bias[0], wrench_msr_bias[1], wrench_msr_bias[2],
                         wrench_msr_bias[3], wrench_msr_bias[4], wrench_msr_bias[5]);
            }
        } else {
            // apply bias correction
            for (int i = 0; i < 6; i++) {
                tool_ext_wrench_msr[i] -= wrench_msr_bias[i];
            }
        }

        for (int i = 0; i < NUM_JOINTS; i++) {
            jnt_array_vel.q(i)    = pos_msr[i];
            jnt_array_vel.qdot(i) = vel_msr[i];
        }
        fk_solver_vel.JntToCart(jnt_array_vel, fvel, -1);
        LOG_INFO_S(node, "EE FrameVel: " << fvel);


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
            usleep(cycle_time_state.cycle_time_exp - cycle_time_state.cycle_time_msr);
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

