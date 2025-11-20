#include <robif2b/types/kinova_gen3.h>
#include <signal.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>

#include <rclcpp/rclcpp.hpp>

#include "robif2b/functions/kinova_gen3.h"


#define LOG_INFO(node, msg, ...) RCLCPP_INFO(node->get_logger(), msg, ##__VA_ARGS__)
#define LOG_ERROR(node, msg, ...) RCLCPP_ERROR(node->get_logger(), msg, ##__VA_ARGS__)

#define NUM_JOINTS 7


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

    bool success = false;
    double cycle_time = 0.001;
    
    enum robif2b_hl_ctrl_mode ctrl_mode = ROBIF2B_HL_CTRL_MODE_WRENCH;
    enum robif2b_kinova_cart_ref_frame ref_frame = ROBIF2B_KINOVA_CART_REF_FRAME_MIXED;
    enum robif2b_kinova_cart_wrench_mode wrench_mode = ROBIF2B_KINOVA_CART_WRENCH_MODE_NORMAL;
    double pos_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double imu_ang_vel_msr[] = { 0.0, 0.0, 0.0 };
    double imu_lin_acc_msr[] = { 0.0, 0.0, 0.0 };
    double tool_ext_wrench_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; 

    double wrench_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 10.0 };
   
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

    robif2b_kinova_gen3_hl_update(&rob);

    
    while (rclcpp::ok() && !stop_signal_received) {
        // clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_start);

        // ---- update robot ----
        // robif2b_kinova_gen3_hl_update(&rob);
        // if (!success) {
        //     LOG_ERROR(node, "Failed to update Kinova Gen3 robot.");
        //     break;
        // }

        // clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_end);
        // cycle_time_state.cycle_time_msr = timespec_to_usec(&cycle_time_state.cycle_end)
        //                           - timespec_to_usec(&cycle_time_state.cycle_start);

        // threshold for cycle time to be >= 0
        // if (cycle_time_state.cycle_time_msr < cycle_time_state.cycle_time_exp) {
        //     usleep(cycle_time_state.cycle_time_exp - cycle_time_state.cycle_time_msr);
        // }
    }

    LOG_INFO(node, "Stopping Kinova Gen3 robot control.");
    robif2b_kinova_gen3_hl_stop(&rob);
    LOG_INFO(node, "Kinova Gen3 robot control loop stopped.");

    robif2b_kinova_gen3_hl_shutdown(&rob);
    LOG_INFO(node, "Kinova Gen3 robot shutdown completed.");

    rclcpp::shutdown();
    return 0;
}

