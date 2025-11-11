#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <signal.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/kinova_gen3.h"

#include <llvm/ExecutionEngine/Orc/Mangling.h>
#include <llvm/ExecutionEngine/Orc/SymbolStringPool.h>


#define LOG_INFO(node, msg, ...) RCLCPP_INFO(node->get_logger(), msg, ##__VA_ARGS__)
#define LOG_ERROR(node, msg, ...) RCLCPP_ERROR(node->get_logger(), msg, ##__VA_ARGS__)

#define NUM_JOINTS 7


volatile sig_atomic_t stop_signal_received = 0;

using namespace llvm;
using namespace llvm::orc;

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
    
    auto urdf = "gen3_robotiq_2f_85.urdf";
    auto pkg_path = ament_index_cpp::get_package_share_directory("jit_control_arch");
    auto urdf_path = pkg_path + "/urdf/" + urdf;

    bool success = false;
    double cycle_time = 0.001;
    enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
    double pos_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double pos_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double vel_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double eff_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double cur_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double imu_ang_vel_msr[] = { 0.0, 0.0, 0.0 };
    double imu_lin_acc_msr[] = { 0.0, 0.0, 0.0 };

    struct robif2b_kinova_gen3_nbx rob = {
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
        .cycle_time      = &cycle_time,
        .ctrl_mode       = &ctrl_mode,
        .jnt_pos_msr     = pos_msr,
        .jnt_vel_msr     = vel_msr,
        .jnt_trq_msr     = eff_msr,
        .act_cur_msr     = cur_msr,
        .jnt_pos_cmd     = pos_cmd,
        .jnt_vel_cmd     = vel_cmd,
        .jnt_trq_cmd     = eff_cmd,
        .act_cur_cmd     = cur_cmd,
        .imu_ang_vel_msr = imu_ang_vel_msr,
        .imu_lin_acc_msr = imu_lin_acc_msr,
        .success         = &success
    };

    struct CycleTimeState cycle_time_state = {
        .cycle_time_exp = 1000 // [us]
    };

    robif2b_kinova_gen3_configure(&rob);
    if (!success) {
        LOG_ERROR(node, "Failed to configure Kinova Gen3 robot.");
        return -1;
    }

    robif2b_kinova_gen3_recover(&rob);
    if (!success) {
        LOG_ERROR(node, "Failed to recover Kinova Gen3 robot.");
        return -1;
    }

    LOG_INFO(node, "Starting Kinova Gen3 robot control loop.");
    robif2b_kinova_gen3_start(&rob);
    if (!success) {
        LOG_ERROR(node, "Failed to start Kinova Gen3 robot.");
        return -1;
    }
    while (rclcpp::ok() && !stop_signal_received) {
        clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_start);
        
        robif2b_kinova_gen3_update(&rob);
        if (!success) {
            LOG_ERROR(node, "Failed to update Kinova Gen3 robot.");
            break;
        }


        clock_gettime(CLOCK_MONOTONIC, &cycle_time_state.cycle_end);
        cycle_time_state.cycle_time_msr = timespec_to_usec(&cycle_time_state.cycle_end)
                                  - timespec_to_usec(&cycle_time_state.cycle_start);

        // threshold for cycle time to be >= 0
        if (cycle_time_state.cycle_time_msr < cycle_time_state.cycle_time_exp)
            usleep(cycle_time_state.cycle_time_exp - cycle_time_state.cycle_time_msr);
    }

    robif2b_kinova_gen3_stop(&rob);
    LOG_INFO(node, "Kinova Gen3 robot control loop stopped.");

    robif2b_kinova_gen3_shutdown(&rob);
    LOG_INFO(node, "Kinova Gen3 robot shutdown completed.");

    rclcpp::shutdown();
    return 0;
}

