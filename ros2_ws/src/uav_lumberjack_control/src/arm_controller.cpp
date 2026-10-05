#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "uav_lumberjack_control/arm_kinematics.hpp"
#include "uav_lumberjack_interfaces/action/arm_motion.hpp"
#include "uav_lumberjack_interfaces/msg/arm_status.hpp"
#include "uav_lumberjack_interfaces/srv/set_saw.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
    using ArmMotion = uav_lumberjack_interfaces::action::ArmMotion;
    using GoalHandleArmMotion = rclcpp_action::ServerGoalHandle<ArmMotion>;
    using SetSaw = uav_lumberjack_interfaces::srv::SetSaw;
    using ArmStatus = uav_lumberjack_interfaces::msg::ArmStatus;

    ArmController() : Node("arm_controller")
    {
        j2_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j2/cmd_pos", 10);
        j3_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j3/cmd_pos", 10);
        j4_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j4/cmd_pos", 10);
        saw_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/saw/cmd_vel", 10);
        status_pub_ = create_publisher<ArmStatus>("/lumberjack_arm/status", 10);

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/lumberjack_arm/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&ArmController::joint_state_callback, this, std::placeholders::_1));

        arm_motion_server_ = rclcpp_action::create_server<ArmMotion>(
            this,
            "/lumberjack_arm/motion",
            std::bind(&ArmController::handle_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ArmController::handle_arm_cancel, this, std::placeholders::_1),
            std::bind(&ArmController::handle_arm_accepted, this, std::placeholders::_1));

        set_saw_service_ = create_service<SetSaw>(
            "/lumberjack_arm/set_saw",
            std::bind(
                &ArmController::handle_set_saw_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        status_timer_ = create_wall_timer(
            100ms,
            std::bind(&ArmController::publish_status_topic, this));

        RCLCPP_INFO(get_logger(), "UAV_lumberjack V2 3-DOF Front Arm + Saw Controller started.");
        RCLCPP_INFO(get_logger(), "Arm Action : /lumberjack_arm/motion");
        RCLCPP_INFO(get_logger(), "Saw Service: /lumberjack_arm/set_saw");
        RCLCPP_INFO(get_logger(), "Arm Status : /lumberjack_arm/status");
    }

    void run()
    {
        wait_for_bridge();
        wait_for_joint_feedback();
        print_startup_banner();

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(io_mutex_);
                std::cout << (busy_.load() ? "\n[ARM:BUSY] > " : "\n[ARM:IDLE] > ") << std::flush;
            }

            std::string line;
            if (!std::getline(std::cin, line)) break;
            if (line.empty()) continue;

            std::istringstream iss(line);
            std::string command;
            iss >> command;

            if (command == "status") {
                print_status();
                continue;
            }

            if (command == "help") {
                print_help();
                continue;
            }

            if (command == "fk") {
                handle_fk_command();
                continue;
            }

            if (command == "ik") {
                handle_ik_command(iss);
                continue;
            }

            if (command == "movetcp") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                handle_movetcp_command(iss);
                continue;
            }

            if (command == "saw") {
                handle_saw_command(iss);
                continue;
            }

            if (command == "quit" || command == "exit") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                publish_saw_rpm(0.0);
                safe_print("Exit arm controller. Saw OFF.");
                break;
            }

            if (command == "home") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                start_home_motion();
                continue;
            }

            if (command == "prework" || command == "ready") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                start_prework_motion();
                continue;
            }

            if (command == "init") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                start_init_motion();
                continue;
            }

            if (command == "j2" || command == "j3" || command == "j4") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }

                double angle_deg;
                if (!(iss >> angle_deg)) {
                    safe_print("Invalid command. Example: j4 45");
                    continue;
                }

                if (!check_joint_limit(command, angle_deg)) continue;
                start_joint_motion(command, angle_deg);
                continue;
            }

            safe_print("Unknown command. Type 'help' to show available commands.");
        }

        publish_saw_rpm(0.0);
    }

    void join_motion_thread()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;
    static constexpr double RPM_TO_RAD_S = 2.0 * PI / 60.0;
    static constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * PI);

    static constexpr double J2_MIN_DEG = -165.0;
    static constexpr double J2_MAX_DEG = 15.0;
    static constexpr double J3_MIN_DEG = -150.0;
    static constexpr double J3_MAX_DEG = 150.0;
    static constexpr double J4_MIN_DEG = -180.0;
    static constexpr double J4_MAX_DEG = 180.0;

    // Named operating poses
    static constexpr double HOME_J2_DEG = -30.0;
    static constexpr double HOME_J3_DEG = -150.0;
    static constexpr double HOME_J4_DEG = -90.0;

    static constexpr double PREWORK_J2_DEG = -30.0;
    static constexpr double PREWORK_J3_DEG = -60.0;
    static constexpr double PREWORK_J4_DEG = 0.0;

    // Gazebo model is physically baked in HOME at raw joint position 0.
    // User-facing logical angle = Gazebo raw angle + this offset.
    static constexpr double J2_LOGICAL_OFFSET_DEG = HOME_J2_DEG;
    static constexpr double J3_LOGICAL_OFFSET_DEG = HOME_J3_DEG;
    static constexpr double J4_LOGICAL_OFFSET_DEG = HOME_J4_DEG;
    static constexpr double SAW_MIN_RPM = 0.0;
    static constexpr double SAW_MAX_RPM = 1800.0;

    static constexpr std::uint8_t ARM_MODE_HOME = 0;
    static constexpr std::uint8_t ARM_MODE_PREWORK = 1;
    static constexpr std::uint8_t ARM_MODE_JOINT = 2;
    static constexpr std::uint8_t ARM_MODE_TCP = 3;

    static constexpr double POSITION_TOLERANCE_DEG = 1.0;
    static constexpr double VELOCITY_TOLERANCE_DEG_S = 1.0;
    static constexpr int STABLE_CYCLES_REQUIRED = 10;
    static constexpr int CHECK_PERIOD_MS = 50;
    static constexpr double MOTION_TIMEOUT_SEC = 10.0;

    // Trapezoidal velocity trajectory:
    // acceleration -> constant speed -> deceleration.
    // If a move is too short to reach vmax, it automatically becomes a triangular profile.
    static constexpr int TRAJECTORY_PERIOD_MS = 20;
    static constexpr double J2_MAX_CMD_SPEED_DEG_S = 30.0;
    static constexpr double J3_MAX_CMD_SPEED_DEG_S = 45.0;
    static constexpr double J4_MAX_CMD_SPEED_DEG_S = 60.0;

    static constexpr double J2_MAX_CMD_ACCEL_DEG_S2 = 25.0;
    static constexpr double J3_MAX_CMD_ACCEL_DEG_S2 = 35.0;
    static constexpr double J4_MAX_CMD_ACCEL_DEG_S2 = 40.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j3_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j4_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr saw_pub_;
    rclcpp::Publisher<ArmStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp_action::Server<ArmMotion>::SharedPtr arm_motion_server_;
    rclcpp::Service<SetSaw>::SharedPtr set_saw_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    double j2_cmd_deg_ = HOME_J2_DEG;
    double j3_cmd_deg_ = HOME_J3_DEG;
    double j4_cmd_deg_ = HOME_J4_DEG;
    double saw_cmd_rpm_ = 0.0;

    double j2_actual_deg_ = 0.0;
    double j3_actual_deg_ = 0.0;
    double j4_actual_deg_ = 0.0;
    double saw_actual_rpm_ = 0.0;

    double j2_velocity_deg_s_ = 0.0;
    double j3_velocity_deg_s_ = 0.0;
    double j4_velocity_deg_s_ = 0.0;

    bool j2_feedback_received_ = false;
    bool j3_feedback_received_ = false;
    bool j4_feedback_received_ = false;
    bool saw_feedback_received_ = false;

    struct TrapProfile
    {
        double start_deg = 0.0;
        double target_deg = 0.0;
        double direction = 1.0;
        double distance_deg = 0.0;
        double accel_deg_s2 = 0.0;
        double peak_speed_deg_s = 0.0;
        double t_accel = 0.0;
        double t_flat = 0.0;
        double total_time = 0.0;
        double d_accel = 0.0;
        double d_flat = 0.0;
    };

    std::atomic<bool> busy_{false};
    std::atomic<bool> cancel_requested_{false};
    std::thread motion_thread_;
    std::mutex state_mutex_;
    std::mutex io_mutex_;
    std::mutex task_mutex_;
    std::string active_task_ = "NONE";

    void safe_print(const std::string & text)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        std::cout << text << std::endl;
    }

    void set_active_task(const std::string & task)
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        active_task_ = task;
    }

    std::string get_active_task()
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        return active_task_;
    }

    void finish_task()
    {
        set_active_task("NONE");
        cancel_requested_.store(false);
        busy_.store(false);
    }

    bool try_begin_task(const std::string & task)
    {
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true)) return false;
        cancel_requested_.store(false);
        set_active_task(task);
        return true;
    }

    void reject_busy()
    {
        safe_print("[BUSY] Command rejected: current task = " + get_active_task() +
                   "\n       Available now: status | fk | ik <x> <y> <z> | help | saw off");
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const std::size_t count = std::min(msg->name.size(), msg->position.size());

        for (std::size_t i = 0; i < count; ++i) {
            const double raw_position_deg = msg->position[i] * RAD_TO_DEG;
            double velocity_rad_s = 0.0;
            if (i < msg->velocity.size()) velocity_rad_s = msg->velocity[i];
            const double velocity_deg_s = velocity_rad_s * RAD_TO_DEG;

            if (msg->name[i] == "j2") {
                j2_actual_deg_ = raw_position_deg + J2_LOGICAL_OFFSET_DEG;
                j2_velocity_deg_s_ = velocity_deg_s;
                j2_feedback_received_ = true;
            } else if (msg->name[i] == "j3") {
                j3_actual_deg_ = raw_position_deg + J3_LOGICAL_OFFSET_DEG;
                j3_velocity_deg_s_ = velocity_deg_s;
                j3_feedback_received_ = true;
            } else if (msg->name[i] == "j4") {
                j4_actual_deg_ = raw_position_deg + J4_LOGICAL_OFFSET_DEG;
                j4_velocity_deg_s_ = velocity_deg_s;
                j4_feedback_received_ = true;
            } else if (msg->name[i] == "saw_spin_joint") {
                saw_actual_rpm_ = velocity_rad_s * RAD_S_TO_RPM;
                saw_feedback_received_ = true;
            }
        }
    }

    void wait_for_bridge()
    {
        safe_print("[STARTUP] Waiting for ros_gz_bridge command channels...");

        while (rclcpp::ok()) {
            const bool j2_ready = j2_pub_->get_subscription_count() > 0;
            const bool j3_ready = j3_pub_->get_subscription_count() > 0;
            const bool j4_ready = j4_pub_->get_subscription_count() > 0;
            const bool saw_ready = saw_pub_->get_subscription_count() > 0;

            if (j2_ready && j3_ready && j4_ready && saw_ready) {
                safe_print("[STARTUP] Command bridges: READY (J2/J3/J4/Saw)");
                return;
            }
            std::this_thread::sleep_for(500ms);
        }
    }

    void wait_for_joint_feedback()
    {
        safe_print("[STARTUP] Waiting for joint state feedback...");

        while (rclcpp::ok()) {
            bool ready = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ready = j2_feedback_received_ && j3_feedback_received_ &&
                        j4_feedback_received_ && saw_feedback_received_;
            }

            if (ready) {
                safe_print("[STARTUP] Joint feedback: READY (J2/J3/J4/Saw)");
                return;
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    bool check_joint_limit(const std::string & joint, double angle_deg)
    {
        double min_deg = 0.0;
        double max_deg = 0.0;

        if (joint == "j2") {
            min_deg = J2_MIN_DEG;
            max_deg = J2_MAX_DEG;
        } else if (joint == "j3") {
            min_deg = J3_MIN_DEG;
            max_deg = J3_MAX_DEG;
        } else if (joint == "j4") {
            min_deg = J4_MIN_DEG;
            max_deg = J4_MAX_DEG;
        } else {
            safe_print("Unknown joint: " + joint);
            return false;
        }

        if (angle_deg < min_deg || angle_deg > max_deg) {
            std::ostringstream oss;
            oss << "Rejected: " << joint << " angle must be within ["
                << min_deg << ", " << max_deg << "] deg.";
            safe_print(oss.str());
            return false;
        }
        return true;
    }

    double get_commanded_angle(const std::string & joint)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (joint == "j2") return j2_cmd_deg_;
        if (joint == "j3") return j3_cmd_deg_;
        if (joint == "j4") return j4_cmd_deg_;
        return 0.0;
    }

    void publish_joint(const std::string & joint, double angle_deg)
    {
        double raw_angle_deg = angle_deg;
        if (joint == "j2") raw_angle_deg = angle_deg - J2_LOGICAL_OFFSET_DEG;
        else if (joint == "j3") raw_angle_deg = angle_deg - J3_LOGICAL_OFFSET_DEG;
        else if (joint == "j4") raw_angle_deg = angle_deg - J4_LOGICAL_OFFSET_DEG;

        std_msgs::msg::Float64 msg;
        msg.data = raw_angle_deg * DEG_TO_RAD;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (joint == "j2") j2_cmd_deg_ = angle_deg;
            else if (joint == "j3") j3_cmd_deg_ = angle_deg;
            else if (joint == "j4") j4_cmd_deg_ = angle_deg;
        }

        if (joint == "j2") j2_pub_->publish(msg);
        else if (joint == "j3") j3_pub_->publish(msg);
        else if (joint == "j4") j4_pub_->publish(msg);
    }

    void publish_saw_rpm(double rpm)
    {
        std_msgs::msg::Float64 msg;
        msg.data = rpm * RPM_TO_RAD_S;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            saw_cmd_rpm_ = rpm;
        }

        saw_pub_->publish(msg);
    }

    void handle_saw_command(std::istringstream & iss)
    {
        std::string value;
        if (!(iss >> value)) {
            safe_print("Invalid command. Example: saw 500   or   saw off");
            return;
        }

        if (value == "off") {
            publish_saw_rpm(0.0);
            safe_print("[SAW] OFF");
            return;
        }

        std::istringstream value_stream(value);
        double rpm = 0.0;
        char extra = '\0';
        if (!(value_stream >> rpm) || (value_stream >> extra)) {
            safe_print("Invalid saw speed. Example: saw 500");
            return;
        }

        if (rpm < SAW_MIN_RPM || rpm > SAW_MAX_RPM) {
            std::ostringstream oss;
            oss << "Rejected: saw speed must be within [" << SAW_MIN_RPM
                << ", " << SAW_MAX_RPM << "] rpm.";
            safe_print(oss.str());
            return;
        }

        if (busy_.load() && rpm > 0.0) {
            safe_print("[REJECTED] Saw start is disabled while arm is BUSY. 'saw off' is always allowed.");
            return;
        }

        publish_saw_rpm(rpm);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "[SAW] Command = " << rpm << " rpm ("
            << rpm * RPM_TO_RAD_S << " rad/s)";
        safe_print(oss.str());
    }

    bool get_joint_feedback(const std::string & joint, double & actual_deg, double & velocity_deg_s)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (joint == "j2") {
            if (!j2_feedback_received_) return false;
            actual_deg = j2_actual_deg_;
            velocity_deg_s = j2_velocity_deg_s_;
            return true;
        }
        if (joint == "j3") {
            if (!j3_feedback_received_) return false;
            actual_deg = j3_actual_deg_;
            velocity_deg_s = j3_velocity_deg_s_;
            return true;
        }
        if (joint == "j4") {
            if (!j4_feedback_received_) return false;
            actual_deg = j4_actual_deg_;
            velocity_deg_s = j4_velocity_deg_s_;
            return true;
        }
        return false;
    }

    bool wait_until_joint_reached(const std::string & joint, double target_deg)
    {
        int stable_cycles = 0;
        const auto start_time = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            double actual_deg = 0.0;
            double velocity_deg_s = 0.0;

            if (!get_joint_feedback(joint, actual_deg, velocity_deg_s)) {
                safe_print("[ERROR] " + joint + " feedback unavailable.");
                return false;
            }

            const double error_deg = target_deg - actual_deg;
            if (std::abs(error_deg) <= POSITION_TOLERANCE_DEG &&
                std::abs(velocity_deg_s) <= VELOCITY_TOLERANCE_DEG_S) {
                stable_cycles++;
            } else {
                stable_cycles = 0;
            }

            if (stable_cycles >= STABLE_CYCLES_REQUIRED) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "[REACHED] " << joint
                    << " target = " << target_deg
                    << " deg, actual = " << actual_deg
                    << " deg, error = " << error_deg << " deg";
                safe_print(oss.str());
                return true;
            }

            const auto now = std::chrono::steady_clock::now();
            const double elapsed_sec = std::chrono::duration<double>(now - start_time).count();
            if (elapsed_sec >= MOTION_TIMEOUT_SEC) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "[TIMEOUT] " << joint << " failed to reach target.\n"
                    << "Target   = " << target_deg << " deg\n"
                    << "Actual   = " << actual_deg << " deg\n"
                    << "Error    = " << error_deg << " deg\n"
                    << "Velocity = " << velocity_deg_s << " deg/s";
                safe_print(oss.str());
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_PERIOD_MS));
        }
        return false;
    }

    bool wait_until_home_reached()
    {
        int stable_cycles = 0;
        const auto start_time = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            double j2_actual, j3_actual, j4_actual;
            double j2_velocity, j3_velocity, j4_velocity;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                j2_actual = j2_actual_deg_;
                j3_actual = j3_actual_deg_;
                j4_actual = j4_actual_deg_;
                j2_velocity = j2_velocity_deg_s_;
                j3_velocity = j3_velocity_deg_s_;
                j4_velocity = j4_velocity_deg_s_;
            }

            const bool j2_ok = std::abs(HOME_J2_DEG - j2_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j2_velocity) <= VELOCITY_TOLERANCE_DEG_S;
            const bool j3_ok = std::abs(HOME_J3_DEG - j3_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j3_velocity) <= VELOCITY_TOLERANCE_DEG_S;
            const bool j4_ok = std::abs(HOME_J4_DEG - j4_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j4_velocity) <= VELOCITY_TOLERANCE_DEG_S;

            if (j2_ok && j3_ok && j4_ok) stable_cycles++;
            else stable_cycles = 0;

            if (stable_cycles >= STABLE_CYCLES_REQUIRED) {
                safe_print("[REACHED] HOME");
                return true;
            }

            const auto now = std::chrono::steady_clock::now();
            const double elapsed_sec = std::chrono::duration<double>(now - start_time).count();

            if (elapsed_sec >= MOTION_TIMEOUT_SEC) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "[TIMEOUT] HOME failed.\n"
                    << "J2 target/actual = " << HOME_J2_DEG << " / " << j2_actual << " deg\n"
                    << "J3 target/actual = " << HOME_J3_DEG << " / " << j3_actual << " deg\n"
                    << "J4 target/actual = " << HOME_J4_DEG << " / " << j4_actual << " deg";
                safe_print(oss.str());
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_PERIOD_MS));
        }
        return false;
    }

    double get_joint_speed_limit_deg_s(const std::string & joint) const
    {
        if (joint == "j2") return J2_MAX_CMD_SPEED_DEG_S;
        if (joint == "j3") return J3_MAX_CMD_SPEED_DEG_S;
        if (joint == "j4") return J4_MAX_CMD_SPEED_DEG_S;
        return J2_MAX_CMD_SPEED_DEG_S;
    }

    double get_joint_accel_limit_deg_s2(const std::string & joint) const
    {
        if (joint == "j2") return J2_MAX_CMD_ACCEL_DEG_S2;
        if (joint == "j3") return J3_MAX_CMD_ACCEL_DEG_S2;
        if (joint == "j4") return J4_MAX_CMD_ACCEL_DEG_S2;
        return J2_MAX_CMD_ACCEL_DEG_S2;
    }

    TrapProfile make_trapezoid(
        double start_deg,
        double target_deg,
        double vmax_deg_s,
        double amax_deg_s2) const
    {
        TrapProfile p;
        p.start_deg = start_deg;
        p.target_deg = target_deg;

        const double delta = target_deg - start_deg;
        p.direction = (delta >= 0.0) ? 1.0 : -1.0;
        p.distance_deg = std::abs(delta);
        p.accel_deg_s2 = amax_deg_s2;

        if (p.distance_deg <= 1e-9) return p;

        const double t_to_vmax = vmax_deg_s / amax_deg_s2;
        const double d_accel_to_vmax = 0.5 * amax_deg_s2 * t_to_vmax * t_to_vmax;

        if (p.distance_deg >= 2.0 * d_accel_to_vmax) {
            p.peak_speed_deg_s = vmax_deg_s;
            p.t_accel = t_to_vmax;
            p.d_accel = d_accel_to_vmax;
            p.d_flat = p.distance_deg - 2.0 * p.d_accel;
            p.t_flat = p.d_flat / p.peak_speed_deg_s;
        } else {
            p.t_accel = std::sqrt(p.distance_deg / amax_deg_s2);
            p.peak_speed_deg_s = amax_deg_s2 * p.t_accel;
            p.d_accel = 0.5 * amax_deg_s2 * p.t_accel * p.t_accel;
            p.d_flat = 0.0;
            p.t_flat = 0.0;
        }

        p.total_time = 2.0 * p.t_accel + p.t_flat;
        return p;
    }

    double sample_trapezoid(const TrapProfile & p, double t_sec) const
    {
        if (p.distance_deg <= 1e-9) return p.target_deg;
        if (t_sec <= 0.0) return p.start_deg;
        if (t_sec >= p.total_time) return p.target_deg;

        double traveled = 0.0;

        if (t_sec < p.t_accel) {
            traveled = 0.5 * p.accel_deg_s2 * t_sec * t_sec;
        } else if (t_sec < p.t_accel + p.t_flat) {
            const double t = t_sec - p.t_accel;
            traveled = p.d_accel + p.peak_speed_deg_s * t;
        } else {
            const double t = t_sec - p.t_accel - p.t_flat;
            traveled = p.d_accel + p.d_flat +
                       p.peak_speed_deg_s * t -
                       0.5 * p.accel_deg_s2 * t * t;
        }

        traveled = std::clamp(traveled, 0.0, p.distance_deg);
        return p.start_deg + p.direction * traveled;
    }

    std::string profile_type(const TrapProfile & p) const
    {
        return (p.t_flat > 1e-9) ? "trapezoid" : "triangle";
    }

    bool publish_joint_trapezoid(const std::string & joint, double target_deg)
    {
        double start_deg = 0.0;
        double start_vel = 0.0;
        if (!get_joint_feedback(joint, start_deg, start_vel)) {
            safe_print("[ERROR] " + joint + " feedback unavailable before trajectory.");
            return false;
        }

        const double vmax = get_joint_speed_limit_deg_s(joint);
        const double amax = get_joint_accel_limit_deg_s2(joint);
        const TrapProfile profile = make_trapezoid(start_deg, target_deg, vmax, amax);

        if (profile.distance_deg <= 1e-9) {
            publish_joint(joint, target_deg);
            return true;
        }

        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(profile.total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            const double t = std::min(i * dt_sec, profile.total_time);
            publish_joint(joint, sample_trapezoid(profile, t));
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint(joint, target_deg);
        return true;
    }

    bool publish_pose_trapezoid(
        double target_q2_deg,
        double target_q3_deg,
        double target_q4_deg)
    {
        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        double v = 0.0;
        if (!get_joint_feedback("j2", q2, v) || !get_joint_feedback("j3", q3, v) ||
            !get_joint_feedback("j4", q4, v)) {
            safe_print("[ERROR] Joint feedback unavailable before TCP trajectory.");
            return false;
        }

        const TrapProfile p2 = make_trapezoid(
            q2, target_q2_deg, J2_MAX_CMD_SPEED_DEG_S, J2_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p3 = make_trapezoid(
            q3, target_q3_deg, J3_MAX_CMD_SPEED_DEG_S, J3_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p4 = make_trapezoid(
            q4, target_q4_deg, J4_MAX_CMD_SPEED_DEG_S, J4_MAX_CMD_ACCEL_DEG_S2);

        const double total_time = std::max(p2.total_time, std::max(p3.total_time, p4.total_time));
        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            const double t = i * dt_sec;
            publish_joint("j2", sample_trapezoid(p2, std::min(t, p2.total_time)));
            publish_joint("j3", sample_trapezoid(p3, std::min(t, p3.total_time)));
            publish_joint("j4", sample_trapezoid(p4, std::min(t, p4.total_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint("j2", target_q2_deg);
        publish_joint("j3", target_q3_deg);
        publish_joint("j4", target_q4_deg);
        return true;
    }

    bool wait_until_pose_reached(
        double target_q2_deg,
        double target_q3_deg,
        double target_q4_deg)
    {
        return wait_until_joint_reached("j2", target_q2_deg) &&
               wait_until_joint_reached("j3", target_q3_deg) &&
               wait_until_joint_reached("j4", target_q4_deg);
    }

    bool publish_home_trapezoid()
    {
        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        double v = 0.0;
        if (!get_joint_feedback("j2", q2, v) || !get_joint_feedback("j3", q3, v) ||
            !get_joint_feedback("j4", q4, v)) {
            safe_print("[ERROR] Joint feedback unavailable before HOME trajectory.");
            return false;
        }

        const TrapProfile p2 = make_trapezoid(q2, HOME_J2_DEG, J2_MAX_CMD_SPEED_DEG_S, J2_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p3 = make_trapezoid(q3, HOME_J3_DEG, J3_MAX_CMD_SPEED_DEG_S, J3_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p4 = make_trapezoid(q4, HOME_J4_DEG, J4_MAX_CMD_SPEED_DEG_S, J4_MAX_CMD_ACCEL_DEG_S2);

        const double total_time = std::max(p2.total_time, std::max(p3.total_time, p4.total_time));
        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            const double t = i * dt_sec;
            publish_joint("j2", sample_trapezoid(p2, std::min(t, p2.total_time)));
            publish_joint("j3", sample_trapezoid(p3, std::min(t, p3.total_time)));
            publish_joint("j4", sample_trapezoid(p4, std::min(t, p4.total_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint("j2", HOME_J2_DEG);
        publish_joint("j3", HOME_J3_DEG);
        publish_joint("j4", HOME_J4_DEG);
        return true;
    }

    bool publish_zero_trapezoid()
    {
        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        double v = 0.0;
        if (!get_joint_feedback("j2", q2, v) || !get_joint_feedback("j3", q3, v) ||
            !get_joint_feedback("j4", q4, v)) {
            safe_print("[ERROR] Joint feedback unavailable before ZERO trajectory.");
            return false;
        }

        const TrapProfile p2 = make_trapezoid(q2, 0.0, J2_MAX_CMD_SPEED_DEG_S, J2_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p3 = make_trapezoid(q3, 0.0, J3_MAX_CMD_SPEED_DEG_S, J3_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p4 = make_trapezoid(q4, 0.0, J4_MAX_CMD_SPEED_DEG_S, J4_MAX_CMD_ACCEL_DEG_S2);

        const double total_time = std::max(p2.total_time, std::max(p3.total_time, p4.total_time));
        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            const double t = i * dt_sec;
            publish_joint("j2", sample_trapezoid(p2, std::min(t, p2.total_time)));
            publish_joint("j3", sample_trapezoid(p3, std::min(t, p3.total_time)));
            publish_joint("j4", sample_trapezoid(p4, std::min(t, p4.total_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint("j2", 0.0);
        publish_joint("j3", 0.0);
        publish_joint("j4", 0.0);
        return true;
    }

    bool wait_until_zero_reached()
    {
        return wait_until_joint_reached("j2", 0.0) &&
               wait_until_joint_reached("j3", 0.0) &&
               wait_until_joint_reached("j4", 0.0);
    }

    bool publish_prework_trapezoid()
    {
        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        double v = 0.0;
        if (!get_joint_feedback("j2", q2, v) || !get_joint_feedback("j3", q3, v) ||
            !get_joint_feedback("j4", q4, v)) {
            safe_print("[ERROR] Joint feedback unavailable before PREWORK trajectory.");
            return false;
        }

        const TrapProfile p2 = make_trapezoid(q2, PREWORK_J2_DEG, J2_MAX_CMD_SPEED_DEG_S, J2_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p3 = make_trapezoid(q3, PREWORK_J3_DEG, J3_MAX_CMD_SPEED_DEG_S, J3_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p4 = make_trapezoid(q4, PREWORK_J4_DEG, J4_MAX_CMD_SPEED_DEG_S, J4_MAX_CMD_ACCEL_DEG_S2);

        const double total_time = std::max(p2.total_time, std::max(p3.total_time, p4.total_time));
        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            const double t = i * dt_sec;
            publish_joint("j2", sample_trapezoid(p2, std::min(t, p2.total_time)));
            publish_joint("j3", sample_trapezoid(p3, std::min(t, p3.total_time)));
            publish_joint("j4", sample_trapezoid(p4, std::min(t, p4.total_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint("j2", PREWORK_J2_DEG);
        publish_joint("j3", PREWORK_J3_DEG);
        publish_joint("j4", PREWORK_J4_DEG);
        return true;
    }

    bool wait_until_prework_reached()
    {
        return wait_until_joint_reached("j2", PREWORK_J2_DEG) &&
               wait_until_joint_reached("j3", PREWORK_J3_DEG) &&
               wait_until_joint_reached("j4", PREWORK_J4_DEG);
    }

    bool move_joint_sync(const std::string & joint, double target_deg)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "[TEST] " << joint << " -> " << target_deg
            << " deg  [trapezoid, vmax " << get_joint_speed_limit_deg_s(joint)
            << " deg/s, amax " << get_joint_accel_limit_deg_s2(joint) << " deg/s^2]";
        safe_print(oss.str());
        if (!publish_joint_trapezoid(joint, target_deg)) return false;
        return wait_until_joint_reached(joint, target_deg);
    }

    bool home_all_sync()
    {
        publish_saw_rpm(0.0);
        if (!publish_home_trapezoid()) return false;
        return wait_until_home_reached();
    }

    void start_joint_motion(const std::string & joint, double target_deg)
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        if (!try_begin_task("JOINT_" + joint)) {
            reject_busy();
            return;
        }

        motion_thread_ = std::thread([this, joint, target_deg]() {
            double actual_deg = 0.0;
            double actual_vel = 0.0;
            get_joint_feedback(joint, actual_deg, actual_vel);

            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << "[BUSY] " << joint << ": " << actual_deg
                << " deg -> " << target_deg << " deg"
                << "  [trapezoid, vmax " << get_joint_speed_limit_deg_s(joint)
                << " deg/s, amax " << get_joint_accel_limit_deg_s2(joint) << " deg/s^2]";
            safe_print(oss.str());

            bool reached = false;
            if (publish_joint_trapezoid(joint, target_deg)) {
                reached = wait_until_joint_reached(joint, target_deg);
            }
            safe_print(reached ? "Done. [IDLE]" : "Motion ended with timeout/error. [IDLE]");
            finish_task();
        });
    }

    void start_tcp_motion(
        double target_x,
        double target_y,
        double target_z,
        double target_q2_deg,
        double target_q3_deg,
        double target_q4_deg)
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        if (!try_begin_task("MOVETCP")) {
            reject_busy();
            return;
        }

        motion_thread_ = std::thread([
            this, target_x, target_y, target_z,
            target_q2_deg, target_q3_deg, target_q4_deg]() {
            std::ostringstream start_oss;
            start_oss << std::fixed << std::setprecision(3)
                      << "[BUSY] MOVETCP target [m] = ["
                      << target_x << ", " << target_y << ", " << target_z << "]\n"
                      << "       IK joint target [deg] = ["
                      << target_q2_deg << ", " << target_q3_deg << ", " << target_q4_deg << "]";
            safe_print(start_oss.str());

            // Keep the saw stopped during arm repositioning.
            publish_saw_rpm(0.0);

            bool reached = false;
            if (publish_pose_trapezoid(target_q2_deg, target_q3_deg, target_q4_deg)) {
                reached = wait_until_pose_reached(
                    target_q2_deg, target_q3_deg, target_q4_deg);
            }

            if (reached) {
                double q2_deg = 0.0, q3_deg = 0.0, q4_deg = 0.0;
                if (get_current_joint_angles_deg(q2_deg, q3_deg, q4_deg)) {
                    const auto q = uav_lumberjack_control::ArmKinematics::degreesToRadians(
                        q2_deg, q3_deg, q4_deg);
                    const auto pose = uav_lumberjack_control::ArmKinematics::forward(q);
                    const double dx = pose.position.x - target_x;
                    const double dy = pose.position.y - target_y;
                    const double dz = pose.position.z - target_z;
                    const double tcp_error = std::sqrt(dx * dx + dy * dy + dz * dz);

                    std::ostringstream end_oss;
                    end_oss << std::fixed << std::setprecision(6)
                            << "[MOVETCP] Reached joint target. Actual TCP [m] = ["
                            << pose.position.x << ", " << pose.position.y << ", "
                            << pose.position.z << "]\n"
                            << "          target error = " << tcp_error << " m";
                    safe_print(end_oss.str());
                }
                safe_print("MOVETCP reached. [IDLE]");
            } else {
                safe_print("MOVETCP ended with timeout/error. [IDLE]");
            }

            finish_task();
        });
    }

    void start_home_motion()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        if (!try_begin_task("HOME")) {
            reject_busy();
            return;
        }

        motion_thread_ = std::thread([this]() {
            safe_print("[BUSY] Saw OFF, moving to TAKEOFF/LANDING HOME pose...");
            const bool reached = home_all_sync();
            safe_print(reached ? "HOME reached. [IDLE]" : "HOME ended with timeout/error. [IDLE]");
            finish_task();
        });
    }

    void start_prework_motion()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        if (!try_begin_task("PREWORK")) {
            reject_busy();
            return;
        }

        motion_thread_ = std::thread([this]() {
            safe_print("[BUSY] Saw OFF, moving to PREWORK pose...");
            publish_saw_rpm(0.0);

            bool reached = false;
            if (publish_prework_trapezoid()) {
                reached = wait_until_prework_reached();
            }

            safe_print(reached ? "PREWORK reached. [IDLE]" : "PREWORK ended with timeout/error. [IDLE]");
            finish_task();
        });
    }

    void start_init_motion()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        if (!try_begin_task("INIT")) {
            reject_busy();
            return;
        }

        motion_thread_ = std::thread([this]() {
            safe_print("[INIT] Joint full-range check starting...");
            publish_saw_rpm(0.0);

            bool ok = true;

            // Start from a neutral logical pose so each joint can be checked independently.
            safe_print("[INIT] Step 0: J2/J3/J4 -> 0 deg / Saw OFF");
            if (ok) ok = publish_zero_trapezoid();
            if (ok) ok = wait_until_zero_reached();

            // J2 full logical range: +15 -> -165 -> 0.
            if (ok) safe_print("[INIT] J2 full range: 0 -> +15 -> -165 -> 0 deg");
            if (ok) ok = move_joint_sync("j2", J2_MAX_DEG);
            if (ok) ok = move_joint_sync("j2", J2_MIN_DEG);
            if (ok) ok = move_joint_sync("j2", 0.0);

            // J3 full logical range: +150 -> -150 -> 0.
            if (ok) safe_print("[INIT] J3 full range: 0 -> +150 -> -150 -> 0 deg");
            if (ok) ok = move_joint_sync("j3", J3_MAX_DEG);
            if (ok) ok = move_joint_sync("j3", J3_MIN_DEG);
            if (ok) ok = move_joint_sync("j3", 0.0);

            // J4 full logical range: +180 -> -180 -> 0.
            if (ok) safe_print("[INIT] J4 full range: 0 -> +180 -> -180 -> 0 deg");
            if (ok) ok = move_joint_sync("j4", J4_MAX_DEG);
            if (ok) ok = move_joint_sync("j4", J4_MIN_DEG);
            if (ok) ok = move_joint_sync("j4", 0.0);

            if (!ok) safe_print("[INIT] A joint-range step failed. Attempting HOME anyway...");
            else safe_print("[INIT] Joint full-range check complete. Returning to HOME...");

            const bool home_ok = home_all_sync();
            if (ok && home_ok) safe_print("[INIT] PASS - all joint ranges checked, HOME reached. [IDLE]");
            else if (home_ok) safe_print("[INIT] END - a range check had an error, but HOME was recovered. [IDLE]");
            else safe_print("[INIT] FAIL - HOME recovery also failed. [IDLE]");
            finish_task();
        });
    }

    bool get_current_joint_angles_deg(double & q2_deg, double & q3_deg, double & q4_deg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!j2_feedback_received_ || !j3_feedback_received_ || !j4_feedback_received_) {
            return false;
        }
        q2_deg = j2_actual_deg_;
        q3_deg = j3_actual_deg_;
        q4_deg = j4_actual_deg_;
        return true;
    }

    void handle_fk_command()
    {
        double q2_deg = 0.0;
        double q3_deg = 0.0;
        double q4_deg = 0.0;
        if (!get_current_joint_angles_deg(q2_deg, q3_deg, q4_deg)) {
            safe_print("[FK] Joint feedback unavailable.");
            return;
        }

        const auto q = uav_lumberjack_control::ArmKinematics::degreesToRadians(
            q2_deg, q3_deg, q4_deg);
        const auto pose = uav_lumberjack_control::ArmKinematics::forward(q);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "\n[FK] Current TCP from actual joint feedback\n"
            << "  q [deg]        : [" << q2_deg << ", " << q3_deg << ", " << q4_deg << "]\n"
            << "  TCP position m : [" << pose.position.x << ", "
            << pose.position.y << ", " << pose.position.z << "]\n"
            << "  TCP rotation R :\n"
            << "    [" << std::setw(10) << pose.rotation(0, 0) << ", "
            << std::setw(10) << pose.rotation(0, 1) << ", "
            << std::setw(10) << pose.rotation(0, 2) << "]\n"
            << "    [" << std::setw(10) << pose.rotation(1, 0) << ", "
            << std::setw(10) << pose.rotation(1, 1) << ", "
            << std::setw(10) << pose.rotation(1, 2) << "]\n"
            << "    [" << std::setw(10) << pose.rotation(2, 0) << ", "
            << std::setw(10) << pose.rotation(2, 1) << ", "
            << std::setw(10) << pose.rotation(2, 2) << "]";
        safe_print(oss.str());
    }

    void handle_ik_command(std::istringstream & iss)
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        char extra = '\0';
        if (!(iss >> x >> y >> z) || (iss >> extra)) {
            safe_print("Invalid IK command. Example: ik 0.155885 0.030000 -0.455000");
            return;
        }

        double current_q2_deg = 0.0;
        double current_q3_deg = 0.0;
        double current_q4_deg = 0.0;
        const bool have_feedback = get_current_joint_angles_deg(
            current_q2_deg, current_q3_deg, current_q4_deg);

        const uav_lumberjack_control::Vec3 target{x, y, z};
        const auto solutions = uav_lumberjack_control::ArmKinematics::inversePosition(target);

        if (solutions.empty()) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << "[IK] No geometric solution for TCP position ["
                << x << ", " << y << ", " << z << "] m.";
            safe_print(oss.str());
            return;
        }

        std::size_t best_index = solutions.size();
        double best_joint_distance_deg = std::numeric_limits<double>::infinity();

        if (have_feedback) {
            for (std::size_t i = 0; i < solutions.size(); ++i) {
                if (!solutions[i].within_joint_limits) continue;
                const auto q_deg = uav_lumberjack_control::ArmKinematics::radiansToDegrees(
                    solutions[i].q);
                const double d2 = q_deg.q2 - current_q2_deg;
                const double d3 = q_deg.q3 - current_q3_deg;
                const double d4 = q_deg.q4 - current_q4_deg;
                const double distance = std::sqrt(d2 * d2 + d3 * d3 + d4 * d4);
                if (distance < best_joint_distance_deg) {
                    best_joint_distance_deg = distance;
                    best_index = i;
                }
            }
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "\n[IK] TCP position target [m]: ["
            << x << ", " << y << ", " << z << "]\n";

        if (have_feedback) {
            oss << std::setprecision(3)
                << "  current q [deg] : [" << current_q2_deg << ", "
                << current_q3_deg << ", " << current_q4_deg << "]\n";
        } else {
            oss << "  current q       : feedback unavailable; nearest-solution ranking disabled\n";
        }

        std::size_t valid_count = 0;
        for (std::size_t i = 0; i < solutions.size(); ++i) {
            const auto q_deg = uav_lumberjack_control::ArmKinematics::radiansToDegrees(
                solutions[i].q);
            if (solutions[i].within_joint_limits) ++valid_count;

            oss << std::setprecision(3)
                << "  candidate " << (i + 1)
                << ": q2=" << q_deg.q2
                << " deg, q3=" << q_deg.q3
                << " deg, q4=" << q_deg.q4 << " deg"
                << (solutions[i].within_joint_limits ? "  [VALID]" : "  [OUT OF LIMITS]")
                << std::setprecision(9)
                << "  pos_err=" << solutions[i].position_error_m << " m";

            if (i == best_index) {
                oss << "  <-- RECOMMENDED (nearest to current pose)";
            }
            oss << "\n";
        }

        oss << "  valid candidates : " << valid_count << " / " << solutions.size() << "\n"
            << "  NOTE             : calculation only; no arm motion is commanded.";

        safe_print(oss.str());
    }

    void handle_movetcp_command(std::istringstream & iss)
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        char extra = '\0';
        if (!(iss >> x >> y >> z) || (iss >> extra)) {
            safe_print("Invalid MOVETCP command. Example: movetcp 0.155885 0.030000 -0.455000");
            return;
        }

        double current_q2_deg = 0.0;
        double current_q3_deg = 0.0;
        double current_q4_deg = 0.0;
        if (!get_current_joint_angles_deg(
                current_q2_deg, current_q3_deg, current_q4_deg)) {
            safe_print("[MOVETCP] Joint feedback unavailable; motion rejected.");
            return;
        }

        const uav_lumberjack_control::Vec3 target{x, y, z};
        const auto solutions = uav_lumberjack_control::ArmKinematics::inversePosition(target);
        if (solutions.empty()) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << "[MOVETCP] No geometric IK solution for TCP target ["
                << x << ", " << y << ", " << z << "] m.";
            safe_print(oss.str());
            return;
        }

        std::size_t best_index = solutions.size();
        double best_joint_distance_deg = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < solutions.size(); ++i) {
            if (!solutions[i].within_joint_limits) continue;
            const auto q_deg = uav_lumberjack_control::ArmKinematics::radiansToDegrees(
                solutions[i].q);
            const double d2 = q_deg.q2 - current_q2_deg;
            const double d3 = q_deg.q3 - current_q3_deg;
            const double d4 = q_deg.q4 - current_q4_deg;
            const double distance = std::sqrt(d2 * d2 + d3 * d3 + d4 * d4);
            if (distance < best_joint_distance_deg) {
                best_joint_distance_deg = distance;
                best_index = i;
            }
        }

        if (best_index >= solutions.size()) {
            safe_print("[MOVETCP] IK candidates exist, but none satisfy the joint limits. Motion rejected.");
            return;
        }

        const auto q_deg = uav_lumberjack_control::ArmKinematics::radiansToDegrees(
            solutions[best_index].q);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "\n[MOVETCP] TCP target [m]: ["
            << x << ", " << y << ", " << z << "]\n"
            << std::setprecision(3)
            << "  current q [deg] : [" << current_q2_deg << ", "
            << current_q3_deg << ", " << current_q4_deg << "]\n"
            << "  selected q [deg]: [" << q_deg.q2 << ", "
            << q_deg.q3 << ", " << q_deg.q4 << "]\n"
            << "  selection       : nearest valid IK solution in joint space\n"
            << "  safety          : joint limits only; NO collision checking in V1";
        safe_print(oss.str());

        start_tcp_motion(x, y, z, q_deg.q2, q_deg.q3, q_deg.q4);
    }

    void publish_status_topic()
    {
        double j2_cmd = 0.0, j3_cmd = 0.0, j4_cmd = 0.0;
        double j2_actual = 0.0, j3_actual = 0.0, j4_actual = 0.0;
        double j2_vel = 0.0, j3_vel = 0.0, j4_vel = 0.0;
        double saw_cmd = 0.0, saw_actual = 0.0;
        bool feedback_ready = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            j2_cmd = j2_cmd_deg_;
            j3_cmd = j3_cmd_deg_;
            j4_cmd = j4_cmd_deg_;
            j2_actual = j2_actual_deg_;
            j3_actual = j3_actual_deg_;
            j4_actual = j4_actual_deg_;
            j2_vel = j2_velocity_deg_s_;
            j3_vel = j3_velocity_deg_s_;
            j4_vel = j4_velocity_deg_s_;
            saw_cmd = saw_cmd_rpm_;
            saw_actual = saw_actual_rpm_;
            feedback_ready = j2_feedback_received_ && j3_feedback_received_ && j4_feedback_received_;
        }

        if (!feedback_ready) return;

        const auto q = uav_lumberjack_control::ArmKinematics::degreesToRadians(
            j2_actual, j3_actual, j4_actual);
        const auto pose = uav_lumberjack_control::ArmKinematics::forward(q);

        ArmStatus msg;
        msg.header.stamp = now();
        msg.header.frame_id = "base_link";
        msg.busy = busy_.load();
        msg.task = get_active_task();
        msg.joint_command_deg = {j2_cmd, j3_cmd, j4_cmd};
        msg.joint_actual_deg = {j2_actual, j3_actual, j4_actual};
        msg.joint_velocity_deg_s = {j2_vel, j3_vel, j4_vel};
        msg.saw_command_rpm = saw_cmd;
        msg.saw_actual_rpm = saw_actual;
        msg.tcp_position.x = pose.position.x;
        msg.tcp_position.y = pose.position.y;
        msg.tcp_position.z = pose.position.z;
        status_pub_->publish(msg);
    }

    bool joints_within_limits(double q2_deg, double q3_deg, double q4_deg) const
    {
        return q2_deg >= J2_MIN_DEG && q2_deg <= J2_MAX_DEG &&
               q3_deg >= J3_MIN_DEG && q3_deg <= J3_MAX_DEG &&
               q4_deg >= J4_MIN_DEG && q4_deg <= J4_MAX_DEG;
    }

    bool choose_nearest_tcp_ik(
        double x,
        double y,
        double z,
        double & target_q2_deg,
        double & target_q3_deg,
        double & target_q4_deg)
    {
        double current_q2_deg = 0.0;
        double current_q3_deg = 0.0;
        double current_q4_deg = 0.0;
        if (!get_current_joint_angles_deg(
                current_q2_deg, current_q3_deg, current_q4_deg)) {
            return false;
        }

        const uav_lumberjack_control::Vec3 target{x, y, z};
        const auto solutions = uav_lumberjack_control::ArmKinematics::inversePosition(target);
        std::size_t best_index = solutions.size();
        double best_distance = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < solutions.size(); ++i) {
            if (!solutions[i].within_joint_limits) continue;
            const auto q_deg = uav_lumberjack_control::ArmKinematics::radiansToDegrees(
                solutions[i].q);
            const double d2 = q_deg.q2 - current_q2_deg;
            const double d3 = q_deg.q3 - current_q3_deg;
            const double d4 = q_deg.q4 - current_q4_deg;
            const double distance = std::sqrt(d2 * d2 + d3 * d3 + d4 * d4);
            if (distance < best_distance) {
                best_distance = distance;
                best_index = i;
            }
        }

        if (best_index >= solutions.size()) return false;
        const auto q_deg = uav_lumberjack_control::ArmKinematics::radiansToDegrees(
            solutions[best_index].q);
        target_q2_deg = q_deg.q2;
        target_q3_deg = q_deg.q3;
        target_q4_deg = q_deg.q4;
        return true;
    }

    void hold_current_arm_pose()
    {
        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        if (get_current_joint_angles_deg(q2, q3, q4)) {
            publish_joint("j2", q2);
            publish_joint("j3", q3);
            publish_joint("j4", q4);
        }
        publish_saw_rpm(0.0);
    }

    void fill_arm_feedback(
        const std::shared_ptr<ArmMotion::Feedback> & feedback,
        const std::string & state,
        float progress,
        double target_q2_deg,
        double target_q3_deg,
        double target_q4_deg,
        const uav_lumberjack_control::Vec3 & target_tcp)
    {
        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        if (!get_current_joint_angles_deg(q2, q3, q4)) return;

        const auto q = uav_lumberjack_control::ArmKinematics::degreesToRadians(q2, q3, q4);
        const auto pose = uav_lumberjack_control::ArmKinematics::forward(q);

        const double e2 = std::abs(target_q2_deg - q2);
        const double e3 = std::abs(target_q3_deg - q3);
        const double e4 = std::abs(target_q4_deg - q4);
        const double dx = pose.position.x - target_tcp.x;
        const double dy = pose.position.y - target_tcp.y;
        const double dz = pose.position.z - target_tcp.z;

        feedback->state = state;
        feedback->progress = std::clamp(progress, 0.0f, 1.0f);
        feedback->actual_joint_deg = {q2, q3, q4};
        feedback->actual_tcp.x = pose.position.x;
        feedback->actual_tcp.y = pose.position.y;
        feedback->actual_tcp.z = pose.position.z;
        feedback->max_joint_error_deg = std::max(e2, std::max(e3, e4));
        feedback->tcp_error_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    rclcpp_action::GoalResponse handle_arm_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ArmMotion::Goal> goal)
    {
        if (goal->mode > ARM_MODE_TCP) {
            RCLCPP_WARN(get_logger(), "Arm action rejected: invalid mode %u", static_cast<unsigned int>(goal->mode));
            return rclcpp_action::GoalResponse::REJECT;
        }

        double q2 = 0.0, q3 = 0.0, q4 = 0.0;
        if (!get_current_joint_angles_deg(q2, q3, q4)) {
            RCLCPP_WARN(get_logger(), "Arm action rejected: joint feedback unavailable");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->mode == ARM_MODE_JOINT &&
            !joints_within_limits(
                goal->joint_target_deg[0],
                goal->joint_target_deg[1],
                goal->joint_target_deg[2])) {
            RCLCPP_WARN(get_logger(), "Arm action rejected: joint target outside limits");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->mode == ARM_MODE_TCP) {
            double tq2 = 0.0, tq3 = 0.0, tq4 = 0.0;
            if (!choose_nearest_tcp_ik(
                    goal->tcp_target.x,
                    goal->tcp_target.y,
                    goal->tcp_target.z,
                    tq2, tq3, tq4)) {
                RCLCPP_WARN(get_logger(), "Arm action rejected: no valid TCP IK solution");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        if (!try_begin_task("ACTION_PENDING")) {
            RCLCPP_WARN(
                get_logger(),
                "Arm action rejected: controller BUSY (%s)",
                get_active_task().c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_arm_cancel(
        const std::shared_ptr<GoalHandleArmMotion>)
    {
        if (!busy_.load()) return rclcpp_action::CancelResponse::REJECT;
        cancel_requested_.store(true);
        publish_saw_rpm(0.0);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_arm_accepted(const std::shared_ptr<GoalHandleArmMotion> goal_handle)
    {
        std::thread{
            [this, goal_handle]() {
                execute_arm_action(goal_handle);
            }}.detach();
    }

    void handle_set_saw_service(
        const std::shared_ptr<SetSaw::Request> request,
        std::shared_ptr<SetSaw::Response> response)
    {
        const double rpm = request->rpm;
        double current_saw_cmd = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_saw_cmd = saw_cmd_rpm_;
        }

        if (rpm < SAW_MIN_RPM || rpm > SAW_MAX_RPM) {
            response->accepted = false;
            response->commanded_rpm = current_saw_cmd;
            response->message = "RPM outside allowed range [0, 1800].";
            return;
        }

        if (busy_.load() && rpm > 0.0) {
            response->accepted = false;
            response->commanded_rpm = current_saw_cmd;
            response->message = "Arm is BUSY; saw start rejected. rpm=0 is always allowed.";
            return;
        }

        publish_saw_rpm(rpm);
        response->accepted = true;
        response->commanded_rpm = rpm;
        response->message = (rpm <= 0.0) ? "Saw OFF accepted." : "Saw RPM accepted.";
    }

    void execute_arm_action(const std::shared_ptr<GoalHandleArmMotion> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ArmMotion::Result>();
        auto feedback = std::make_shared<ArmMotion::Feedback>();

        double target_q2_deg = 0.0;
        double target_q3_deg = 0.0;
        double target_q4_deg = 0.0;
        std::string task_name;

        if (goal->mode == ARM_MODE_HOME) {
            task_name = "ACTION_HOME";
            target_q2_deg = HOME_J2_DEG;
            target_q3_deg = HOME_J3_DEG;
            target_q4_deg = HOME_J4_DEG;
        } else if (goal->mode == ARM_MODE_PREWORK) {
            task_name = "ACTION_PREWORK";
            target_q2_deg = PREWORK_J2_DEG;
            target_q3_deg = PREWORK_J3_DEG;
            target_q4_deg = PREWORK_J4_DEG;
        } else if (goal->mode == ARM_MODE_JOINT) {
            task_name = "ACTION_JOINT";
            target_q2_deg = goal->joint_target_deg[0];
            target_q3_deg = goal->joint_target_deg[1];
            target_q4_deg = goal->joint_target_deg[2];
        } else if (goal->mode == ARM_MODE_TCP) {
            task_name = "ACTION_TCP";
            if (!choose_nearest_tcp_ik(
                    goal->tcp_target.x,
                    goal->tcp_target.y,
                    goal->tcp_target.z,
                    target_q2_deg,
                    target_q3_deg,
                    target_q4_deg)) {
                result->success = false;
                result->message = "No valid IK solution when action execution started.";
                goal_handle->abort(result);
                finish_task();
                return;
            }
        }

        set_active_task(task_name);
        cancel_requested_.store(false);
        publish_saw_rpm(0.0);

        const auto target_q_rad = uav_lumberjack_control::ArmKinematics::degreesToRadians(
            target_q2_deg, target_q3_deg, target_q4_deg);
        const auto target_pose = uav_lumberjack_control::ArmKinematics::forward(target_q_rad);
        uav_lumberjack_control::Vec3 target_tcp{
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z};
        if (goal->mode == ARM_MODE_TCP) {
            target_tcp.x = goal->tcp_target.x;
            target_tcp.y = goal->tcp_target.y;
            target_tcp.z = goal->tcp_target.z;
        }

        double start_q2_deg = 0.0, start_q3_deg = 0.0, start_q4_deg = 0.0;
        if (!get_current_joint_angles_deg(start_q2_deg, start_q3_deg, start_q4_deg)) {
            result->success = false;
            result->message = "Joint feedback unavailable at action start.";
            goal_handle->abort(result);
            finish_task();
            return;
        }

        const TrapProfile p2 = make_trapezoid(
            start_q2_deg, target_q2_deg,
            J2_MAX_CMD_SPEED_DEG_S, J2_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p3 = make_trapezoid(
            start_q3_deg, target_q3_deg,
            J3_MAX_CMD_SPEED_DEG_S, J3_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p4 = make_trapezoid(
            start_q4_deg, target_q4_deg,
            J4_MAX_CMD_SPEED_DEG_S, J4_MAX_CMD_ACCEL_DEG_S2);

        const double total_time = std::max(p2.total_time, std::max(p3.total_time, p4.total_time));
        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            if (cancel_requested_.load() || goal_handle->is_canceling()) {
                hold_current_arm_pose();
                fill_arm_feedback(
                    feedback, "CANCELED", 0.0f,
                    target_q2_deg, target_q3_deg, target_q4_deg, target_tcp);
                goal_handle->publish_feedback(feedback);
                result->success = false;
                result->message = "Arm motion canceled; current pose held and saw stopped.";
                goal_handle->canceled(result);
                finish_task();
                return;
            }

            const double t = std::min(i * dt_sec, total_time);
            publish_joint("j2", sample_trapezoid(p2, std::min(t, p2.total_time)));
            publish_joint("j3", sample_trapezoid(p3, std::min(t, p3.total_time)));
            publish_joint("j4", sample_trapezoid(p4, std::min(t, p4.total_time)));

            if (i == 1 || i == steps || i % 5 == 0) {
                const float progress = total_time <= 1e-9
                    ? 0.90f
                    : static_cast<float>(0.90 * t / total_time);
                fill_arm_feedback(
                    feedback, "MOVING", progress,
                    target_q2_deg, target_q3_deg, target_q4_deg, target_tcp);
                goal_handle->publish_feedback(feedback);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint("j2", target_q2_deg);
        publish_joint("j3", target_q3_deg);
        publish_joint("j4", target_q4_deg);

        int stable_cycles = 0;
        const auto settle_start = std::chrono::steady_clock::now();
        bool reached = false;

        while (rclcpp::ok()) {
            if (cancel_requested_.load() || goal_handle->is_canceling()) {
                hold_current_arm_pose();
                result->success = false;
                result->message = "Arm motion canceled during settling; current pose held.";
                goal_handle->canceled(result);
                finish_task();
                return;
            }

            double q2 = 0.0, q3 = 0.0, q4 = 0.0;
            double v2 = 0.0, v3 = 0.0, v4 = 0.0;
            if (!get_joint_feedback("j2", q2, v2) ||
                !get_joint_feedback("j3", q3, v3) ||
                !get_joint_feedback("j4", q4, v4)) {
                result->success = false;
                result->message = "Joint feedback lost while settling.";
                goal_handle->abort(result);
                finish_task();
                return;
            }

            const bool ok2 = std::abs(target_q2_deg - q2) <= POSITION_TOLERANCE_DEG &&
                             std::abs(v2) <= VELOCITY_TOLERANCE_DEG_S;
            const bool ok3 = std::abs(target_q3_deg - q3) <= POSITION_TOLERANCE_DEG &&
                             std::abs(v3) <= VELOCITY_TOLERANCE_DEG_S;
            const bool ok4 = std::abs(target_q4_deg - q4) <= POSITION_TOLERANCE_DEG &&
                             std::abs(v4) <= VELOCITY_TOLERANCE_DEG_S;

            stable_cycles = (ok2 && ok3 && ok4) ? stable_cycles + 1 : 0;
            fill_arm_feedback(
                feedback, "SETTLING", 0.95f,
                target_q2_deg, target_q3_deg, target_q4_deg, target_tcp);
            goal_handle->publish_feedback(feedback);

            if (stable_cycles >= STABLE_CYCLES_REQUIRED) {
                reached = true;
                break;
            }

            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - settle_start).count();
            if (elapsed >= MOTION_TIMEOUT_SEC) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_PERIOD_MS));
        }

        double final_q2 = 0.0, final_q3 = 0.0, final_q4 = 0.0;
        get_current_joint_angles_deg(final_q2, final_q3, final_q4);
        const auto final_q_rad = uav_lumberjack_control::ArmKinematics::degreesToRadians(
            final_q2, final_q3, final_q4);
        const auto final_pose = uav_lumberjack_control::ArmKinematics::forward(final_q_rad);
        const double dx = final_pose.position.x - target_tcp.x;
        const double dy = final_pose.position.y - target_tcp.y;
        const double dz = final_pose.position.z - target_tcp.z;
        const double final_tcp_error = std::sqrt(dx * dx + dy * dy + dz * dz);

        result->final_joint_deg = {final_q2, final_q3, final_q4};
        result->final_tcp.x = final_pose.position.x;
        result->final_tcp.y = final_pose.position.y;
        result->final_tcp.z = final_pose.position.z;
        result->final_tcp_error_m = final_tcp_error;

        fill_arm_feedback(
            feedback,
            reached ? "SUCCEEDED" : "TIMEOUT",
            reached ? 1.0f : 0.95f,
            target_q2_deg, target_q3_deg, target_q4_deg, target_tcp);
        goal_handle->publish_feedback(feedback);

        if (reached) {
            result->success = true;
            result->message = "Arm motion completed successfully.";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Arm motion timed out before stable convergence.";
            goal_handle->abort(result);
        }

        finish_task();
    }

    void print_startup_banner()
    {
        std::ostringstream oss;
        oss <<
            "\n+----------------------------------------------------------+\n"
            "| UAV_lumberjack | 3-DOF ARM + SAW CONSOLE                 |\n"
            "+----------------------------------------------------------+\n"
            "| System : READY                                           |\n"
            "| HOME   : J2=-30  J3=-150  J4=-90 deg                     |\n"
            "| PREWORK: J2=-30  J3=-60   J4=0 deg                       |\n"
            "| TCP  : chainsaw_body origin                              |\n"
            "| AUTO   : Action + Service READY                          |\n"
            "+----------------------------------------------------------+\n"
            " Quick: home | prework | status | fk | ik x y z | movetcp x y z\n"
            "        saw rpm | saw off | help | quit\n"
            "        \n"
            " ROS2 : action /motion | service /set_saw | topic /status\n"
            " Note : V1 checks joint limits only; no collision checking.";
        safe_print(oss.str());
    }

    void print_status()
    {
        double j2_cmd, j3_cmd, j4_cmd, saw_cmd;
        double j2_actual, j3_actual, j4_actual, saw_actual;
        double j2_velocity, j3_velocity, j4_velocity;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            j2_cmd = j2_cmd_deg_;
            j3_cmd = j3_cmd_deg_;
            j4_cmd = j4_cmd_deg_;
            saw_cmd = saw_cmd_rpm_;
            j2_actual = j2_actual_deg_;
            j3_actual = j3_actual_deg_;
            j4_actual = j4_actual_deg_;
            saw_actual = saw_actual_rpm_;
            j2_velocity = j2_velocity_deg_s_;
            j3_velocity = j3_velocity_deg_s_;
            j4_velocity = j4_velocity_deg_s_;
        }

        const auto q = uav_lumberjack_control::ArmKinematics::degreesToRadians(
            j2_actual, j3_actual, j4_actual);
        const auto pose = uav_lumberjack_control::ArmKinematics::forward(q);

        std::ostringstream oss;
        oss << "\n================ ARM / STATUS ==================\n"
            << "State : " << (busy_.load() ? "BUSY" : "IDLE")
            << "    Task : " << get_active_task() << "\n"
            << "------------------------------------------------\n"
            << "Joint     Command      Actual       Error      Velocity\n"
            << "------------------------------------------------\n"
            << std::fixed << std::setprecision(2);

        auto row = [&oss](const std::string & name, double cmd, double actual, double vel) {
            oss << std::left << std::setw(7) << name
                << std::right << std::setw(8) << cmd << " deg   "
                << std::setw(8) << actual << " deg   "
                << std::setw(7) << (cmd - actual) << " deg   "
                << std::setw(7) << vel << " deg/s\n";
        };

        row("J2", j2_cmd, j2_actual, j2_velocity);
        row("J3", j3_cmd, j3_actual, j3_velocity);
        row("J4", j4_cmd, j4_actual, j4_velocity);

        oss << "------------------------------------------------\n"
            << "Saw       : cmd " << saw_cmd << " rpm | actual " << saw_actual << " rpm\n"
            << std::setprecision(6)
            << "TCP [m]   : [" << pose.position.x << ", "
            << pose.position.y << ", " << pose.position.z << "]\n"
            << "TCP model : chainsaw_body origin (V1)\n"
            << "ROS API   : /lumberjack_arm/motion | /set_saw | /status\n"
            << "================================================";
        safe_print(oss.str());
    }

    void print_help()
    {
        std::ostringstream oss;
        oss <<
            "\n+----------------------------------------------------------+\n"
            "| UAV_lumberjack | ARM + SAW CONTROL HELP                  |\n"
            "+----------------------------------------------------------+\n"
            "| Manual motion                                            |\n"
            "|   home                 -> HOME (-30,-150,-90 deg)        |\n"
            "|   prework / ready      -> PREWORK (-30,-60,0 deg)        |\n"
            "|   j2/j3/j4 <deg>       -> single joint motion            |\n"
            "|   movetcp x y z        -> IK + TCP position motion       |\n"
            "|                                                          |\n"
            "| Kinematics (calculation only)                            |\n"
            "|   fk                   -> actual joints -> TCP pose      |\n"
            "|   ik x y z             -> TCP -> IK candidates           |\n"
            "|                                                          |\n"
            "| Saw                                                      |\n"
            "|   saw <rpm>            -> 0 ... 1800 rpm                 |\n"
            "|   saw off              -> emergency / immediate OFF      |\n"
            "|                                                          |\n"
            "| Utility                                                  |\n"
            "|   status | init | help | quit                            |\n"
            "+----------------------------------------------------------+\n"
            "| ROS2 automatic API                                       |\n"
            "|   Action : /lumberjack_arm/motion                        |\n"
            "|            HOME / PREWORK / JOINT / TCP                  |\n"
            "|            Goal + Feedback + Result + Cancel             |\n"
            "|   Service: /lumberjack_arm/set_saw                       |\n"
            "|            request rpm -> accepted / rejected            |\n"
            "|   Topic  : /lumberjack_arm/status                        |\n"
            "|            joints / TCP / saw / BUSY state @ 10 Hz       |\n"
            "+----------------------------------------------------------+\n"
            "| BUSY policy                                              |\n"
            "|   Allowed : status | fk | ik | help | saw off            |\n"
            "|   Rejected: new arm motion | saw start | quit            |\n"
            "+----------------------------------------------------------+\n"
            "| Safety V1: joint limits only; NO collision checking.     |\n"
            "+----------------------------------------------------------+";
        safe_print(oss.str());
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmController>();

    std::thread ros_spin_thread([node]() {
        rclcpp::spin(node);
    });

    node->run();
    node->join_motion_thread();
    rclcpp::shutdown();

    if (ros_spin_thread.joinable()) ros_spin_thread.join();
    return 0;
}
