#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm_controller")
    {
        j1_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j1/cmd_pos", 10);
        j2_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j2/cmd_pos", 10);
        j3_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j3/cmd_pos", 10);

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/lumberjack_arm/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&ArmController::joint_state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "UAV_lumberjack Arm Controller started.");
    }

    void run()
    {
        wait_for_bridge();
        wait_for_joint_feedback();
        print_help();

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(io_mutex_);
                std::cout << (busy_.load() ? "\n[BUSY] > " : "\n[IDLE] > ") << std::flush;
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

            if (command == "quit" || command == "exit") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                safe_print("Exit arm controller.");
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

            if (command == "j1" || command == "j2" || command == "j3") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }

                double angle_deg;
                if (!(iss >> angle_deg)) {
                    safe_print("Invalid command. Example: j1 45");
                    continue;
                }

                if (!check_joint_limit(command, angle_deg)) continue;
                start_joint_motion(command, angle_deg);
                continue;
            }

            safe_print("Unknown command. Type 'help' to show available commands.");
        }
    }

    void join_motion_thread()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
    }

private:
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double DEG_TO_RAD = PI / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / PI;

    static constexpr double J1_MIN_DEG = -180.0;
    static constexpr double J1_MAX_DEG = 180.0;
    static constexpr double J2_MIN_DEG = -90.0;
    static constexpr double J2_MAX_DEG = 90.0;
    static constexpr double J3_MIN_DEG = -90.0;
    static constexpr double J3_MAX_DEG = 90.0;

    static constexpr double POSITION_TOLERANCE_DEG = 1.0;
    static constexpr double VELOCITY_TOLERANCE_DEG_S = 1.0;
    static constexpr int STABLE_CYCLES_REQUIRED = 10;
    static constexpr int CHECK_PERIOD_MS = 50;
    static constexpr double MOTION_TIMEOUT_SEC = 10.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j3_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    double j1_cmd_deg_ = 0.0;
    double j2_cmd_deg_ = 0.0;
    double j3_cmd_deg_ = 0.0;

    double j1_actual_deg_ = 0.0;
    double j2_actual_deg_ = 0.0;
    double j3_actual_deg_ = 0.0;

    double j1_velocity_deg_s_ = 0.0;
    double j2_velocity_deg_s_ = 0.0;
    double j3_velocity_deg_s_ = 0.0;

    bool j1_feedback_received_ = false;
    bool j2_feedback_received_ = false;
    bool j3_feedback_received_ = false;

    std::atomic<bool> busy_{false};
    std::thread motion_thread_;

    std::mutex state_mutex_;
    std::mutex io_mutex_;

    void safe_print(const std::string & text)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        std::cout << text << std::endl;
    }

    void reject_busy()
    {
        safe_print("[REJECTED] Arm is BUSY. Wait until the current motion finishes.");
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const std::size_t count = std::min(msg->name.size(), msg->position.size());

        for (std::size_t i = 0; i < count; ++i) {
            const double position_deg = msg->position[i] * RAD_TO_DEG;
            double velocity_deg_s = 0.0;
            if (i < msg->velocity.size()) velocity_deg_s = msg->velocity[i] * RAD_TO_DEG;

            if (msg->name[i] == "j1") {
                j1_actual_deg_ = position_deg;
                j1_velocity_deg_s_ = velocity_deg_s;
                j1_feedback_received_ = true;
            } else if (msg->name[i] == "j2") {
                j2_actual_deg_ = position_deg;
                j2_velocity_deg_s_ = velocity_deg_s;
                j2_feedback_received_ = true;
            } else if (msg->name[i] == "j3") {
                j3_actual_deg_ = position_deg;
                j3_velocity_deg_s_ = velocity_deg_s;
                j3_feedback_received_ = true;
            }
        }
    }

    void wait_for_bridge()
    {
        safe_print("Waiting for ros_gz_bridge command channels...");

        while (rclcpp::ok()) {
            const bool j1_ready = j1_pub_->get_subscription_count() > 0;
            const bool j2_ready = j2_pub_->get_subscription_count() > 0;
            const bool j3_ready = j3_pub_->get_subscription_count() > 0;

            if (j1_ready && j2_ready && j3_ready) {
                safe_print("All command bridges detected.");
                return;
            }
            std::this_thread::sleep_for(500ms);
        }
    }

    void wait_for_joint_feedback()
    {
        safe_print("Waiting for joint state feedback...");

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (j1_feedback_received_ && j2_feedback_received_ && j3_feedback_received_) {
                    safe_print("Joint state feedback detected.");
                    return;
                }
            }
            std::this_thread::sleep_for(100ms);
        }
    }

    bool check_joint_limit(const std::string & joint, double angle_deg)
    {
        double min_deg = 0.0;
        double max_deg = 0.0;

        if (joint == "j1") {
            min_deg = J1_MIN_DEG;
            max_deg = J1_MAX_DEG;
        } else if (joint == "j2") {
            min_deg = J2_MIN_DEG;
            max_deg = J2_MAX_DEG;
        } else if (joint == "j3") {
            min_deg = J3_MIN_DEG;
            max_deg = J3_MAX_DEG;
        } else {
            safe_print("Unknown joint: " + joint);
            return false;
        }

        if (angle_deg < min_deg || angle_deg > max_deg) {
            std::ostringstream oss;
            oss << "Rejected: " << joint
                << " angle must be within [" << min_deg
                << ", " << max_deg << "] deg.";
            safe_print(oss.str());
            return false;
        }

        return true;
    }

    double get_commanded_angle(const std::string & joint)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (joint == "j1") return j1_cmd_deg_;
        if (joint == "j2") return j2_cmd_deg_;
        if (joint == "j3") return j3_cmd_deg_;
        return 0.0;
    }

    void publish_joint(const std::string & joint, double angle_deg)
    {
        std_msgs::msg::Float64 msg;
        msg.data = angle_deg * DEG_TO_RAD;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (joint == "j1") j1_cmd_deg_ = angle_deg;
            else if (joint == "j2") j2_cmd_deg_ = angle_deg;
            else if (joint == "j3") j3_cmd_deg_ = angle_deg;
        }

        if (joint == "j1") j1_pub_->publish(msg);
        else if (joint == "j2") j2_pub_->publish(msg);
        else if (joint == "j3") j3_pub_->publish(msg);
    }

    bool get_joint_feedback(const std::string & joint, double & actual_deg, double & velocity_deg_s)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (joint == "j1") {
            if (!j1_feedback_received_) return false;
            actual_deg = j1_actual_deg_;
            velocity_deg_s = j1_velocity_deg_s_;
            return true;
        }

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
            const double elapsed_sec =
                std::chrono::duration<double>(now - start_time).count();

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
            double j1_actual, j2_actual, j3_actual;
            double j1_velocity, j2_velocity, j3_velocity;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                j1_actual = j1_actual_deg_;
                j2_actual = j2_actual_deg_;
                j3_actual = j3_actual_deg_;
                j1_velocity = j1_velocity_deg_s_;
                j2_velocity = j2_velocity_deg_s_;
                j3_velocity = j3_velocity_deg_s_;
            }

            const bool j1_ok =
                std::abs(j1_actual) <= POSITION_TOLERANCE_DEG &&
                std::abs(j1_velocity) <= VELOCITY_TOLERANCE_DEG_S;

            const bool j2_ok =
                std::abs(j2_actual) <= POSITION_TOLERANCE_DEG &&
                std::abs(j2_velocity) <= VELOCITY_TOLERANCE_DEG_S;

            const bool j3_ok =
                std::abs(j3_actual) <= POSITION_TOLERANCE_DEG &&
                std::abs(j3_velocity) <= VELOCITY_TOLERANCE_DEG_S;

            if (j1_ok && j2_ok && j3_ok) stable_cycles++;
            else stable_cycles = 0;

            if (stable_cycles >= STABLE_CYCLES_REQUIRED) {
                safe_print("[REACHED] HOME");
                return true;
            }

            const auto now = std::chrono::steady_clock::now();
            const double elapsed_sec =
                std::chrono::duration<double>(now - start_time).count();

            if (elapsed_sec >= MOTION_TIMEOUT_SEC) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "[TIMEOUT] HOME failed.\n"
                    << "J1 actual = " << j1_actual << " deg\n"
                    << "J2 actual = " << j2_actual << " deg\n"
                    << "J3 actual = " << j3_actual << " deg";
                safe_print(oss.str());
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_PERIOD_MS));
        }

        return false;
    }

    void start_joint_motion(const std::string & joint, double target_deg)
    {
        if (motion_thread_.joinable()) motion_thread_.join();

        busy_.store(true);

        motion_thread_ = std::thread([this, joint, target_deg]() {
            const double old_angle_deg = get_commanded_angle(joint);

            {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "[BUSY] " << joint
                    << ": " << old_angle_deg
                    << " deg -> " << target_deg << " deg";
                safe_print(oss.str());
            }

            publish_joint(joint, target_deg);
            const bool reached = wait_until_joint_reached(joint, target_deg);

            if (reached) safe_print("Done. [IDLE]");
            else safe_print("Motion ended with timeout/error. [IDLE]");

            busy_.store(false);
        });
    }

    void start_home_motion()
    {
        if (motion_thread_.joinable()) motion_thread_.join();

        busy_.store(true);

        motion_thread_ = std::thread([this]() {
            safe_print("[BUSY] Returning arm to HOME...");

            publish_joint("j1", 0.0);
            publish_joint("j2", 0.0);
            publish_joint("j3", 0.0);

            const bool reached = wait_until_home_reached();

            if (reached) safe_print("HOME reached. [IDLE]");
            else safe_print("HOME ended with timeout/error. [IDLE]");

            busy_.store(false);
        });
    }

    void print_status()
    {
        double j1_cmd, j2_cmd, j3_cmd;
        double j1_actual, j2_actual, j3_actual;
        double j1_velocity, j2_velocity, j3_velocity;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            j1_cmd = j1_cmd_deg_;
            j2_cmd = j2_cmd_deg_;
            j3_cmd = j3_cmd_deg_;
            j1_actual = j1_actual_deg_;
            j2_actual = j2_actual_deg_;
            j3_actual = j3_actual_deg_;
            j1_velocity = j1_velocity_deg_s_;
            j2_velocity = j2_velocity_deg_s_;
            j3_velocity = j3_velocity_deg_s_;
        }

        std::ostringstream oss;
        oss << "\n====================================================================\n"
            << "                  UAV_lumberjack Arm State\n"
            << "====================================================================\n\n"
            << "Joint   Commanded     Actual       Error       Velocity\n"
            << "--------------------------------------------------------------------\n"
            << std::fixed << std::setprecision(2);

        auto row = [&oss](const std::string & name, double cmd, double actual, double vel) {
            oss << name << "      "
                << std::setw(8) << cmd << " deg   "
                << std::setw(8) << actual << " deg   "
                << std::setw(8) << (cmd - actual) << " deg   "
                << std::setw(8) << vel << " deg/s\n";
        };

        row("J1", j1_cmd, j1_actual, j1_velocity);
        row("J2", j2_cmd, j2_actual, j2_velocity);
        row("J3", j3_cmd, j3_actual, j3_velocity);

        oss << "\nArm State: " << (busy_.load() ? "BUSY" : "IDLE")
            << "\n====================================================================";

        safe_print(oss.str());
    }

    void print_help()
    {
        std::ostringstream oss;
        oss <<
            "\n========================================\n"
            " UAV_lumberjack 3-DOF Arm Controller\n"
            "========================================\n\n"
            "Joint Commands:\n"
            "  j1 <deg>     Base Yaw\n"
            "  j2 <deg>     Shoulder Pitch\n"
            "  j3 <deg>     Elbow Pitch\n\n"
            "Other Commands:\n"
            "  home         All joints -> 0 deg\n"
            "  status       Show state (allowed while BUSY)\n"
            "  help         Show help (allowed while BUSY)\n"
            "  quit         Exit controller (IDLE only)\n\n"
            "Joint Limits:\n"
            "  J1: -180 deg ~ +180 deg\n"
            "  J2:  -90 deg ~  +90 deg\n"
            "  J3:  -90 deg ~  +90 deg\n\n"
            "Motion Completion:\n"
            "  Position tolerance : +/- 1.0 deg\n"
            "  Velocity tolerance : +/- 1.0 deg/s\n"
            "  Stable cycles      : 10\n"
            "  Check period       : 50 ms\n"
            "  Timeout            : 10 s\n\n"
            "BUSY Policy:\n"
            "  j1 / j2 / j3 / home / quit -> REJECTED while BUSY\n"
            "  status / help                 -> allowed while BUSY\n"
            "========================================";
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