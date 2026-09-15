#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

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
        j4_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/j4/cmd_pos", 10);
        saw_pub_ = create_publisher<std_msgs::msg::Float64>("/lumberjack_arm/saw/cmd_vel", 10);

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/lumberjack_arm/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&ArmController::joint_state_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "UAV_lumberjack 4-DOF Arm + Saw Controller started.");
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

            if (command == "test") {
                if (busy_.load()) {
                    reject_busy();
                    continue;
                }
                start_test_motion();
                continue;
            }

            if (command == "j1" || command == "j2" || command == "j3" || command == "j4") {
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

    static constexpr double J1_MIN_DEG = -180.0;
    static constexpr double J1_MAX_DEG = 180.0;
    static constexpr double J2_MIN_DEG = -90.0;
    static constexpr double J2_MAX_DEG = 90.0;
    static constexpr double J3_MIN_DEG = -90.0;
    static constexpr double J3_MAX_DEG = 90.0;
    static constexpr double J4_MIN_DEG = -180.0;
    static constexpr double J4_MAX_DEG = 180.0;
    static constexpr double SAW_MIN_RPM = 0.0;
    static constexpr double SAW_MAX_RPM = 1800.0;

    static constexpr double POSITION_TOLERANCE_DEG = 1.0;
    static constexpr double VELOCITY_TOLERANCE_DEG_S = 1.0;
    static constexpr int STABLE_CYCLES_REQUIRED = 10;
    static constexpr int CHECK_PERIOD_MS = 50;
    static constexpr double MOTION_TIMEOUT_SEC = 10.0;

    // Trapezoidal velocity trajectory:
    // acceleration -> constant speed -> deceleration.
    // If a move is too short to reach vmax, it automatically becomes a triangular profile.
    static constexpr int TRAJECTORY_PERIOD_MS = 20;
    static constexpr double J1_MAX_CMD_SPEED_DEG_S = 45.0;
    static constexpr double J2_MAX_CMD_SPEED_DEG_S = 30.0;
    static constexpr double J3_MAX_CMD_SPEED_DEG_S = 45.0;
    static constexpr double J4_MAX_CMD_SPEED_DEG_S = 60.0;

    static constexpr double J1_MAX_CMD_ACCEL_DEG_S2 = 30.0;
    static constexpr double J2_MAX_CMD_ACCEL_DEG_S2 = 25.0;
    static constexpr double J3_MAX_CMD_ACCEL_DEG_S2 = 35.0;
    static constexpr double J4_MAX_CMD_ACCEL_DEG_S2 = 40.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j3_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j4_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr saw_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    double j1_cmd_deg_ = 0.0;
    double j2_cmd_deg_ = 0.0;
    double j3_cmd_deg_ = 0.0;
    double j4_cmd_deg_ = 0.0;
    double saw_cmd_rpm_ = 0.0;

    double j1_actual_deg_ = 0.0;
    double j2_actual_deg_ = 0.0;
    double j3_actual_deg_ = 0.0;
    double j4_actual_deg_ = 0.0;
    double saw_actual_rpm_ = 0.0;

    double j1_velocity_deg_s_ = 0.0;
    double j2_velocity_deg_s_ = 0.0;
    double j3_velocity_deg_s_ = 0.0;
    double j4_velocity_deg_s_ = 0.0;

    bool j1_feedback_received_ = false;
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
            double velocity_rad_s = 0.0;
            if (i < msg->velocity.size()) velocity_rad_s = msg->velocity[i];
            const double velocity_deg_s = velocity_rad_s * RAD_TO_DEG;

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
            } else if (msg->name[i] == "j4") {
                j4_actual_deg_ = position_deg;
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
        safe_print("Waiting for ros_gz_bridge command channels...");

        while (rclcpp::ok()) {
            const bool j1_ready = j1_pub_->get_subscription_count() > 0;
            const bool j2_ready = j2_pub_->get_subscription_count() > 0;
            const bool j3_ready = j3_pub_->get_subscription_count() > 0;
            const bool j4_ready = j4_pub_->get_subscription_count() > 0;
            const bool saw_ready = saw_pub_->get_subscription_count() > 0;

            if (j1_ready && j2_ready && j3_ready && j4_ready && saw_ready) {
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
            bool ready = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ready = j1_feedback_received_ && j2_feedback_received_ && j3_feedback_received_ &&
                        j4_feedback_received_ && saw_feedback_received_;
            }

            if (ready) {
                safe_print("Joint state feedback detected: J1/J2/J3/J4/Saw.");
                return;
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
        if (joint == "j1") return j1_cmd_deg_;
        if (joint == "j2") return j2_cmd_deg_;
        if (joint == "j3") return j3_cmd_deg_;
        if (joint == "j4") return j4_cmd_deg_;
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
            else if (joint == "j4") j4_cmd_deg_ = angle_deg;
        }

        if (joint == "j1") j1_pub_->publish(msg);
        else if (joint == "j2") j2_pub_->publish(msg);
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
            double j1_actual, j2_actual, j3_actual, j4_actual;
            double j1_velocity, j2_velocity, j3_velocity, j4_velocity;

            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                j1_actual = j1_actual_deg_;
                j2_actual = j2_actual_deg_;
                j3_actual = j3_actual_deg_;
                j4_actual = j4_actual_deg_;
                j1_velocity = j1_velocity_deg_s_;
                j2_velocity = j2_velocity_deg_s_;
                j3_velocity = j3_velocity_deg_s_;
                j4_velocity = j4_velocity_deg_s_;
            }

            const bool j1_ok = std::abs(j1_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j1_velocity) <= VELOCITY_TOLERANCE_DEG_S;
            const bool j2_ok = std::abs(j2_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j2_velocity) <= VELOCITY_TOLERANCE_DEG_S;
            const bool j3_ok = std::abs(j3_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j3_velocity) <= VELOCITY_TOLERANCE_DEG_S;
            const bool j4_ok = std::abs(j4_actual) <= POSITION_TOLERANCE_DEG &&
                               std::abs(j4_velocity) <= VELOCITY_TOLERANCE_DEG_S;

            if (j1_ok && j2_ok && j3_ok && j4_ok) stable_cycles++;
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
                    << "J1 actual = " << j1_actual << " deg\n"
                    << "J2 actual = " << j2_actual << " deg\n"
                    << "J3 actual = " << j3_actual << " deg\n"
                    << "J4 actual = " << j4_actual << " deg";
                safe_print(oss.str());
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHECK_PERIOD_MS));
        }
        return false;
    }

    double get_joint_speed_limit_deg_s(const std::string & joint) const
    {
        if (joint == "j1") return J1_MAX_CMD_SPEED_DEG_S;
        if (joint == "j2") return J2_MAX_CMD_SPEED_DEG_S;
        if (joint == "j3") return J3_MAX_CMD_SPEED_DEG_S;
        if (joint == "j4") return J4_MAX_CMD_SPEED_DEG_S;
        return J1_MAX_CMD_SPEED_DEG_S;
    }

    double get_joint_accel_limit_deg_s2(const std::string & joint) const
    {
        if (joint == "j1") return J1_MAX_CMD_ACCEL_DEG_S2;
        if (joint == "j2") return J2_MAX_CMD_ACCEL_DEG_S2;
        if (joint == "j3") return J3_MAX_CMD_ACCEL_DEG_S2;
        if (joint == "j4") return J4_MAX_CMD_ACCEL_DEG_S2;
        return J1_MAX_CMD_ACCEL_DEG_S2;
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

    bool publish_home_trapezoid()
    {
        double q1 = 0.0, q2 = 0.0, q3 = 0.0, q4 = 0.0;
        double v = 0.0;
        if (!get_joint_feedback("j1", q1, v) || !get_joint_feedback("j2", q2, v) ||
            !get_joint_feedback("j3", q3, v) || !get_joint_feedback("j4", q4, v)) {
            safe_print("[ERROR] Joint feedback unavailable before HOME trajectory.");
            return false;
        }

        const TrapProfile p1 = make_trapezoid(q1, 0.0, J1_MAX_CMD_SPEED_DEG_S, J1_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p2 = make_trapezoid(q2, 0.0, J2_MAX_CMD_SPEED_DEG_S, J2_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p3 = make_trapezoid(q3, 0.0, J3_MAX_CMD_SPEED_DEG_S, J3_MAX_CMD_ACCEL_DEG_S2);
        const TrapProfile p4 = make_trapezoid(q4, 0.0, J4_MAX_CMD_SPEED_DEG_S, J4_MAX_CMD_ACCEL_DEG_S2);

        const double total_time = std::max(
            std::max(p1.total_time, p2.total_time),
            std::max(p3.total_time, p4.total_time));

        const double dt_sec = static_cast<double>(TRAJECTORY_PERIOD_MS) / 1000.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(total_time / dt_sec)));

        for (int i = 1; i <= steps && rclcpp::ok(); ++i) {
            const double t = i * dt_sec;
            publish_joint("j1", sample_trapezoid(p1, std::min(t, p1.total_time)));
            publish_joint("j2", sample_trapezoid(p2, std::min(t, p2.total_time)));
            publish_joint("j3", sample_trapezoid(p3, std::min(t, p3.total_time)));
            publish_joint("j4", sample_trapezoid(p4, std::min(t, p4.total_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(TRAJECTORY_PERIOD_MS));
        }

        publish_joint("j1", 0.0);
        publish_joint("j2", 0.0);
        publish_joint("j3", 0.0);
        publish_joint("j4", 0.0);
        return true;
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
        busy_.store(true);

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
            busy_.store(false);
        });
    }

    void start_home_motion()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        busy_.store(true);

        motion_thread_ = std::thread([this]() {
            safe_print("[BUSY] Saw OFF, returning J1/J2/J3/J4 to HOME...");
            const bool reached = home_all_sync();
            safe_print(reached ? "HOME reached. [IDLE]" : "HOME ended with timeout/error. [IDLE]");
            busy_.store(false);
        });
    }

    void start_test_motion()
    {
        if (motion_thread_.joinable()) motion_thread_.join();
        busy_.store(true);

        motion_thread_ = std::thread([this]() {
            safe_print("[TEST] Starting automatic arm sweep. Saw will remain OFF.");
            publish_saw_rpm(0.0);

            bool ok = true;
            if (ok) ok = move_joint_sync("j2", -60.0);
            if (ok) ok = move_joint_sync("j1", 180.0);
            if (ok) ok = move_joint_sync("j1", -180.0);
            if (ok) ok = move_joint_sync("j1", 0.0);
            if (ok) ok = move_joint_sync("j3", 90.0);
            if (ok) ok = move_joint_sync("j3", -90.0);
            if (ok) ok = move_joint_sync("j3", 0.0);
            if (ok) ok = move_joint_sync("j4", 180.0);
            if (ok) ok = move_joint_sync("j4", -180.0);
            if (ok) ok = move_joint_sync("j4", 0.0);

            if (!ok) safe_print("[TEST] A sweep step failed. Attempting HOME anyway...");
            else safe_print("[TEST] Sweep complete. Returning to HOME...");

            const bool home_ok = home_all_sync();
            if (ok && home_ok) safe_print("[TEST] PASS - automatic sweep complete, HOME reached. [IDLE]");
            else if (home_ok) safe_print("[TEST] END - sweep had an error, but HOME was recovered. [IDLE]");
            else safe_print("[TEST] FAIL - HOME recovery also failed. [IDLE]");
            busy_.store(false);
        });
    }

    void print_status()
    {
        double j1_cmd, j2_cmd, j3_cmd, j4_cmd, saw_cmd;
        double j1_actual, j2_actual, j3_actual, j4_actual, saw_actual;
        double j1_velocity, j2_velocity, j3_velocity, j4_velocity;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            j1_cmd = j1_cmd_deg_;
            j2_cmd = j2_cmd_deg_;
            j3_cmd = j3_cmd_deg_;
            j4_cmd = j4_cmd_deg_;
            saw_cmd = saw_cmd_rpm_;
            j1_actual = j1_actual_deg_;
            j2_actual = j2_actual_deg_;
            j3_actual = j3_actual_deg_;
            j4_actual = j4_actual_deg_;
            saw_actual = saw_actual_rpm_;
            j1_velocity = j1_velocity_deg_s_;
            j2_velocity = j2_velocity_deg_s_;
            j3_velocity = j3_velocity_deg_s_;
            j4_velocity = j4_velocity_deg_s_;
        }

        std::ostringstream oss;
        oss << "\n============================================================================\n"
            << "                     UAV_lumberjack Arm + Saw State\n"
            << "============================================================================\n\n"
            << "Joint   Commanded     Actual       Error       Velocity\n"
            << "----------------------------------------------------------------------------\n"
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
        row("J4", j4_cmd, j4_actual, j4_velocity);

        oss << "\nSaw commanded : " << saw_cmd << " rpm"
            << "\nSaw actual    : " << saw_actual << " rpm"
            << "\nArm State     : " << (busy_.load() ? "BUSY" : "IDLE")
            << "\n============================================================================";
        safe_print(oss.str());
    }

    void print_help()
    {
        std::ostringstream oss;
        oss <<
            "\n===============================================\n"
            " UAV_lumberjack 4-DOF Arm + Saw Controller\n"
            "===============================================\n\n"
            "Joint Commands:\n"
            "  j1 <deg>       Base Yaw\n"
            "  j2 <deg>       Shoulder Pitch\n"
            "  j3 <deg>       Elbow Pitch\n"
            "  j4 <deg>       Wrist Roll\n\n"
            "Saw Commands:\n"
            "  saw <rpm>      Saw speed, 0 ~ 1800 rpm\n"
            "  saw off        Stop saw immediately\n\n"
            "Automatic Test:\n"
            "  test            J2 -> -60, sweep J1/J3/J4, then HOME\n"
            "                  J1: +180 -> -180 -> 0\n"
            "                  J3:  +90 ->  -90 -> 0\n"
            "                  J4: +180 -> -180 -> 0\n"
            "                  Saw remains OFF during test\n\n"
            "Other Commands:\n"
            "  home            Saw OFF, J1/J2/J3/J4 -> 0 deg\n"
            "  status          Show state (allowed while BUSY)\n"
            "  help            Show help (allowed while BUSY)\n"
            "  quit            Saw OFF and exit (IDLE only)\n\n"
            "Joint Limits:\n"
            "  J1: -180 deg ~ +180 deg\n"
            "  J2:  -90 deg ~  +90 deg\n"
            "  J3:  -90 deg ~  +90 deg\n"
            "  J4: -180 deg ~ +180 deg\n\n"
            "Trapezoidal Trajectory Limits:\n"
            "  J1: vmax 45 deg/s, amax 30 deg/s^2\n"
            "  J2: vmax 30 deg/s, amax 25 deg/s^2\n"
            "  J3: vmax 45 deg/s, amax 35 deg/s^2\n"
            "  J4: vmax 60 deg/s, amax 40 deg/s^2\n"
            "  Command period: 20 ms\n"
            "  Short moves automatically use triangular velocity profiles\n\n"
            "BUSY Policy:\n"
            "  Joint motion / home / test / saw start / quit -> REJECTED while BUSY\n"
            "  saw off / saw 0 / status / help              -> always allowed\n"
            "===============================================";
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
