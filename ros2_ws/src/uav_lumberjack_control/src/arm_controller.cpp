#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
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

    ArmController()
    : Node("arm_controller")
    {
        // ==================================================
        // Joint Command Publishers
        // ==================================================

        j1_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/lumberjack_arm/j1/cmd_pos",
                10
            );

        j2_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/lumberjack_arm/j2/cmd_pos",
                10
            );

        j3_pub_ =
            this->create_publisher<std_msgs::msg::Float64>(
                "/lumberjack_arm/j3/cmd_pos",
                10
            );


        // ==================================================
        // Joint State Subscriber
        // ==================================================

        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/lumberjack_arm/joint_states",
                rclcpp::SensorDataQoS(),

                std::bind(
                    &ArmController::joint_state_callback,
                    this,
                    std::placeholders::_1
                )
            );


        RCLCPP_INFO(
            this->get_logger(),
            "UAV_lumberjack Arm Controller started."
        );
    }



    // ======================================================
    // Interactive Terminal
    // ======================================================

    void run()
    {
        wait_for_bridge();

        wait_for_joint_feedback();

        print_help();


        while (rclcpp::ok())
        {
            std::cout
                << "\n[IDLE] > "
                << std::flush;


            std::string line;


            if (!std::getline(std::cin, line))
            {
                break;
            }


            if (line.empty())
            {
                continue;
            }


            std::istringstream iss(line);

            std::string command;

            iss >> command;


            // ==================================================
            // quit / exit
            // ==================================================

            if (command == "quit" ||
                command == "exit")
            {
                std::cout
                    << "Exit arm controller."
                    << std::endl;

                break;
            }


            // ==================================================
            // help
            // ==================================================

            if (command == "help")
            {
                print_help();

                continue;
            }


            // ==================================================
            // status
            // ==================================================

            if (command == "status")
            {
                print_status();

                continue;
            }


            // ==================================================
            // home
            // ==================================================

            if (command == "home")
            {
                execute_home();

                continue;
            }


            // ==================================================
            // Joint Command
            // ==================================================

            if (command == "j1" ||
                command == "j2" ||
                command == "j3")
            {
                double angle_deg;


                if (!(iss >> angle_deg))
                {
                    std::cout
                        << "Invalid command."
                        << std::endl;

                    std::cout
                        << "Example: j1 45"
                        << std::endl;

                    continue;
                }


                execute_joint_command(
                    command,
                    angle_deg
                );

                continue;
            }


            // ==================================================
            // Unknown Command
            // ==================================================

            std::cout
                << "Unknown command."
                << std::endl;

            std::cout
                << "Type 'help' to show available commands."
                << std::endl;
        }
    }



private:

    // ======================================================
    // Mathematical Constants
    // ======================================================

    static constexpr double PI =
        3.14159265358979323846;


    static constexpr double DEG_TO_RAD =
        PI / 180.0;


    static constexpr double RAD_TO_DEG =
        180.0 / PI;



    // ======================================================
    // Joint Limits [deg]
    // ======================================================

    static constexpr double J1_MIN_DEG = -180.0;
    static constexpr double J1_MAX_DEG =  180.0;

    static constexpr double J2_MIN_DEG = -90.0;
    static constexpr double J2_MAX_DEG =  90.0;

    static constexpr double J3_MIN_DEG = -90.0;
    static constexpr double J3_MAX_DEG =  90.0;



    // ======================================================
    // Motion Completion Parameters
    //
    // 一个动作必须同时满足：
    //
    // 1. Position Error 足够小
    // 2. Velocity 足够小
    // 3. 连续满足若干次
    //
    // 才算真正 REACHED。
    // ======================================================

    static constexpr double POSITION_TOLERANCE_DEG =
        1.0;


    static constexpr double VELOCITY_TOLERANCE_DEG_S =
        1.0;


    static constexpr int STABLE_CYCLES_REQUIRED =
        10;


    static constexpr int CHECK_PERIOD_MS =
        50;


    static constexpr double MOTION_TIMEOUT_SEC =
        10.0;



    // ======================================================
    // ROS Publishers
    // ======================================================

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        j1_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        j2_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        j3_pub_;



    // ======================================================
    // ROS Subscriber
    // ======================================================

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_sub_;



    // ======================================================
    // Commanded Joint Position [deg]
    // ======================================================

    double j1_cmd_deg_ = 0.0;
    double j2_cmd_deg_ = 0.0;
    double j3_cmd_deg_ = 0.0;



    // ======================================================
    // Actual Joint Position [deg]
    // ======================================================

    double j1_actual_deg_ = 0.0;
    double j2_actual_deg_ = 0.0;
    double j3_actual_deg_ = 0.0;



    // ======================================================
    // Actual Joint Velocity [deg/s]
    // ======================================================

    double j1_velocity_deg_s_ = 0.0;
    double j2_velocity_deg_s_ = 0.0;
    double j3_velocity_deg_s_ = 0.0;



    // ======================================================
    // Feedback Flags
    // ======================================================

    bool j1_feedback_received_ = false;
    bool j2_feedback_received_ = false;
    bool j3_feedback_received_ = false;



    // ======================================================
    // Mutex
    // ======================================================

    std::mutex joint_state_mutex_;



    // ======================================================
    // Joint State Callback
    // ======================================================

    void joint_state_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex>
            lock(joint_state_mutex_);


        const std::size_t count =
            std::min(
                msg->name.size(),
                msg->position.size()
            );


        for (std::size_t i = 0;
             i < count;
             ++i)
        {
            const double position_deg =
                msg->position[i] *
                RAD_TO_DEG;


            double velocity_deg_s = 0.0;


            if (i < msg->velocity.size())
            {
                velocity_deg_s =
                    msg->velocity[i] *
                    RAD_TO_DEG;
            }


            // ==============================================
            // J1
            // ==============================================

            if (msg->name[i] == "j1")
            {
                j1_actual_deg_ =
                    position_deg;

                j1_velocity_deg_s_ =
                    velocity_deg_s;

                j1_feedback_received_ =
                    true;
            }


            // ==============================================
            // J2
            // ==============================================

            else if (msg->name[i] == "j2")
            {
                j2_actual_deg_ =
                    position_deg;

                j2_velocity_deg_s_ =
                    velocity_deg_s;

                j2_feedback_received_ =
                    true;
            }


            // ==============================================
            // J3
            // ==============================================

            else if (msg->name[i] == "j3")
            {
                j3_actual_deg_ =
                    position_deg;

                j3_velocity_deg_s_ =
                    velocity_deg_s;

                j3_feedback_received_ =
                    true;
            }
        }
    }



    // ======================================================
    // Wait For Command Bridges
    // ======================================================

    void wait_for_bridge()
    {
        std::cout
            << "Waiting for ros_gz_bridge command channels..."
            << std::endl;


        while (rclcpp::ok())
        {
            const bool j1_ready =
                j1_pub_->get_subscription_count() > 0;

            const bool j2_ready =
                j2_pub_->get_subscription_count() > 0;

            const bool j3_ready =
                j3_pub_->get_subscription_count() > 0;


            if (j1_ready &&
                j2_ready &&
                j3_ready)
            {
                std::cout
                    << "All command bridges detected."
                    << std::endl;

                return;
            }


            std::this_thread::sleep_for(
                500ms
            );
        }
    }



    // ======================================================
    // Wait For Initial Joint Feedback
    // ======================================================

    void wait_for_joint_feedback()
    {
        std::cout
            << "Waiting for joint state feedback..."
            << std::endl;


        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex>
                    lock(joint_state_mutex_);


                if (j1_feedback_received_ &&
                    j2_feedback_received_ &&
                    j3_feedback_received_)
                {
                    std::cout
                        << "Joint state feedback detected."
                        << std::endl;

                    return;
                }
            }


            std::this_thread::sleep_for(
                100ms
            );
        }
    }



    // ======================================================
    // Joint Limit Check
    // ======================================================

    bool check_joint_limit(
        const std::string & joint,
        double angle_deg)
    {
        double min_deg = 0.0;
        double max_deg = 0.0;


        if (joint == "j1")
        {
            min_deg =
                J1_MIN_DEG;

            max_deg =
                J1_MAX_DEG;
        }

        else if (joint == "j2")
        {
            min_deg =
                J2_MIN_DEG;

            max_deg =
                J2_MAX_DEG;
        }

        else if (joint == "j3")
        {
            min_deg =
                J3_MIN_DEG;

            max_deg =
                J3_MAX_DEG;
        }

        else
        {
            std::cout
                << "Unknown joint: "
                << joint
                << std::endl;

            return false;
        }


        if (angle_deg < min_deg ||
            angle_deg > max_deg)
        {
            std::cout
                << "Rejected: "
                << joint
                << " angle must be within ["
                << min_deg
                << ", "
                << max_deg
                << "] deg."
                << std::endl;

            return false;
        }


        return true;
    }



    // ======================================================
    // Publish Joint Command
    // ======================================================

    void publish_joint(
        const std::string & joint,
        double angle_deg)
    {
        std_msgs::msg::Float64 msg;


        msg.data =
            angle_deg *
            DEG_TO_RAD;


        if (joint == "j1")
        {
            j1_pub_->publish(msg);

            j1_cmd_deg_ =
                angle_deg;
        }

        else if (joint == "j2")
        {
            j2_pub_->publish(msg);

            j2_cmd_deg_ =
                angle_deg;
        }

        else if (joint == "j3")
        {
            j3_pub_->publish(msg);

            j3_cmd_deg_ =
                angle_deg;
        }
    }



    // ======================================================
    // Get One Joint Feedback
    //
    // 返回：
    //
    // actual position [deg]
    // actual velocity [deg/s]
    // ======================================================

    bool get_joint_feedback(
        const std::string & joint,
        double & actual_deg,
        double & velocity_deg_s)
    {
        std::lock_guard<std::mutex>
            lock(joint_state_mutex_);


        if (joint == "j1")
        {
            if (!j1_feedback_received_)
            {
                return false;
            }


            actual_deg =
                j1_actual_deg_;

            velocity_deg_s =
                j1_velocity_deg_s_;

            return true;
        }


        if (joint == "j2")
        {
            if (!j2_feedback_received_)
            {
                return false;
            }


            actual_deg =
                j2_actual_deg_;

            velocity_deg_s =
                j2_velocity_deg_s_;

            return true;
        }


        if (joint == "j3")
        {
            if (!j3_feedback_received_)
            {
                return false;
            }


            actual_deg =
                j3_actual_deg_;

            velocity_deg_s =
                j3_velocity_deg_s_;

            return true;
        }


        return false;
    }



    // ======================================================
    // Wait Until One Joint Reaches Target
    //
    // Completion condition:
    //
    // |position error| < tolerance
    //
    // AND
    //
    // |velocity| < tolerance
    //
    // continuously for N cycles.
    // ======================================================

    bool wait_until_joint_reached(
        const std::string & joint,
        double target_deg)
    {
        int stable_cycles = 0;


        const auto start_time =
            std::chrono::steady_clock::now();


        while (rclcpp::ok())
        {
            double actual_deg = 0.0;
            double velocity_deg_s = 0.0;


            const bool feedback_ok =
                get_joint_feedback(
                    joint,
                    actual_deg,
                    velocity_deg_s
                );


            if (!feedback_ok)
            {
                std::cout
                    << "[ERROR] "
                    << joint
                    << " feedback unavailable."
                    << std::endl;

                return false;
            }


            const double error_deg =
                target_deg -
                actual_deg;


            // ==============================================
            // Check position + velocity
            // ==============================================

            if (
                std::abs(error_deg)
                    <= POSITION_TOLERANCE_DEG
                &&
                std::abs(velocity_deg_s)
                    <= VELOCITY_TOLERANCE_DEG_S
            )
            {
                stable_cycles++;
            }

            else
            {
                stable_cycles = 0;
            }


            // ==============================================
            // Stable long enough -> reached
            // ==============================================

            if (
                stable_cycles
                    >= STABLE_CYCLES_REQUIRED
            )
            {
                std::cout
                    << "[REACHED] "
                    << joint
                    << " target = "
                    << std::fixed
                    << std::setprecision(2)
                    << target_deg
                    << " deg, actual = "
                    << actual_deg
                    << " deg, error = "
                    << error_deg
                    << " deg"
                    << std::endl;


                return true;
            }


            // ==============================================
            // Timeout Check
            // ==============================================

            const auto now =
                std::chrono::steady_clock::now();


            const double elapsed_sec =
                std::chrono::duration<double>(
                    now -
                    start_time
                ).count();


            if (
                elapsed_sec
                    >= MOTION_TIMEOUT_SEC
            )
            {
                std::cout
                    << "[TIMEOUT] "
                    << joint
                    << " failed to reach target."
                    << std::endl;


                std::cout
                    << "Target   = "
                    << target_deg
                    << " deg"
                    << std::endl;


                std::cout
                    << "Actual   = "
                    << actual_deg
                    << " deg"
                    << std::endl;


                std::cout
                    << "Error    = "
                    << error_deg
                    << " deg"
                    << std::endl;


                std::cout
                    << "Velocity = "
                    << velocity_deg_s
                    << " deg/s"
                    << std::endl;


                return false;
            }


            // ==============================================
            // Polling Period
            // ==============================================

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CHECK_PERIOD_MS
                )
            );
        }


        return false;
    }



    // ======================================================
    // Wait Until HOME Reached
    //
    // All three joints must simultaneously satisfy:
    //
    // position tolerance
    // +
    // velocity tolerance
    // ======================================================

    bool wait_until_home_reached()
    {
        int stable_cycles = 0;


        const auto start_time =
            std::chrono::steady_clock::now();


        while (rclcpp::ok())
        {
            double j1_actual;
            double j2_actual;
            double j3_actual;

            double j1_velocity;
            double j2_velocity;
            double j3_velocity;


            {
                std::lock_guard<std::mutex>
                    lock(joint_state_mutex_);


                j1_actual =
                    j1_actual_deg_;

                j2_actual =
                    j2_actual_deg_;

                j3_actual =
                    j3_actual_deg_;


                j1_velocity =
                    j1_velocity_deg_s_;

                j2_velocity =
                    j2_velocity_deg_s_;

                j3_velocity =
                    j3_velocity_deg_s_;
            }


            const double j1_error =
                0.0 -
                j1_actual;

            const double j2_error =
                0.0 -
                j2_actual;

            const double j3_error =
                0.0 -
                j3_actual;


            const bool j1_reached =
                std::abs(j1_error)
                    <= POSITION_TOLERANCE_DEG
                &&
                std::abs(j1_velocity)
                    <= VELOCITY_TOLERANCE_DEG_S;


            const bool j2_reached =
                std::abs(j2_error)
                    <= POSITION_TOLERANCE_DEG
                &&
                std::abs(j2_velocity)
                    <= VELOCITY_TOLERANCE_DEG_S;


            const bool j3_reached =
                std::abs(j3_error)
                    <= POSITION_TOLERANCE_DEG
                &&
                std::abs(j3_velocity)
                    <= VELOCITY_TOLERANCE_DEG_S;


            if (
                j1_reached &&
                j2_reached &&
                j3_reached
            )
            {
                stable_cycles++;
            }

            else
            {
                stable_cycles = 0;
            }


            // ==============================================
            // HOME reached
            // ==============================================

            if (
                stable_cycles
                    >= STABLE_CYCLES_REQUIRED
            )
            {
                std::cout
                    << "[REACHED] HOME"
                    << std::endl;


                return true;
            }


            // ==============================================
            // Timeout
            // ==============================================

            const auto now =
                std::chrono::steady_clock::now();


            const double elapsed_sec =
                std::chrono::duration<double>(
                    now -
                    start_time
                ).count();


            if (
                elapsed_sec
                    >= MOTION_TIMEOUT_SEC
            )
            {
                std::cout
                    << "[TIMEOUT] HOME failed."
                    << std::endl;


                std::cout
                    << "J1 actual = "
                    << j1_actual
                    << " deg"
                    << std::endl;


                std::cout
                    << "J2 actual = "
                    << j2_actual
                    << " deg"
                    << std::endl;


                std::cout
                    << "J3 actual = "
                    << j3_actual
                    << " deg"
                    << std::endl;


                return false;
            }


            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    CHECK_PERIOD_MS
                )
            );
        }


        return false;
    }



    // ======================================================
    // Execute One Joint Command
    // ======================================================

    void execute_joint_command(
        const std::string & joint,
        double angle_deg)
    {
        // --------------------------------------------------
        // Safety Limit
        // --------------------------------------------------

        if (!check_joint_limit(
                joint,
                angle_deg))
        {
            return;
        }


        // --------------------------------------------------
        // Previous command
        // --------------------------------------------------

        double old_angle_deg = 0.0;


        if (joint == "j1")
        {
            old_angle_deg =
                j1_cmd_deg_;
        }

        else if (joint == "j2")
        {
            old_angle_deg =
                j2_cmd_deg_;
        }

        else if (joint == "j3")
        {
            old_angle_deg =
                j3_cmd_deg_;
        }


        // --------------------------------------------------
        // BUSY
        // --------------------------------------------------

        std::cout
            << "\n[BUSY] "
            << joint
            << ": "
            << old_angle_deg
            << " deg -> "
            << angle_deg
            << " deg"
            << std::endl;


        // --------------------------------------------------
        // Publish target
        // --------------------------------------------------

        publish_joint(
            joint,
            angle_deg
        );


        // --------------------------------------------------
        // Real feedback-based completion
        // --------------------------------------------------

        const bool reached =
            wait_until_joint_reached(
                joint,
                angle_deg
            );


        if (reached)
        {
            std::cout
                << "Done. [IDLE]"
                << std::endl;
        }

        else
        {
            std::cout
                << "Motion ended with timeout/error. [IDLE]"
                << std::endl;
        }
    }



    // ======================================================
    // HOME
    // ======================================================

    void execute_home()
    {
        std::cout
            << "\n[BUSY] Returning arm to HOME..."
            << std::endl;


        // --------------------------------------------------
        // Publish all three targets simultaneously
        // --------------------------------------------------

        publish_joint(
            "j1",
            0.0
        );

        publish_joint(
            "j2",
            0.0
        );

        publish_joint(
            "j3",
            0.0
        );


        // --------------------------------------------------
        // Wait using real feedback
        // --------------------------------------------------

        const bool reached =
            wait_until_home_reached();


        if (reached)
        {
            std::cout
                << "HOME reached. [IDLE]"
                << std::endl;
        }

        else
        {
            std::cout
                << "HOME ended with timeout/error. [IDLE]"
                << std::endl;
        }
    }



    // ======================================================
    // STATUS
    // ======================================================

    void print_status()
    {
        double j1_actual;
        double j2_actual;
        double j3_actual;

        double j1_velocity;
        double j2_velocity;
        double j3_velocity;


        {
            std::lock_guard<std::mutex>
                lock(joint_state_mutex_);


            j1_actual =
                j1_actual_deg_;

            j2_actual =
                j2_actual_deg_;

            j3_actual =
                j3_actual_deg_;


            j1_velocity =
                j1_velocity_deg_s_;

            j2_velocity =
                j2_velocity_deg_s_;

            j3_velocity =
                j3_velocity_deg_s_;
        }


        const double j1_error =
            j1_cmd_deg_ -
            j1_actual;

        const double j2_error =
            j2_cmd_deg_ -
            j2_actual;

        const double j3_error =
            j3_cmd_deg_ -
            j3_actual;


        std::cout
            << "\n"
            << "====================================================================\n"
            << "                  UAV_lumberjack Arm State\n"
            << "====================================================================\n"
            << std::fixed
            << std::setprecision(2)
            << "\n"
            << "Joint   Commanded     Actual       Error       Velocity\n"
            << "--------------------------------------------------------------------\n";


        std::cout
            << "J1      "
            << std::setw(8)
            << j1_cmd_deg_
            << " deg   "
            << std::setw(8)
            << j1_actual
            << " deg   "
            << std::setw(8)
            << j1_error
            << " deg   "
            << std::setw(8)
            << j1_velocity
            << " deg/s\n";


        std::cout
            << "J2      "
            << std::setw(8)
            << j2_cmd_deg_
            << " deg   "
            << std::setw(8)
            << j2_actual
            << " deg   "
            << std::setw(8)
            << j2_error
            << " deg   "
            << std::setw(8)
            << j2_velocity
            << " deg/s\n";


        std::cout
            << "J3      "
            << std::setw(8)
            << j3_cmd_deg_
            << " deg   "
            << std::setw(8)
            << j3_actual
            << " deg   "
            << std::setw(8)
            << j3_error
            << " deg   "
            << std::setw(8)
            << j3_velocity
            << " deg/s\n";


        std::cout
            << "\n"
            << "===================================================================="
            << std::endl;
    }



    // ======================================================
    // HELP
    // ======================================================

    void print_help()
    {
        std::cout
            << "\n"
            << "========================================\n"
            << " UAV_lumberjack 3-DOF Arm Controller\n"
            << "========================================\n"
            << "\n"
            << "Joint Commands:\n"
            << "\n"
            << "  j1 <deg>     Base Yaw\n"
            << "  j2 <deg>     Shoulder Pitch\n"
            << "  j3 <deg>     Elbow Pitch\n"
            << "\n"
            << "Other Commands:\n"
            << "\n"
            << "  home         All joints -> 0 deg\n"
            << "  status       Show command / actual / error / velocity\n"
            << "  help         Show this help\n"
            << "  quit         Exit controller\n"
            << "\n"
            << "Joint Limits:\n"
            << "\n"
            << "  J1: -180 deg ~ +180 deg\n"
            << "  J2:  -90 deg ~  +90 deg\n"
            << "  J3:  -90 deg ~  +90 deg\n"
            << "\n"
            << "Motion Completion:\n"
            << "\n"
            << "  Position tolerance : +/- 1.0 deg\n"
            << "  Velocity tolerance : +/- 1.0 deg/s\n"
            << "  Stable cycles      : 10\n"
            << "  Check period       : 50 ms\n"
            << "  Timeout            : 8 s\n"
            << "\n"
            << "========================================"
            << std::endl;
    }
};



// ==========================================================
// Main
//
// Thread 1:
// Terminal interaction + motion waiting
//
// Thread 2:
// ROS spin + JointState feedback
// ==========================================================

int main(
    int argc,
    char * argv[])
{
    rclcpp::init(
        argc,
        argv
    );


    auto node =
        std::make_shared<ArmController>();


    // ======================================================
    // ROS Callback Thread
    // ======================================================

    std::thread ros_spin_thread(
        [node]()
        {
            rclcpp::spin(node);
        }
    );


    // ======================================================
    // Terminal Main Thread
    // ======================================================

    node->run();


    // ======================================================
    // Shutdown
    // ======================================================

    rclcpp::shutdown();


    if (ros_spin_thread.joinable())
    {
        ros_spin_thread.join();
    }


    return 0;
}