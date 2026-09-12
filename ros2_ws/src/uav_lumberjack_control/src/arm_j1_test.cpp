#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ArmJ1Test : public rclcpp::Node
{
public:
    ArmJ1Test()
    : Node("arm_j1_test")
    {
        // 发布 J1 的目标关节角
        j1_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/lumberjack_arm/j1/cmd_pos",
            10
        );

        // 每 500 ms 运行一次状态机
        timer_ = this->create_wall_timer(
            500ms,
            std::bind(&ArmJ1Test::timer_callback, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Arm J1 test node started."
        );
    }

private:
    // 角度转弧度
    static constexpr double DEG_TO_RAD =
        3.14159265358979323846 / 180.0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j1_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int state_ = 0;
    int wait_count_ = 0;

    void publish_j1(double angle_deg)
    {
        std_msgs::msg::Float64 msg;

        msg.data = angle_deg * DEG_TO_RAD;

        j1_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "J1 command: %.1f deg (%.6f rad)",
            angle_deg,
            msg.data
        );
    }

    void timer_callback()
    {
        // State 0:
        // 等待 ros_gz_bridge 的订阅端出现
        if (state_ == 0)
        {
            if (j1_pub_->get_subscription_count() == 0)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Waiting for ros_gz_bridge..."
                );

                return;
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Bridge detected."
            );

            publish_j1(45.0);

            state_ = 1;
            wait_count_ = 0;

            return;
        }

        // State 1:
        // 保持 +45° 约 2 秒
        if (state_ == 1)
        {
            wait_count_++;

            // 500 ms × 4 = 2 s
            if (wait_count_ >= 4)
            {
                publish_j1(0.0);

                state_ = 2;
                wait_count_ = 0;
            }

            return;
        }

        // State 2:
        // 回零后等待约 1 秒，再退出
        if (state_ == 2)
        {
            wait_count_++;

            if (wait_count_ >= 2)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "J1 test completed."
                );

                rclcpp::shutdown();
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArmJ1Test>();

    rclcpp::spin(node);

    return 0;
}
