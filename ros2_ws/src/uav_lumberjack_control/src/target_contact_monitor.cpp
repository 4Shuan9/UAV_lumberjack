#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class TargetContactMonitor : public rclcpp::Node
{
public:
  TargetContactMonitor()
  : Node("target_contact_monitor"), contact_active_(false)
  {
    const std::vector<std::string> topics = {
      "/lumberjack_arm/cutting_zone/top/contacts",
      "/lumberjack_arm/cutting_zone/bottom/contacts",
      "/lumberjack_arm/cutting_zone/rear/contacts",
      "/lumberjack_arm/cutting_zone/front/contacts"
    };

    for (const auto & topic : topics) {
      contact_subs_.push_back(
        create_subscription<ros_gz_interfaces::msg::Contacts>(
          topic,
          rclcpp::SensorDataQoS(),
          std::bind(&TargetContactMonitor::contact_callback, this, std::placeholders::_1)));
      RCLCPP_INFO(get_logger(), "Listening: %s", topic.c_str());
    }

    contact_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/lumberjack_arm/target_contact", 10);

    timeout_timer_ = create_wall_timer(
      50ms, std::bind(&TargetContactMonitor::check_contact_timeout, this));

    // Publish current state continuously so `ros2 topic echo` always
    // shows whether the monitor is alive, even before first contact.
    heartbeat_timer_ = create_wall_timer(
      200ms, std::bind(&TargetContactMonitor::publish_state, this));

    last_target_contact_time_ = now();

    RCLCPP_INFO(get_logger(), "Target contact monitor started.");
    RCLCPP_INFO(
      get_logger(),
      "Target: target_branch::target_branch_link::target_branch_outer_collision");
  }

private:
  static bool is_target_collision(const std::string & name)
  {
    return name.find("target_branch::target_branch_link::target_branch_outer_collision") !=
           std::string::npos;
  }

  void contact_callback(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    bool target_detected = false;
    std::string collision_a;
    std::string collision_b;

    for (const auto & contact : msg->contacts) {
      const auto & name1 = contact.collision1.name;
      const auto & name2 = contact.collision2.name;

      if (is_target_collision(name1) || is_target_collision(name2)) {
        target_detected = true;
        collision_a = name1;
        collision_b = name2;
        break;
      }
    }

    if (!target_detected) {
      return;
    }

    last_target_contact_time_ = now();

    if (!contact_active_) {
      contact_active_ = true;
      publish_state();

      RCLCPP_INFO(get_logger(), "========================================");
      RCLCPP_INFO(get_logger(), "CONTACT TRUE");
      RCLCPP_INFO(get_logger(), "collision1: %s", collision_a.c_str());
      RCLCPP_INFO(get_logger(), "collision2: %s", collision_b.c_str());
      RCLCPP_INFO(get_logger(), "========================================");
    }
  }

  void check_contact_timeout()
  {
    constexpr double kContactTimeoutSec = 0.20;

    if (contact_active_ &&
        (now() - last_target_contact_time_).seconds() > kContactTimeoutSec)
    {
      contact_active_ = false;
      publish_state();
      RCLCPP_INFO(get_logger(), "CONTACT FALSE");
    }
  }

  void publish_state()
  {
    std_msgs::msg::Bool msg;
    msg.data = contact_active_;
    contact_pub_->publish(msg);
  }

  std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> contact_subs_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  bool contact_active_;
  rclcpp::Time last_target_contact_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetContactMonitor>());
  rclcpp::shutdown();
  return 0;
}
