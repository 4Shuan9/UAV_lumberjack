#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class AutoCutController : public rclcpp::Node
{
public:
  AutoCutController()
  : Node("auto_cut_controller")
  {
    contact_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/lumberjack_arm/target_contact", 10,
      std::bind(&AutoCutController::contact_callback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/lumberjack_arm/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&AutoCutController::joint_state_callback, this, std::placeholders::_1));

    detach_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/target_branch/detach", 10);

    progress_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/lumberjack_arm/cut_progress", 10);

    success_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/lumberjack_arm/cut_success", 10);

    timer_ = create_wall_timer(
      20ms, std::bind(&AutoCutController::update, this));

    heartbeat_timer_ = create_wall_timer(
      200ms, std::bind(&AutoCutController::publish_status, this));

    last_update_time_ = now();
    last_contact_time_ = now();

    RCLCPP_INFO(get_logger(), "Auto cut controller started.");
    RCLCPP_INFO(get_logger(), "Saw threshold     : %.0f rpm", kSawThresholdRpm);
    RCLCPP_INFO(get_logger(), "Required contact  : %.2f s effective", kRequiredContactSec);
    RCLCPP_INFO(get_logger(), "Contact gap grace : %.2f s", kContactGapGraceSec);
    RCLCPP_INFO(get_logger(), "Detach topic      : /target_branch/detach");
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kRadSToRpm = 60.0 / (2.0 * kPi);
  static constexpr double kSawThresholdRpm = 800.0;
  static constexpr double kRequiredContactSec = 0.80;
  static constexpr double kContactGapGraceSec = 0.25;

  void contact_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    target_contact_ = msg->data;
    if (target_contact_) {
      last_contact_time_ = now();
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::size_t count = std::min(msg->name.size(), msg->velocity.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (msg->name[i] == "saw_spin_joint") {
        saw_actual_rpm_ = std::abs(msg->velocity[i]) * kRadSToRpm;
        saw_feedback_received_ = true;
        return;
      }
    }
  }

  void reset_progress(const char * reason)
  {
    if (effective_contact_sec_ > 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "CUT RESET: %s (progress %.0f%%)",
        reason,
        100.0 * effective_contact_sec_ / kRequiredContactSec);
    }
    effective_contact_sec_ = 0.0;
    next_progress_log_sec_ = 0.20;
  }

  void update()
  {
    const auto t_now = now();
    double dt = (t_now - last_update_time_).seconds();
    last_update_time_ = t_now;

    // Protect against a long scheduler pause from being counted as cutting time.
    dt = std::clamp(dt, 0.0, 0.10);

    if (cut_success_) {
      return;
    }

    if (!saw_feedback_received_) {
      return;
    }

    const bool saw_ready = saw_actual_rpm_ >= kSawThresholdRpm;

    // Saw stopped or too slow: cutting progress must restart from zero.
    if (!saw_ready) {
      if (effective_contact_sec_ > 0.0) {
        reset_progress("saw below 800 rpm");
      }
      return;
    }

    if (target_contact_) {
      effective_contact_sec_ += dt;
      last_contact_time_ = t_now;

      if (!cutting_announced_) {
        cutting_announced_ = true;
        RCLCPP_INFO(
          get_logger(),
          "CUTTING START: target contact + saw %.0f rpm",
          saw_actual_rpm_);
      }

      if (effective_contact_sec_ >= next_progress_log_sec_ &&
          effective_contact_sec_ < kRequiredContactSec)
      {
        RCLCPP_INFO(
          get_logger(),
          "CUTTING: %.2f / %.2f s (%.0f%%), saw %.0f rpm",
          effective_contact_sec_,
          kRequiredContactSec,
          100.0 * effective_contact_sec_ / kRequiredContactSec,
          saw_actual_rpm_);
        next_progress_log_sec_ += 0.20;
      }
    } else {
      cutting_announced_ = false;

      if (effective_contact_sec_ > 0.0) {
        const double gap_sec = (t_now - last_contact_time_).seconds();
        if (gap_sec > kContactGapGraceSec) {
          reset_progress("target contact lost for more than 0.25 s");
        }
      }
    }

    if (effective_contact_sec_ >= kRequiredContactSec) {
      trigger_cut_success();
    }
  }

  void trigger_cut_success()
  {
    if (cut_success_) {
      return;
    }

    cut_success_ = true;
    effective_contact_sec_ = kRequiredContactSec;

    std_msgs::msg::Empty detach_msg;
    detach_pub_->publish(detach_msg);

    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "CUT SUCCESS");
    RCLCPP_INFO(get_logger(), "Saw actual : %.0f rpm", saw_actual_rpm_);
    RCLCPP_INFO(get_logger(), "Effective contact reached %.2f s", kRequiredContactSec);
    RCLCPP_INFO(get_logger(), "Published /target_branch/detach");
    RCLCPP_INFO(get_logger(), "Cut is now latched; detach will not repeat.");
    RCLCPP_INFO(get_logger(), "========================================");

    publish_status();
  }

  void publish_status()
  {
    std_msgs::msg::Float64 progress_msg;
    progress_msg.data = cut_success_ ? 1.0 :
      std::clamp(effective_contact_sec_ / kRequiredContactSec, 0.0, 1.0);
    progress_pub_->publish(progress_msg);

    std_msgs::msg::Bool success_msg;
    success_msg.data = cut_success_;
    success_pub_->publish(success_msg);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr detach_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr progress_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  bool target_contact_{false};
  bool saw_feedback_received_{false};
  bool cut_success_{false};
  bool cutting_announced_{false};
  double saw_actual_rpm_{0.0};
  double effective_contact_sec_{0.0};
  double next_progress_log_sec_{0.20};
  rclcpp::Time last_update_time_;
  rclcpp::Time last_contact_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoCutController>());
  rclcpp::shutdown();
  return 0;
}
