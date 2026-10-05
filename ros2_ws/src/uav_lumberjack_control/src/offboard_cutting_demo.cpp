#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uav_lumberjack_interfaces/action/arm_motion.hpp"
#include "uav_lumberjack_interfaces/msg/arm_status.hpp"
#include "uav_lumberjack_interfaces/msg/branch_model.hpp"
#include "uav_lumberjack_interfaces/srv/set_saw.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kControlDt = 0.02;  // 50 Hz
constexpr std::uint8_t kArmModeHome = 0;
constexpr std::uint8_t kArmModePrework = 1;

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(value, hi));
}

double wrap_pi(double angle)
{
  while (angle > kPi) angle -= kTwoPi;
  while (angle < -kPi) angle += kTwoPi;
  return angle;
}

double deg2rad(double deg)
{
  return deg * kPi / 180.0;
}

double rad2deg(double rad)
{
  return rad * 180.0 / kPi;
}

double norm2d(double x, double y)
{
  return std::hypot(x, y);
}

double norm3d(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 operator+(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3 & v, double s)
{
  return {v.x * s, v.y * s, v.z * s};
}

double yaw_from_xyzw(double x, double y, double z, double w)
{
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

double yaw_from_wxyz(const std::array<float, 4> & q)
{
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];
  return std::atan2(
    2.0 * (w * z + x * y),
    1.0 - 2.0 * (y * y + z * z));
}

std::string fmt_vec(const Vec3 & v, int precision = 3)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision)
      << "[" << v.x << ", " << v.y << ", " << v.z << "]";
  return oss.str();
}

}  // namespace

class OffboardCuttingDemo : public rclcpp::Node
{
public:
  using ArmMotion = uav_lumberjack_interfaces::action::ArmMotion;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<ArmMotion>;
  using ArmStatus = uav_lumberjack_interfaces::msg::ArmStatus;
  using BranchModel = uav_lumberjack_interfaces::msg::BranchModel;
  using SetSaw = uav_lumberjack_interfaces::srv::SetSaw;

  OffboardCuttingDemo()
  : Node("offboard_cutting_demo")
  {
    declare_parameters();
    load_parameters();
    create_ros_interfaces();

    control_timer_ = create_wall_timer(
      20ms, std::bind(&OffboardCuttingDemo::control_loop, this));

    start_cli_thread();
    print_startup();
  }

  ~OffboardCuttingDemo() override
  {
    stop_cli_.store(true);
    if (cli_thread_.joinable()) {
      // stdin may be blocked in getline(); detaching avoids shutdown deadlock.
      cli_thread_.detach();
    }
  }

private:
  enum class Phase
  {
    IDLE,
    ARM_HOME_PREP,
    OFFBOARD_PRESTREAM,
    TAKEOFF,
    OBSERVE,
    ARC_SCAN,
    TARGET_LOCK,
    FAR_APPROACH,
    ARM_PREWORK,
    NEAR_APPROACH,
    SAW_SPINUP,
    CUT_IN,
    CUT_WAIT,
    RETREAT_NEAR,
    RETREAT_FAR,
    ARM_HOME_POST,
    RETURN_HOME,
    LANDING,
    HOLD,
    PAUSED,
    CUT_FAILED,
    DONE
  };

  struct BranchSample
  {
    Vec3 center;
    Vec3 direction;
    double length{0.0};
    double radius{0.0};
    double fit_rms{0.0};
    std::uint32_t point_count{0};
  };

  struct LockedTarget
  {
    bool valid{false};
    Vec3 center;
    Vec3 direction;
    double length{0.0};
    double radius{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // ============================================================
  // Parameters
  // ============================================================

  void declare_parameters()
  {
    declare_parameter("tree_hint_x", 4.0);
    declare_parameter("tree_hint_y", 0.1);
    declare_parameter("takeoff_height", 3.0);
    declare_parameter("observe_distance", 1.4);
    declare_parameter("observe_bearing_offset_deg", 17.0);
    declare_parameter("arc_half_angle_deg", 25.0);
    declare_parameter("arc_settle_sec", 1.0);

    declare_parameter("perception_min_points", 60);
    declare_parameter("perception_max_fit_rms", 0.03);
    declare_parameter("perception_stable_samples", 5);
    declare_parameter("perception_center_spread_max", 0.05);

    declare_parameter("cut_z_offset", 0.455);
    declare_parameter("tool_forward_offset", 0.19);
    declare_parameter("near_standoff", 0.20);
    declare_parameter("far_standoff", 0.75);
    declare_parameter("cut_in_penetration", 0.04);
    declare_parameter("cut_in_max_distance", 0.30);

    declare_parameter("takeoff_speed", 0.50);
    declare_parameter("observe_speed", 0.45);
    declare_parameter("far_speed", 0.55);
    declare_parameter("near_speed", 0.18);
    declare_parameter("cut_in_speed", 0.04);
    declare_parameter("retreat_speed", 0.12);
    declare_parameter("return_speed", 0.60);
    declare_parameter("yaw_rate_deg_s", 35.0);

    declare_parameter("position_tolerance", 0.08);
    declare_parameter("near_position_tolerance", 0.04);
    declare_parameter("yaw_tolerance_deg", 5.0);

    declare_parameter("saw_command_rpm", 1000.0);
    declare_parameter("saw_ready_rpm", 850.0);
    declare_parameter("saw_ready_hold_sec", 0.40);
    declare_parameter("cut_timeout_sec", 4.0);

    declare_parameter("offboard_prestream_cycles", 20);
    declare_parameter("auto_land_after_cut", true);
    declare_parameter("reset_perception_before_scan", true);

    declare_parameter(
      "gazebo_odom_topic", "/model/x500_lumberjack/base_link_odometry");
    declare_parameter("px4_vehicle_odometry_topic", "/fmu/out/vehicle_odometry");
    declare_parameter<std::vector<std::string>>(
      "px4_vehicle_status_topics",
      {"/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"});
    declare_parameter("px4_offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    declare_parameter("px4_trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    declare_parameter("px4_vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter("branch_model_topic", "/perception/branch_model");
    declare_parameter("arm_status_topic", "/lumberjack_arm/status");
    declare_parameter("cut_progress_topic", "/lumberjack_arm/cut_progress");
    declare_parameter("cut_success_topic", "/lumberjack_arm/cut_success");
    declare_parameter("target_contact_topic", "/lumberjack_arm/target_contact");
  }

  void load_parameters()
  {
    tree_hint_x_ = get_parameter("tree_hint_x").as_double();
    tree_hint_y_ = get_parameter("tree_hint_y").as_double();
    takeoff_height_ = get_parameter("takeoff_height").as_double();
    observe_distance_ = get_parameter("observe_distance").as_double();
    observe_bearing_offset_deg_ = get_parameter("observe_bearing_offset_deg").as_double();
    arc_half_angle_deg_ = get_parameter("arc_half_angle_deg").as_double();
    arc_settle_sec_ = get_parameter("arc_settle_sec").as_double();

    perception_min_points_ = get_parameter("perception_min_points").as_int();
    perception_max_fit_rms_ = get_parameter("perception_max_fit_rms").as_double();
    perception_stable_samples_ = get_parameter("perception_stable_samples").as_int();
    perception_center_spread_max_ =
      get_parameter("perception_center_spread_max").as_double();

    cut_z_offset_ = get_parameter("cut_z_offset").as_double();
    tool_forward_offset_ = get_parameter("tool_forward_offset").as_double();
    near_standoff_ = get_parameter("near_standoff").as_double();
    far_standoff_ = get_parameter("far_standoff").as_double();
    cut_in_penetration_ = get_parameter("cut_in_penetration").as_double();
    cut_in_max_distance_ = get_parameter("cut_in_max_distance").as_double();

    takeoff_speed_ = get_parameter("takeoff_speed").as_double();
    observe_speed_ = get_parameter("observe_speed").as_double();
    far_speed_ = get_parameter("far_speed").as_double();
    near_speed_ = get_parameter("near_speed").as_double();
    cut_in_speed_ = get_parameter("cut_in_speed").as_double();
    retreat_speed_ = get_parameter("retreat_speed").as_double();
    return_speed_ = get_parameter("return_speed").as_double();
    yaw_rate_rad_s_ = deg2rad(get_parameter("yaw_rate_deg_s").as_double());

    position_tolerance_ = get_parameter("position_tolerance").as_double();
    near_position_tolerance_ = get_parameter("near_position_tolerance").as_double();
    yaw_tolerance_rad_ = deg2rad(get_parameter("yaw_tolerance_deg").as_double());

    saw_command_rpm_ = get_parameter("saw_command_rpm").as_double();
    saw_ready_rpm_ = get_parameter("saw_ready_rpm").as_double();
    saw_ready_hold_sec_ = get_parameter("saw_ready_hold_sec").as_double();
    cut_timeout_sec_ = get_parameter("cut_timeout_sec").as_double();

    offboard_prestream_cycles_ = get_parameter("offboard_prestream_cycles").as_int();
    auto_land_after_cut_ = get_parameter("auto_land_after_cut").as_bool();
    reset_perception_before_scan_ =
      get_parameter("reset_perception_before_scan").as_bool();

    gazebo_odom_topic_ = get_parameter("gazebo_odom_topic").as_string();
    px4_vehicle_odometry_topic_ = get_parameter("px4_vehicle_odometry_topic").as_string();
    px4_vehicle_status_topics_ = get_parameter("px4_vehicle_status_topics").as_string_array();
    px4_offboard_control_mode_topic_ =
      get_parameter("px4_offboard_control_mode_topic").as_string();
    px4_trajectory_setpoint_topic_ =
      get_parameter("px4_trajectory_setpoint_topic").as_string();
    px4_vehicle_command_topic_ = get_parameter("px4_vehicle_command_topic").as_string();
    branch_model_topic_ = get_parameter("branch_model_topic").as_string();
    arm_status_topic_ = get_parameter("arm_status_topic").as_string();
    cut_progress_topic_ = get_parameter("cut_progress_topic").as_string();
    cut_success_topic_ = get_parameter("cut_success_topic").as_string();
    target_contact_topic_ = get_parameter("target_contact_topic").as_string();
  }

  // ============================================================
  // ROS interface creation
  // ============================================================

  void create_ros_interfaces()
  {
    offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      px4_offboard_control_mode_topic_, 10);
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      px4_trajectory_setpoint_topic_, 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      px4_vehicle_command_topic_, 10);

    px4_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4_vehicle_odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OffboardCuttingDemo::px4_odom_callback, this, std::placeholders::_1));

    for (const auto & topic : px4_vehicle_status_topics_) {
      auto sub = create_subscription<px4_msgs::msg::VehicleStatus>(
        topic, rclcpp::SensorDataQoS(),
        [this, topic](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          vehicle_status_callback(msg, topic);
        });
      vehicle_status_subs_.push_back(sub);
    }

    gazebo_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      gazebo_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OffboardCuttingDemo::gazebo_odom_callback, this, std::placeholders::_1));

    branch_model_sub_ = create_subscription<BranchModel>(
      branch_model_topic_, 10,
      std::bind(&OffboardCuttingDemo::branch_model_callback, this, std::placeholders::_1));

    arm_status_sub_ = create_subscription<ArmStatus>(
      arm_status_topic_, 10,
      std::bind(&OffboardCuttingDemo::arm_status_callback, this, std::placeholders::_1));

    cut_progress_sub_ = create_subscription<std_msgs::msg::Float64>(
      cut_progress_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        cut_progress_ = clamp(msg->data, 0.0, 1.0);
      });

    cut_success_sub_ = create_subscription<std_msgs::msg::Bool>(
      cut_success_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        cut_success_ = msg->data;
      });

    target_contact_sub_ = create_subscription<std_msgs::msg::Bool>(
      target_contact_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        target_contact_ = msg->data;
      });

    arm_client_ = rclcpp_action::create_client<ArmMotion>(
      this, "/lumberjack_arm/motion");
    saw_client_ = create_client<SetSaw>("/lumberjack_arm/set_saw");
    perception_reset_client_ = create_client<std_srvs::srv::Trigger>(
      "/perception/multiview/reset");
  }

  // ============================================================
  // Sensor callbacks / frame calibration
  // ============================================================

  void px4_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    px4_ned_ = {msg->position[0], msg->position[1], msg->position[2]};
    px4_velocity_ned_ = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    px4_yaw_ = yaw_from_wxyz(msg->q);
    px4_odom_received_ = true;
    try_calibrate_world_to_ned();
  }

  void vehicle_status_callback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg,
    const std::string & source_topic)
  {
    vehicle_nav_state_ = msg->nav_state;
    vehicle_arming_state_ = msg->arming_state;
    vehicle_status_received_ = true;

    if (vehicle_status_source_topic_.empty()) {
      vehicle_status_source_topic_ = source_topic;
      safe_print("[PX4] VehicleStatus source: " + source_topic);
    }
  }

  void gazebo_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    world_position_ = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};

    const auto & q = msg->pose.pose.orientation;
    world_yaw_ = yaw_from_xyzw(q.x, q.y, q.z, q.w);

    world_velocity_ = {
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z};

    if (!world_odom_received_) {
      home_world_ = world_position_;
      home_world_yaw_ = world_yaw_;
      home_world_valid_ = true;
    }

    world_odom_received_ = true;
    try_calibrate_world_to_ned();
  }

  void try_calibrate_world_to_ned()
  {
    if (frame_map_valid_ || !world_odom_received_ || !px4_odom_received_) {
      return;
    }

    // Gazebo world uses ENU semantics:
    //   world X = East, world Y = North, world Z = Up.
    //
    // PX4 local position uses NED semantics:
    //   PX4 X = North, PX4 Y = East, PX4 Z = Down.
    //
    // Therefore ENU -> NED is an axis swap + Z inversion, NOT a planar
    // rotation. Translation is calibrated once from the simultaneous
    // PX4/Gazebo pose sample.
    //
    // Yaw conventions are also opposite:
    //   world yaw : 0 = East, positive CCW
    //   PX4 yaw   : 0 = North, positive clockwise
    //
    // Preserve the estimator's current heading bias while reversing the
    // yaw direction.
    ned_yaw_offset_ = wrap_pi(px4_yaw_ + world_yaw_);

    ned_offset_x_ = px4_ned_.x - world_position_.y;  // North <- world Y
    ned_offset_y_ = px4_ned_.y - world_position_.x;  // East  <- world X
    ned_offset_z_ = px4_ned_.z + world_position_.z;  // Down  <- -world Z
    frame_map_valid_ = true;

    std::ostringstream oss;
    oss << "[FRAME] world -> PX4 NED calibrated\n"
        << "        offset xyz = ["
        << std::fixed << std::setprecision(3)
        << ned_offset_x_ << ", " << ned_offset_y_ << ", " << ned_offset_z_ << "]\n"
        << "        yaw bias   = " << std::setprecision(2)
        << rad2deg(ned_yaw_offset_) << " deg";
    safe_print(oss.str());
  }

  void branch_model_callback(const BranchModel::SharedPtr msg)
  {
    branch_model_received_ = true;
    live_branch_valid_ = msg->valid && msg->header.frame_id == "world";

    if (!live_branch_valid_) {
      return;
    }

    live_branch_.center = {msg->center.x, msg->center.y, msg->center.z};
    live_branch_.direction = {msg->direction.x, msg->direction.y, msg->direction.z};
    live_branch_.length = msg->length;
    live_branch_.radius = msg->radius;
    live_branch_.fit_rms = msg->fit_rms;
    live_branch_.point_count = msg->point_count;

    if (phase_ == Phase::OBSERVE ||
        phase_ == Phase::ARC_SCAN ||
        phase_ == Phase::TARGET_LOCK)
    {
      if (branch_quality_ok(live_branch_)) {
        branch_samples_.push_back(live_branch_);
        while (branch_samples_.size() > 30) {
          branch_samples_.pop_front();
        }
      }
    }
  }

  void arm_status_callback(const ArmStatus::SharedPtr msg)
  {
    arm_status_received_ = true;
    arm_busy_ = msg->busy;
    arm_task_ = msg->task;
    arm_joint_actual_ = msg->joint_actual_deg;
    saw_command_actual_from_arm_ = msg->saw_command_rpm;
    saw_actual_rpm_ = msg->saw_actual_rpm;
  }

  // ============================================================
  // CLI
  // ============================================================

  void start_cli_thread()
  {
    cli_thread_ = std::thread([this]() {
      std::string line;
      while (rclcpp::ok() && !stop_cli_.load()) {
        if (!std::getline(std::cin, line)) {
          break;
        }
        if (line.empty()) {
          continue;
        }
        {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_queue_.push_back(line);
        }
      }
    });
  }

  void process_command_queue()
  {
    std::deque<std::string> local;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      local.swap(command_queue_);
    }

    for (const auto & line : local) {
      handle_command(line);
    }
  }

  void handle_command(const std::string & line)
  {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    std::transform(cmd.begin(), cmd.end(), cmd.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    if (cmd == "help" || cmd == "?") {
      print_help();
      return;
    }
    if (cmd == "status") {
      print_status();
      return;
    }
    if (cmd == "params") {
      print_params();
      return;
    }
    if (cmd == "sethome") {
      if (!world_odom_received_) {
        safe_print("[REJECT] World odometry unavailable.");
      } else {
        home_world_ = world_position_;
        home_world_yaw_ = world_yaw_;
        home_world_valid_ = true;
        safe_print("[HOME] Sampled current world pose as mission home: " + fmt_vec(home_world_));
      }
      return;
    }
    if (cmd == "start") {
      start_full_demo();
      return;
    }
    if (cmd == "pause") {
      pause_mission("operator request");
      return;
    }
    if (cmd == "resume") {
      resume_mission();
      return;
    }
    if (cmd == "retry") {
      retry_current_phase();
      return;
    }
    if (cmd == "reset") {
      reset_task(false);
      return;
    }
    if (cmd == "reset_target" || cmd == "reobserve") {
      reset_target_and_perception();
      return;
    }
    if (cmd == "abort") {
      start_abort_retreat();
      return;
    }
    if (cmd == "takeoff") {
      start_manual_phase(Phase::ARM_HOME_PREP, Phase::TAKEOFF);
      return;
    }
    if (cmd == "observe") {
      start_manual_flight_phase(Phase::OBSERVE, Phase::OBSERVE);
      return;
    }
    if (cmd == "scan") {
      start_manual_flight_phase(Phase::ARC_SCAN, Phase::ARC_SCAN);
      return;
    }
    if (cmd == "lock") {
      start_manual_phase(Phase::TARGET_LOCK, Phase::TARGET_LOCK);
      return;
    }
    if (cmd == "far") {
      if (!locked_target_.valid) {
        safe_print("[REJECT] No locked target. Run scan -> lock first.");
        return;
      }
      start_manual_flight_phase(Phase::FAR_APPROACH, Phase::FAR_APPROACH);
      return;
    }
    if (cmd == "prework") {
      start_manual_phase(Phase::ARM_PREWORK, Phase::ARM_PREWORK);
      return;
    }
    if (cmd == "near") {
      if (!locked_target_.valid) {
        safe_print("[REJECT] No locked target.");
        return;
      }
      start_manual_flight_phase(Phase::NEAR_APPROACH, Phase::NEAR_APPROACH);
      return;
    }
    if (cmd == "cut") {
      if (!locked_target_.valid) {
        safe_print("[REJECT] No locked target.");
        return;
      }
      // Manual CUT is still a safety chain: spin -> cut-in -> wait -> retreat to NEAR.
      start_manual_flight_phase(Phase::SAW_SPINUP, Phase::RETREAT_NEAR);
      return;
    }
    if (cmd == "retreat") {
      if (!locked_target_.valid) {
        safe_print("[REJECT] No locked target; use pause/QGC or armhome instead.");
        return;
      }
      manual_retreat_to_home_arm_ = true;
      start_manual_flight_phase(Phase::RETREAT_NEAR, Phase::ARM_HOME_POST);
      return;
    }
    if (cmd == "armhome" || cmd == "home") {
      start_manual_phase(Phase::ARM_HOME_POST, Phase::ARM_HOME_POST);
      return;
    }
    if (cmd == "land") {
      manual_land_request_ = true;
      start_manual_phase(Phase::ARM_HOME_POST, Phase::LANDING);
      return;
    }
    if (cmd == "quit" || cmd == "exit") {
      request_saw(0.0);
      safe_print("[EXIT] Offboard cutting demo shutting down. PX4 mode is not forcibly changed.");
      rclcpp::shutdown();
      return;
    }

    safe_print("[UNKNOWN] '" + cmd + "'  (type 'help')");
  }

  // ============================================================
  // High-level command helpers
  // ============================================================

  bool base_readiness_ok(bool require_arm = true) const
  {
    return world_odom_received_ && px4_odom_received_ && vehicle_status_received_ &&
           frame_map_valid_ && (!require_arm || arm_status_received_);
  }

  void start_full_demo()
  {
    if (!base_readiness_ok(true)) {
      safe_print("[REJECT] System not ready. Use 'status' and check PX4/Gazebo/arm data.");
      return;
    }
    if (!arm_client_->action_server_is_ready()) {
      safe_print("[REJECT] /lumberjack_arm/motion action server is not ready.");
      return;
    }
    if (vehicle_arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      safe_print(
        "[REJECT] Full 'start' is a ground-start demo and UAV is already armed. "
        "Use step commands / resume, or land first.");
      return;
    }
    if (mission_running()) {
      safe_print("[REJECT] Mission already active. Use status/pause/abort.");
      return;
    }

    reset_task_state_only();
    auto_chain_ = true;
    manual_stop_after_.reset();
    transition_to(Phase::ARM_HOME_PREP, "FULL AUTO START");
  }

  void start_manual_phase(Phase phase, Phase stop_after)
  {
    if (mission_running()) {
      safe_print("[REJECT] An automatic/step phase is already running. Use pause or wait for HOLD.");
      return;
    }
    if (!base_readiness_ok(false)) {
      safe_print("[REJECT] Required pose/PX4 data are not ready. Use 'status'.");
      return;
    }
    auto_chain_ = false;
    manual_stop_after_ = stop_after;
    manual_retreat_to_home_arm_ = false;
    manual_land_request_ = false;
    transition_to(phase, "MANUAL STEP");
  }

  void start_manual_flight_phase(Phase phase, Phase stop_after)
  {
    if (mission_running()) {
      safe_print("[REJECT] An automatic/step phase is already running. Use pause or wait for HOLD.");
      return;
    }
    if (!base_readiness_ok(false)) {
      safe_print("[REJECT] Required pose/PX4 data are not ready. Use 'status'.");
      return;
    }
    if (phase != Phase::TAKEOFF && vehicle_status_received_ &&
        vehicle_arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      safe_print("[REJECT] UAV is disarmed. Run 'takeoff' first for this flight phase.");
      return;
    }
    auto_chain_ = false;
    manual_stop_after_ = stop_after;
    manual_retreat_to_home_arm_ = false;
    manual_land_request_ = false;

    if (is_offboard_active()) {
      offboard_streaming_ = true;
      offboard_expected_ = true;
      transition_to(phase, "MANUAL FLIGHT STEP");
    } else {
      begin_offboard_prestream(phase, false);
    }
  }

  void pause_mission(const std::string & reason)
  {
    if (phase_ == Phase::PAUSED) {
      safe_print("[PAUSE] Already paused.");
      return;
    }

    paused_phase_ = phase_;
    paused_phase_valid_ = true;
    auto_chain_before_pause_ = auto_chain_;
    manual_stop_before_pause_ = manual_stop_after_;

    request_saw(0.0);
    cancel_arm_goal_if_active();

    // Hand PX4 back to Position mode. The operator can now use QGC.
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 3.0f);
    offboard_expected_ = false;
    offboard_streaming_ = false;
    transition_to(Phase::PAUSED, "PAUSE: " + reason);

    safe_print(
      "[PAUSE] QGC/manual adjustment is allowed now.\n"
      "        'resume' will re-read the current UAV pose and recompute this stage.");
  }

  void resume_mission()
  {
    if (phase_ != Phase::PAUSED || !paused_phase_valid_) {
      safe_print("[REJECT] Mission is not paused.");
      return;
    }
    if (!base_readiness_ok(false)) {
      safe_print("[REJECT] Current UAV state unavailable; cannot resume.");
      return;
    }

    Phase resume_phase = paused_phase_;
    if (resume_phase == Phase::CUT_IN ||
        resume_phase == Phase::CUT_WAIT ||
        resume_phase == Phase::SAW_SPINUP)
    {
      // Never resume halfway through a blade insertion. Re-establish NEAR first.
      resume_phase = Phase::NEAR_APPROACH;
    }
    if (resume_phase == Phase::OFFBOARD_PRESTREAM) {
      resume_phase = pending_after_offboard_;
    }

    auto_chain_ = auto_chain_before_pause_;
    manual_stop_after_ = manual_stop_before_pause_;
    request_saw(0.0);
    begin_offboard_prestream(resume_phase, false);
    safe_print("[RESUME] Re-entering OFFBOARD from the current UAV pose.");
  }

  void retry_current_phase()
  {
    Phase retry = last_task_phase_;
    if (phase_ == Phase::CUT_FAILED) {
      // Re-establish NEAR safely; operator can inspect and issue 'cut' again.
      retry = Phase::NEAR_APPROACH;
    } else if (phase_ != Phase::HOLD && phase_ != Phase::PAUSED) {
      retry = phase_;
    }

    if (retry == Phase::IDLE || retry == Phase::DONE || retry == Phase::LANDING) {
      safe_print("[REJECT] No useful phase to retry.");
      return;
    }

    auto_chain_ = false;
    manual_stop_after_ = retry;
    if (is_flight_phase(retry) && !is_offboard_active()) {
      begin_offboard_prestream(retry, false);
    } else {
      transition_to(retry, "RETRY");
    }
  }

  void reset_task(bool clear_perception)
  {
    request_saw(0.0);
    cancel_arm_goal_if_active();
    reset_task_state_only();
    if (clear_perception) {
      request_perception_reset();
    }

    if (is_offboard_active()) {
      offboard_streaming_ = true;
      offboard_expected_ = true;
      set_hold_at_current();
      phase_ = Phase::HOLD;
    } else {
      offboard_streaming_ = false;
      offboard_expected_ = false;
      phase_ = Phase::IDLE;
    }
    safe_print(
      "[RESET] Task logic reset. UAV is NOT forced to take off/land or move the arm.\n"
      "        Saw OFF; locked target cleared. Use stage commands to continue.");
  }

  void reset_target_and_perception()
  {
    request_saw(0.0);
    locked_target_ = LockedTarget{};
    branch_samples_.clear();
    geometry_valid_ = false;
    request_perception_reset();
    safe_print("[TARGET] Locked target cleared; perception fusion reset requested.");
  }

  void start_abort_retreat()
  {
    request_saw(0.0);
    cancel_arm_goal_if_active();
    auto_chain_ = false;
    manual_retreat_to_home_arm_ = true;
    manual_land_request_ = false;
    manual_stop_after_ = Phase::ARM_HOME_POST;

    if (locked_target_.valid && geometry_valid_) {
      if (!is_offboard_active()) {
        begin_offboard_prestream(Phase::RETREAT_NEAR, false);
      } else {
        transition_to(Phase::RETREAT_NEAR, "CONTROLLED ABORT");
      }
    } else {
      transition_to(Phase::ARM_HOME_POST, "ABORT: no cut geometry");
    }
  }

  // ============================================================
  // Control loop
  // ============================================================

  void control_loop()
  {
    process_command_queue();

    if (frame_map_valid_ && offboard_streaming_) {
      update_commanded_setpoint(kControlDt);
      publish_offboard_control_mode();
      publish_trajectory_setpoint();
    }

    monitor_frame_map_consistency();
    monitor_unexpected_manual_takeover();
    update_phase();
  }

  void monitor_frame_map_consistency()
  {
    if (!frame_map_valid_ || !world_odom_received_ || !px4_odom_received_) {
      frame_map_error_m_ = 0.0;
      frame_map_bad_counter_ = 0;
      return;
    }

    const Vec3 predicted = world_to_ned(world_position_);
    frame_map_error_m_ = norm3d(predicted - px4_ned_);

    if (frame_map_error_m_ > 0.50) {
      ++frame_map_bad_counter_;
      if (frame_map_bad_counter_ > 25 && offboard_expected_ && phase_ != Phase::PAUSED) {
        frame_map_bad_counter_ = 0;
        pause_mission(
          "world->NED consistency error " + format_number(frame_map_error_m_, 3) + " m");
      }
    } else {
      frame_map_bad_counter_ = 0;
    }
  }

  void monitor_unexpected_manual_takeover()
  {
    if (!vehicle_status_received_ || !offboard_expected_ ||
        phase_ == Phase::OFFBOARD_PRESTREAM || phase_ == Phase::LANDING ||
        phase_ == Phase::PAUSED)
    {
      offboard_loss_counter_ = 0;
      return;
    }

    if (vehicle_nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      ++offboard_loss_counter_;
      if (offboard_loss_counter_ > 25) {
        offboard_loss_counter_ = 0;
        pause_mission("PX4 left OFFBOARD / manual takeover detected");
      }
    } else {
      offboard_loss_counter_ = 0;
      offboard_confirmed_once_ = true;
    }
  }

  void update_phase()
  {
    if (phase_ == Phase::IDLE || phase_ == Phase::HOLD || phase_ == Phase::PAUSED ||
        phase_ == Phase::DONE || phase_ == Phase::CUT_FAILED)
    {
      return;
    }

    if (!phase_initialized_) {
      phase_initialized_ = true;
      phase_start_time_ = now();
      enter_phase();
    }

    switch (phase_) {
      case Phase::ARM_HOME_PREP:
      case Phase::ARM_PREWORK:
      case Phase::ARM_HOME_POST:
        update_arm_phase();
        break;

      case Phase::OFFBOARD_PRESTREAM:
        update_offboard_prestream();
        break;

      case Phase::TAKEOFF:
      case Phase::OBSERVE:
      case Phase::FAR_APPROACH:
      case Phase::NEAR_APPROACH:
      case Phase::RETREAT_NEAR:
      case Phase::RETREAT_FAR:
      case Phase::RETURN_HOME:
        update_motion_phase();
        break;

      case Phase::ARC_SCAN:
        update_arc_scan();
        break;

      case Phase::TARGET_LOCK:
        update_target_lock();
        break;

      case Phase::SAW_SPINUP:
        update_saw_spinup();
        break;

      case Phase::CUT_IN:
        update_cut_in();
        break;

      case Phase::CUT_WAIT:
        update_cut_wait();
        break;

      case Phase::LANDING:
        update_landing();
        break;

      default:
        break;
    }
  }

  void enter_phase()
  {
    last_task_phase_ = phase_;

    switch (phase_) {
      case Phase::ARM_HOME_PREP:
        request_saw(0.0);
        send_arm_goal(kArmModeHome, "HOME (takeoff preparation)");
        break;

      case Phase::OFFBOARD_PRESTREAM:
        set_hold_at_current();
        prestream_counter_ = 0;
        offboard_mode_command_sent_ = false;
        offboard_streaming_ = true;
        offboard_expected_ = false;
        break;

      case Phase::TAKEOFF: {
        const Vec3 target{home_world_.x, home_world_.y, home_world_.z + takeoff_height_};
        set_motion_target(target, home_world_yaw_, takeoff_speed_, position_tolerance_);
        break;
      }

      case Phase::OBSERVE: {
        compute_observe_geometry();
        const double yaw = yaw_toward(observe_center_, {tree_hint_x_, tree_hint_y_, observe_center_.z});
        set_motion_target(observe_center_, yaw, observe_speed_, position_tolerance_);
        break;
      }

      case Phase::ARC_SCAN:
        start_arc_scan();
        break;

      case Phase::TARGET_LOCK:
        target_lock_last_attempt_ = now();
        break;

      case Phase::FAR_APPROACH:
        if (!compute_cut_geometry(true)) {
          fail_to_hold("Cannot compute cut geometry from locked target.");
          return;
        }
        set_motion_target(far_approach_, insertion_yaw_, far_speed_, position_tolerance_);
        break;

      case Phase::ARM_PREWORK:
        request_saw(0.0);
        send_arm_goal(kArmModePrework, "PREWORK = CUT pose");
        break;

      case Phase::NEAR_APPROACH:
        if (!geometry_valid_ && !compute_cut_geometry(true)) {
          fail_to_hold("Cut geometry unavailable.");
          return;
        }
        set_motion_target(
          near_approach_, insertion_yaw_, near_speed_, near_position_tolerance_);
        break;

      case Phase::SAW_SPINUP:
        saw_ready_since_.reset();
        saw_request_accepted_ = false;
        saw_request_failed_ = false;
        request_saw(saw_command_rpm_);
        break;

      case Phase::CUT_IN: {
        cut_in_start_world_ = world_position_;
        const double requested = near_standoff_ + cut_in_penetration_;
        const double travel = std::min(requested, cut_in_max_distance_);
        cut_in_target_ = near_approach_ + insertion_direction_ * travel;
        cut_in_target_.z = contact_base_.z;
        set_motion_target(
          cut_in_target_, insertion_yaw_, cut_in_speed_, near_position_tolerance_);
        break;
      }

      case Phase::CUT_WAIT:
        set_hold_at_current();
        cut_wait_start_ = now();
        break;

      case Phase::RETREAT_NEAR:
        request_saw(0.0);
        set_motion_target(
          near_approach_, insertion_yaw_, retreat_speed_, near_position_tolerance_);
        break;

      case Phase::RETREAT_FAR:
        set_motion_target(far_approach_, insertion_yaw_, retreat_speed_, position_tolerance_);
        break;

      case Phase::ARM_HOME_POST:
        request_saw(0.0);
        send_arm_goal(kArmModeHome, "HOME (retreat/landing)");
        break;

      case Phase::RETURN_HOME: {
        Vec3 target = home_world_;
        target.z = home_world_.z + takeoff_height_;
        set_motion_target(target, home_world_yaw_, return_speed_, position_tolerance_);
        break;
      }

      case Phase::LANDING:
        request_saw(0.0);
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0f, 0.0f);
        offboard_expected_ = false;
        landing_command_sent_time_ = now();
        break;

      default:
        break;
    }
  }

  // ============================================================
  // Phase update functions
  // ============================================================

  void update_arm_phase()
  {
    if (arm_action_failed_) {
      arm_action_failed_ = false;
      fail_to_hold("Arm action failed: " + arm_action_message_);
      return;
    }
    if (!arm_action_done_) {
      return;
    }

    arm_action_done_ = false;

    if (phase_ == Phase::ARM_HOME_PREP) {
      if (manual_stop_after_ && *manual_stop_after_ == Phase::ARM_HOME_PREP) {
        finish_manual_or_continue(Phase::HOLD);
      } else {
        begin_offboard_prestream(Phase::TAKEOFF, true);
      }
      return;
    }

    if (phase_ == Phase::ARM_PREWORK) {
      finish_manual_or_continue(Phase::NEAR_APPROACH);
      return;
    }

    if (phase_ == Phase::ARM_HOME_POST) {
      if (manual_land_request_) {
        transition_to(Phase::RETURN_HOME, "arm HOME; return for landing");
      } else if (auto_chain_ && auto_land_after_cut_) {
        transition_to(Phase::RETURN_HOME, "arm HOME; auto return");
      } else if (auto_chain_) {
        transition_to(Phase::DONE, "demo complete; auto-land disabled");
      } else {
        finish_manual_or_continue(Phase::HOLD);
      }
    }
  }

  void update_offboard_prestream()
  {
    ++prestream_counter_;

    if (!offboard_mode_command_sent_ && prestream_counter_ >= offboard_prestream_cycles_) {
      publish_vehicle_command(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      if (arm_on_offboard_entry_) {
        publish_vehicle_command(
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
      }
      offboard_mode_command_sent_ = true;
      offboard_command_time_ = now();
      safe_print("[PX4] OFFBOARD requested" +
        std::string(arm_on_offboard_entry_ ? " + ARM" : ""));
    }

    if (!offboard_mode_command_sent_) {
      return;
    }

    const bool status_confirms = vehicle_status_received_ && is_offboard_active();
    if (status_confirms) {
      offboard_expected_ = true;
      offboard_confirmed_once_ = true;
      transition_to(pending_after_offboard_, "OFFBOARD confirmed by VehicleStatus");
      return;
    }

    if ((now() - offboard_command_time_).seconds() > 3.0) {
      offboard_streaming_ = false;
      offboard_expected_ = false;
      fail_to_hold("PX4 did not enter OFFBOARD within 3 s. Check QGC/XRCE/PX4 state.");
    }
  }

  void update_motion_phase()
  {
    if (!target_reached()) {
      return;
    }

    switch (phase_) {
      case Phase::TAKEOFF:
        finish_manual_or_continue(Phase::OBSERVE);
        break;
      case Phase::OBSERVE:
        finish_manual_or_continue(Phase::ARC_SCAN);
        break;
      case Phase::FAR_APPROACH:
        finish_manual_or_continue(Phase::ARM_PREWORK);
        break;
      case Phase::NEAR_APPROACH:
        finish_manual_or_continue(Phase::SAW_SPINUP);
        break;
      case Phase::RETREAT_NEAR:
        if (cut_failure_pending_pause_) {
          cut_failure_pending_pause_ = false;
          transition_to(Phase::CUT_FAILED, "safe back-out complete; operator inspection required");
          set_hold_at_current();
        } else if (!auto_chain_ && manual_stop_after_ && *manual_stop_after_ == Phase::RETREAT_NEAR) {
          finish_manual_or_continue(Phase::HOLD);
        } else {
          transition_to(Phase::RETREAT_FAR, "clear of cutting zone");
        }
        break;
      case Phase::RETREAT_FAR:
        transition_to(Phase::ARM_HOME_POST, "safe FAR retreat reached");
        break;
      case Phase::RETURN_HOME:
        transition_to(Phase::LANDING, "home overhead reached");
        break;
      default:
        break;
    }
  }

  void update_arc_scan()
  {
    if (arc_points_.empty()) {
      fail_to_hold("ARC_SCAN has no waypoints.");
      return;
    }

    if (!target_reached()) {
      arc_arrival_since_.reset();
      return;
    }

    if (!arc_arrival_since_) {
      arc_arrival_since_ = now();
      return;
    }

    if ((now() - *arc_arrival_since_).seconds() < arc_settle_sec_) {
      return;
    }

    ++arc_index_;
    arc_arrival_since_.reset();

    if (arc_index_ >= arc_points_.size()) {
      finish_manual_or_continue(Phase::TARGET_LOCK);
      return;
    }

    const Vec3 & p = arc_points_[arc_index_];
    Vec3 look = live_branch_valid_ ? live_branch_.center :
      Vec3{tree_hint_x_, tree_hint_y_, p.z};
    const double yaw = yaw_toward(p, look);
    set_motion_target(p, yaw, observe_speed_, position_tolerance_);

    std::ostringstream oss;
    oss << "[SCAN] view " << (arc_index_ + 1) << "/" << arc_points_.size()
        << " target=" << fmt_vec(p);
    safe_print(oss.str());
  }

  void update_target_lock()
  {
    if ((now() - target_lock_last_attempt_).seconds() < 0.4) {
      return;
    }
    target_lock_last_attempt_ = now();

    std::string reason;
    if (try_lock_target(reason)) {
      safe_print(
        "[TARGET LOCK]\n"
        "  center(world) : " + fmt_vec(locked_target_.center) + "\n"
        "  direction     : " + fmt_vec(locked_target_.direction) + "\n"
        "  length/radius : " + format_number(locked_target_.length, 3) + " / " +
          format_number(locked_target_.radius, 3) + " m");
      finish_manual_or_continue(Phase::FAR_APPROACH);
      return;
    }

    if ((now() - phase_start_time_).seconds() > 15.0) {
      fail_to_hold(
        "TARGET_LOCK timeout: " + reason +
        ". Use scan/reobserve or QGC adjustment, then retry.");
    }
  }

  void update_saw_spinup()
  {
    if (saw_request_failed_) {
      fail_to_hold("Saw service rejected/failed.");
      return;
    }
    if (!saw_request_accepted_) {
      return;
    }

    if (saw_actual_rpm_ >= saw_ready_rpm_) {
      if (!saw_ready_since_) {
        saw_ready_since_ = now();
      }
      if ((now() - *saw_ready_since_).seconds() >= saw_ready_hold_sec_) {
        transition_to(Phase::CUT_IN, "saw speed stable");
      }
    } else {
      saw_ready_since_.reset();
    }
  }

  void update_cut_in()
  {
    if (cut_success_) {
      request_saw(0.0);
      transition_to(Phase::RETREAT_NEAR, "CUT SUCCESS during insertion");
      return;
    }

    if (target_contact_) {
      transition_to(Phase::CUT_WAIT, "target contact detected");
      return;
    }

    const double travelled = norm3d(world_position_ - cut_in_start_world_);
    if (travelled >= cut_in_max_distance_ - 1e-3 || target_reached()) {
      trigger_cut_failure(
        "Cut-in reached distance/target limit without target contact.");
    }
  }

  void update_cut_wait()
  {
    if (cut_success_) {
      request_saw(0.0);
      safe_print("[CUT] SUCCESS (auto_cut_controller detached the branch)");
      transition_to(Phase::RETREAT_NEAR, "detach/cut_success received");
      return;
    }

    if ((now() - cut_wait_start_).seconds() > cut_timeout_sec_) {
      trigger_cut_failure("Cut timeout: no detach/cut_success received.");
    }
  }

  void update_landing()
  {
    if ((now() - landing_command_sent_time_).seconds() > 0.5) {
      offboard_streaming_ = false;
    }

    if (vehicle_status_received_ &&
        vehicle_arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    {
      transition_to(Phase::DONE, "landed + disarmed");
      safe_print("[DONE] Offboard cutting demo complete.");
    }
  }

  // ============================================================
  // Geometry / perception
  // ============================================================

  void compute_observe_geometry()
  {
    const Vec3 anchor = home_world_valid_ ? home_world_ : world_position_;
    const double tree_to_anchor = std::atan2(
      anchor.y - tree_hint_y_, anchor.x - tree_hint_x_);
    observe_center_bearing_ = wrap_pi(
      tree_to_anchor + deg2rad(observe_bearing_offset_deg_));

    observe_center_.x = tree_hint_x_ + observe_distance_ * std::cos(observe_center_bearing_);
    observe_center_.y = tree_hint_y_ + observe_distance_ * std::sin(observe_center_bearing_);
    observe_center_.z = home_world_.z + takeoff_height_;
  }

  void start_arc_scan()
  {
    compute_observe_geometry();
    branch_samples_.clear();
    if (reset_perception_before_scan_) {
      request_perception_reset();
    }

    const double a = deg2rad(arc_half_angle_deg_);
    const std::array<double, 3> bearings = {
      observe_center_bearing_ + a,
      observe_center_bearing_ - a,
      observe_center_bearing_};

    arc_points_.clear();
    for (double bearing : bearings) {
      arc_points_.push_back({
        tree_hint_x_ + observe_distance_ * std::cos(bearing),
        tree_hint_y_ + observe_distance_ * std::sin(bearing),
        home_world_.z + takeoff_height_});
    }

    arc_index_ = 0;
    arc_arrival_since_.reset();
    const Vec3 p = arc_points_.front();
    const double yaw = yaw_toward(p, {tree_hint_x_, tree_hint_y_, p.z});
    set_motion_target(p, yaw, observe_speed_, position_tolerance_);

    safe_print(
      "[SCAN] Small-arc observation started: 3 views, +/-" +
      format_number(arc_half_angle_deg_, 1) + " deg");
  }

  bool branch_quality_ok(const BranchSample & sample) const
  {
    return sample.point_count >= static_cast<std::uint32_t>(perception_min_points_) &&
           sample.fit_rms <= perception_max_fit_rms_ &&
           std::isfinite(sample.center.x) &&
           std::isfinite(sample.center.y) &&
           std::isfinite(sample.center.z);
  }

  bool try_lock_target(std::string & reason)
  {
    const int needed = std::max(2, perception_stable_samples_);
    if (static_cast<int>(branch_samples_.size()) < needed) {
      reason = "need " + std::to_string(needed) + " stable samples; have " +
        std::to_string(branch_samples_.size());
      return false;
    }

    const auto start = branch_samples_.end() - needed;
    Vec3 mean_center{};
    for (auto it = start; it != branch_samples_.end(); ++it) {
      mean_center = mean_center + it->center;
    }
    mean_center = mean_center * (1.0 / static_cast<double>(needed));

    double max_spread = 0.0;
    for (auto it = start; it != branch_samples_.end(); ++it) {
      max_spread = std::max(max_spread, norm3d(it->center - mean_center));
    }
    if (max_spread > perception_center_spread_max_) {
      reason = "center spread " + format_number(max_spread, 3) + " m > limit";
      return false;
    }

    Vec3 reference_dir = start->direction;
    const double ref_norm = norm3d(reference_dir);
    if (ref_norm < 1e-6) {
      reason = "invalid direction vector";
      return false;
    }
    reference_dir = reference_dir * (1.0 / ref_norm);

    Vec3 mean_dir{};
    double mean_length = 0.0;
    double mean_radius = 0.0;
    for (auto it = start; it != branch_samples_.end(); ++it) {
      Vec3 d = it->direction;
      const double dn = norm3d(d);
      if (dn < 1e-6) continue;
      d = d * (1.0 / dn);
      const double dot = d.x * reference_dir.x + d.y * reference_dir.y + d.z * reference_dir.z;
      if (dot < 0.0) d = d * -1.0;  // PCA axis sign is arbitrary.
      mean_dir = mean_dir + d;
      mean_length += it->length;
      mean_radius += it->radius;
    }

    const double dir_norm = norm3d(mean_dir);
    if (dir_norm < 1e-6) {
      reason = "direction averaging failed";
      return false;
    }

    locked_target_.valid = true;
    locked_target_.center = mean_center;
    locked_target_.direction = mean_dir * (1.0 / dir_norm);
    locked_target_.length = mean_length / static_cast<double>(needed);
    locked_target_.radius = mean_radius / static_cast<double>(needed);
    locked_target_.stamp = now();
    geometry_valid_ = false;
    return true;
  }

  bool compute_cut_geometry(bool use_current_side)
  {
    if (!locked_target_.valid) {
      return false;
    }

    Vec3 from = use_current_side ? world_position_ : observe_center_;
    double dx = locked_target_.center.x - from.x;
    double dy = locked_target_.center.y - from.y;
    double n = norm2d(dx, dy);

    if (n < 0.15) {
      dx = locked_target_.center.x - observe_center_.x;
      dy = locked_target_.center.y - observe_center_.y;
      n = norm2d(dx, dy);
    }
    if (n < 1e-6) {
      return false;
    }

    insertion_direction_ = {dx / n, dy / n, 0.0};
    insertion_yaw_ = std::atan2(insertion_direction_.y, insertion_direction_.x);

    contact_base_ = {
      locked_target_.center.x - insertion_direction_.x * tool_forward_offset_,
      locked_target_.center.y - insertion_direction_.y * tool_forward_offset_,
      locked_target_.center.z + cut_z_offset_};

    near_approach_ = contact_base_ - insertion_direction_ * near_standoff_;
    far_approach_ = contact_base_ - insertion_direction_ * far_standoff_;
    geometry_valid_ = true;

    std::ostringstream oss;
    oss << "[CUT GEOMETRY]\n"
        << "  insertion yaw : " << std::fixed << std::setprecision(1)
        << rad2deg(insertion_yaw_) << " deg\n"
        << "  FAR            : " << fmt_vec(far_approach_) << "\n"
        << "  NEAR           : " << fmt_vec(near_approach_) << "\n"
        << "  contact base   : " << fmt_vec(contact_base_) << "\n"
        << "  cut Z offset   : +" << std::setprecision(3) << cut_z_offset_ << " m";
    safe_print(oss.str());
    return true;
  }

  double yaw_toward(const Vec3 & from, const Vec3 & to) const
  {
    return std::atan2(to.y - from.y, to.x - from.x);
  }

  // ============================================================
  // Motion target / PX4 output
  // ============================================================

  void begin_offboard_prestream(Phase after, bool arm)
  {
    pending_after_offboard_ = after;
    arm_on_offboard_entry_ = arm ||
      (vehicle_status_received_ &&
       vehicle_arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    transition_to(Phase::OFFBOARD_PRESTREAM, "prepare OFFBOARD");
  }

  void set_hold_at_current()
  {
    commanded_sp_world_ = world_position_;
    commanded_sp_yaw_ = world_yaw_;
    motion_target_world_ = world_position_;
    motion_target_yaw_ = world_yaw_;
    motion_speed_ = 0.0;
    motion_tolerance_ = position_tolerance_;
    motion_target_valid_ = true;
  }

  void set_motion_target(
    const Vec3 & target, double yaw, double speed, double tolerance)
  {
    if (!motion_target_valid_ || !offboard_streaming_) {
      commanded_sp_world_ = world_position_;
      commanded_sp_yaw_ = world_yaw_;
    }
    motion_target_world_ = target;
    motion_target_yaw_ = wrap_pi(yaw);
    motion_speed_ = std::max(0.01, speed);
    motion_tolerance_ = std::max(0.01, tolerance);
    motion_target_valid_ = true;
  }

  void update_commanded_setpoint(double dt)
  {
    if (!motion_target_valid_) {
      set_hold_at_current();
    }

    const Vec3 delta = motion_target_world_ - commanded_sp_world_;
    const double distance = norm3d(delta);
    const double max_step = motion_speed_ > 0.0 ? motion_speed_ * dt : 0.0;

    if (distance > 1e-9 && max_step > 0.0) {
      const double step = std::min(distance, max_step);
      commanded_sp_world_ = commanded_sp_world_ + delta * (step / distance);
    } else if (distance <= 1e-9) {
      commanded_sp_world_ = motion_target_world_;
    }

    const double yaw_error = wrap_pi(motion_target_yaw_ - commanded_sp_yaw_);
    const double yaw_step = clamp(yaw_error, -yaw_rate_rad_s_ * dt, yaw_rate_rad_s_ * dt);
    commanded_sp_yaw_ = wrap_pi(commanded_sp_yaw_ + yaw_step);
  }

  bool target_reached() const
  {
    if (!world_odom_received_ || !motion_target_valid_) {
      return false;
    }
    const double pos_error = norm3d(motion_target_world_ - world_position_);
    const double yaw_error = std::abs(wrap_pi(motion_target_yaw_ - world_yaw_));
    return pos_error <= motion_tolerance_ && yaw_error <= yaw_tolerance_rad_;
  }

  Vec3 world_to_ned(const Vec3 & world) const
  {
    return {
      world.y + ned_offset_x_,
      world.x + ned_offset_y_,
      -world.z + ned_offset_z_};
  }

  double world_yaw_to_ned(double world_yaw) const
  {
    return wrap_pi(ned_yaw_offset_ - world_yaw);
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    offboard_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    if (!frame_map_valid_) return;

    const Vec3 ned = world_to_ned(commanded_sp_world_);
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = now().nanoseconds() / 1000;
    sp.position = {
      static_cast<float>(ned.x),
      static_cast<float>(ned.y),
      static_cast<float>(ned.z)};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sp.velocity = {nan, nan, nan};
    sp.acceleration = {nan, nan, nan};
    sp.yaw = static_cast<float>(world_yaw_to_ned(commanded_sp_yaw_));
    sp.yawspeed = nan;
    trajectory_pub_->publish(sp);
  }

  void publish_vehicle_command(
    std::uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  bool is_offboard_active() const
  {
    return vehicle_status_received_ &&
           vehicle_nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  // ============================================================
  // Arm / saw / perception service helpers
  // ============================================================

  void send_arm_goal(std::uint8_t mode, const std::string & description)
  {
    arm_action_done_ = false;
    arm_action_failed_ = false;
    arm_action_message_.clear();

    if (!arm_client_->action_server_is_ready()) {
      arm_action_failed_ = true;
      arm_action_message_ = "arm action server unavailable";
      return;
    }

    ArmMotion::Goal goal{};
    goal.mode = mode;
    goal.joint_target_deg = {0.0, 0.0, 0.0};
    goal.tcp_target.x = 0.0;
    goal.tcp_target.y = 0.0;
    goal.tcp_target.z = 0.0;

    rclcpp_action::Client<ArmMotion>::SendGoalOptions options;
    options.goal_response_callback =
      [this, description](const ArmGoalHandle::SharedPtr & handle) {
        if (!handle) {
          arm_action_failed_ = true;
          arm_action_message_ = description + " rejected";
          return;
        }
        arm_goal_handle_ = handle;
        safe_print("[ARM] Goal accepted: " + description);
      };

    options.feedback_callback =
      [this](ArmGoalHandle::SharedPtr,
             const std::shared_ptr<const ArmMotion::Feedback> feedback) {
        arm_feedback_state_ = feedback->state;
        arm_feedback_progress_ = feedback->progress;
      };

    options.result_callback =
      [this, description](const ArmGoalHandle::WrappedResult & result) {
        arm_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->success)
        {
          arm_action_message_ = result.result->message;
          arm_action_done_ = true;
          safe_print("[ARM] Completed: " + description);
        } else {
          arm_action_failed_ = true;
          arm_action_message_ = result.result ? result.result->message : "no result";
        }
      };

    arm_client_->async_send_goal(goal, options);
  }

  void cancel_arm_goal_if_active()
  {
    if (arm_goal_handle_) {
      arm_client_->async_cancel_goal(arm_goal_handle_);
      arm_goal_handle_.reset();
      safe_print("[ARM] Active action cancel requested.");
    }
  }

  void request_saw(double rpm)
  {
    if (!saw_client_->service_is_ready()) {
      if (rpm > 1.0) {
        saw_request_failed_ = true;
        safe_print("[SAW] Service unavailable; ON request failed.");
      }
      return;
    }

    auto request = std::make_shared<SetSaw::Request>();
    request->rpm = rpm;
    saw_request_accepted_ = false;
    if (rpm > 1.0) saw_request_failed_ = false;

    saw_client_->async_send_request(
      request,
      [this, rpm](rclcpp::Client<SetSaw>::SharedFuture future) {
        const auto response = future.get();
        if (response->accepted) {
          if (rpm > 1.0) saw_request_accepted_ = true;
          safe_print(
            "[SAW] " + format_number(response->commanded_rpm, 0) +
            " rpm accepted");
        } else {
          if (rpm > 1.0) saw_request_failed_ = true;
          safe_print("[SAW] Request rejected: " + response->message);
        }
      });
  }

  void request_perception_reset()
  {
    if (!perception_reset_client_->service_is_ready()) {
      safe_print("[PERCEPTION] reset service not ready; continuing without reset.");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    perception_reset_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        safe_print(
          std::string("[PERCEPTION] fusion reset: ") +
          (response->success ? "OK - " : "FAILED - ") + response->message);
      });
  }

  // ============================================================
  // State transitions / failure handling
  // ============================================================

  void transition_to(Phase next, const std::string & reason = "")
  {
    const Phase old = phase_;
    phase_ = next;
    phase_initialized_ = false;
    phase_start_time_ = now();

    std::ostringstream oss;
    oss << "\n========== OFFBOARD CUTTING / " << phase_name(next) << " ==========";
    if (!reason.empty()) {
      oss << "\n" << "reason: " << reason;
    }
    if (old != next) {
      oss << "\n" << "from  : " << phase_name(old);
    }
    safe_print(oss.str());
  }

  void finish_manual_or_continue(Phase next)
  {
    const Phase completed = phase_;
    if (!auto_chain_ && manual_stop_after_ && *manual_stop_after_ == completed) {
      last_task_phase_ = completed;
      transition_to(Phase::HOLD, "manual step complete: " + phase_name(completed));
      set_hold_at_current();
      return;
    }
    transition_to(next, "phase complete");
  }

  void fail_to_hold(const std::string & message)
  {
    request_saw(0.0);
    safe_print("[FAILED] " + message);
    auto_chain_ = false;
    manual_stop_after_.reset();
    transition_to(Phase::HOLD, "failure -> operator inspection");
    if (offboard_streaming_) {
      set_hold_at_current();
    }
  }

  void trigger_cut_failure(const std::string & message)
  {
    request_saw(0.0);
    safe_print("[CUT FAILED] " + message);

    // First back out to NEAR automatically; after that pause for inspection.
    auto_chain_ = false;
    manual_stop_after_.reset();
    cut_failure_pending_pause_ = true;
    transition_to(Phase::RETREAT_NEAR, "cut failure -> safe back-out");
  }

  void reset_task_state_only()
  {
    locked_target_ = LockedTarget{};
    branch_samples_.clear();
    geometry_valid_ = false;
    cut_success_ = false;
    cut_progress_ = 0.0;
    target_contact_ = false;
    cut_failure_pending_pause_ = false;
    arm_action_done_ = false;
    arm_action_failed_ = false;
    saw_request_accepted_ = false;
    saw_request_failed_ = false;
    manual_retreat_to_home_arm_ = false;
    manual_land_request_ = false;
    paused_phase_valid_ = false;
    last_task_phase_ = Phase::IDLE;
  }

  bool mission_running() const
  {
    return phase_ != Phase::IDLE && phase_ != Phase::HOLD &&
           phase_ != Phase::PAUSED && phase_ != Phase::DONE &&
           phase_ != Phase::CUT_FAILED;
  }

  bool is_flight_phase(Phase p) const
  {
    switch (p) {
      case Phase::TAKEOFF:
      case Phase::OBSERVE:
      case Phase::ARC_SCAN:
      case Phase::FAR_APPROACH:
      case Phase::NEAR_APPROACH:
      case Phase::SAW_SPINUP:
      case Phase::CUT_IN:
      case Phase::CUT_WAIT:
      case Phase::RETREAT_NEAR:
      case Phase::RETREAT_FAR:
      case Phase::RETURN_HOME:
        return true;
      default:
        return false;
    }
  }

  // ============================================================
  // Terminal UI
  // ============================================================

  void safe_print(const std::string & text) const
  {
    std::lock_guard<std::mutex> lock(print_mutex_);
    std::cout << text << std::endl;
    std::cout << "[CUT-DEMO:" << phase_name(phase_) << "] > " << std::flush;
  }

  void print_startup() const
  {
    std::lock_guard<std::mutex> lock(print_mutex_);
    std::cout
      << "\n+----------------------------------------------------------------+\n"
      << "| UAV_lumberjack | Offboard Cutting Demo                         |\n"
      << "+----------------------------------------------------------------+\n"
      << "| V1 concept : perception -> UAV XYZ+Yaw -> fixed PREWORK saw    |\n"
      << "| Cut point  : BranchModel.center (world)                        |\n"
      << "| Cut motion : UAV slow forward insertion                        |\n"
      << "| Recovery   : pause -> QGC -> resume / recompute current stage  |\n"
      << "+----------------------------------------------------------------+\n"
      << " Quick: status | start | pause | resume | help\n"
      << " Debug: takeoff observe scan lock far prework near cut retreat\n"
      << "        armhome land retry reset reset_target sethome abort params\n\n"
      << "[CUT-DEMO:IDLE] > " << std::flush;
  }

  void print_help() const
  {
    std::lock_guard<std::mutex> lock(print_mutex_);
    std::cout
      << "\n================ OFFBOARD CUTTING / COMMANDS =================\n"
      << " Full demo\n"
      << "   start        : run the complete automatic demo\n"
      << "\n Mission control\n"
      << "   status       : inspect UAV / target / arm / cutting state\n"
      << "   pause        : stop auto progression, Saw OFF, hand PX4 to POSCTL\n"
      << "   resume       : re-enter Offboard from CURRENT pose and recompute stage\n"
      << "   retry        : rerun the last/current stage\n"
      << "   reset        : clear task logic/target; do NOT force UAV/arm motion\n"
      << "   reset_target : clear locked target + reset perception fusion\n"
      << "   sethome      : sample current world pose as mission home\n"
      << "   abort        : Saw OFF + controlled retreat + arm HOME\n"
      << "\n Step-by-step debug\n"
      << "   takeoff -> observe -> scan -> lock -> far -> prework -> near -> cut\n"
      << "   retreat -> armhome -> land\n"
      << "\n Notes\n"
      << "   PREWORK is the fixed CUT pose in V1.\n"
      << "   After TARGET_LOCK, perception is allowed to disappear.\n"
      << "   CUT has both timeout and maximum forward-distance limits.\n"
      << "=======================================================\n"
      << "[CUT-DEMO:" << phase_name(phase_) << "] > " << std::flush;
  }

  void print_status() const
  {
    std::lock_guard<std::mutex> lock(print_mutex_);
    std::cout << "\n================ OFFBOARD CUTTING / STATUS ===================\n";
    std::cout << "Phase       : " << phase_name(phase_)
              << (auto_chain_ ? "  [AUTO]" : "  [MANUAL/STEP]") << "\n";
    std::cout << "PX4 data    : odom=" << yesno(px4_odom_received_)
              << " status=" << yesno(vehicle_status_received_)
              << " offboard=" << yesno(is_offboard_active())
              << " armed=" << yesno(
                vehicle_status_received_ &&
                vehicle_arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
              << "\n";
    std::cout << "PX4 status  : "
              << (vehicle_status_source_topic_.empty() ? "waiting" : vehicle_status_source_topic_)
              << "\n";
    std::cout << "World data  : odom=" << yesno(world_odom_received_)
              << " map=" << yesno(frame_map_valid_)
              << " map_err=" << std::fixed << std::setprecision(3)
              << frame_map_error_m_ << " m\n";
    if (world_odom_received_) {
      std::cout << "UAV world   : " << fmt_vec(world_position_)
                << "  yaw=" << std::fixed << std::setprecision(1)
                << rad2deg(world_yaw_) << " deg\n";
    }
    if (home_world_valid_) {
      std::cout << "Mission home: " << fmt_vec(home_world_)
                << "  yaw=" << std::fixed << std::setprecision(1)
                << rad2deg(home_world_yaw_) << " deg\n";
    }
    if (motion_target_valid_) {
      std::cout << "Flight tgt  : " << fmt_vec(motion_target_world_)
                << "  yaw=" << std::fixed << std::setprecision(1)
                << rad2deg(motion_target_yaw_) << " deg\n";
    }
    std::cout << "Perception  : received=" << yesno(branch_model_received_)
              << " live_valid=" << yesno(live_branch_valid_)
              << " samples=" << branch_samples_.size() << "\n";
    if (live_branch_valid_) {
      std::cout << "Live center : " << fmt_vec(live_branch_.center)
                << "  pts=" << live_branch_.point_count
                << " rms=" << std::fixed << std::setprecision(4)
                << live_branch_.fit_rms << "\n";
    }
    std::cout << "Target lock : " << yesno(locked_target_.valid);
    if (locked_target_.valid) {
      std::cout << "  center=" << fmt_vec(locked_target_.center);
    }
    std::cout << "\n";
    std::cout << "Arm         : status=" << yesno(arm_status_received_)
              << " busy=" << yesno(arm_busy_) << " task=" << arm_task_ << "\n";
    std::cout << "Saw         : cmd=" << std::fixed << std::setprecision(0)
              << saw_command_actual_from_arm_ << " rpm  actual=" << saw_actual_rpm_ << " rpm\n";
    std::cout << "Cut         : contact=" << yesno(target_contact_)
              << " progress=" << std::setprecision(0) << (100.0 * cut_progress_)
              << "% success=" << yesno(cut_success_) << "\n";
    if (geometry_valid_) {
      std::cout << "FAR / NEAR  : " << fmt_vec(far_approach_)
                << " / " << fmt_vec(near_approach_) << "\n";
    }
    std::cout << "=======================================================\n"
              << "[CUT-DEMO:" << phase_name(phase_) << "] > " << std::flush;
  }

  void print_params() const
  {
    std::lock_guard<std::mutex> lock(print_mutex_);
    std::cout
      << "\n================ OFFBOARD CUTTING / KEY PARAMETERS ============\n"
      << "tree_hint             : [" << tree_hint_x_ << ", " << tree_hint_y_ << "] world\n"
      << "takeoff_height        : " << takeoff_height_ << " m above sampled home\n"
      << "observe_distance      : " << observe_distance_ << " m\n"
      << "observe bearing offset: " << observe_bearing_offset_deg_ << " deg\n"
      << "arc half-angle        : " << arc_half_angle_deg_ << " deg\n"
      << "cut_z_offset          : +" << cut_z_offset_ << " m\n"
      << "tool_forward_offset   : " << tool_forward_offset_ << " m\n"
      << "far / near standoff   : " << far_standoff_ << " / " << near_standoff_ << " m\n"
      << "cut speed / max dist  : " << cut_in_speed_ << " m/s / "
      << cut_in_max_distance_ << " m\n"
      << "saw cmd / ready       : " << saw_command_rpm_ << " / " << saw_ready_rpm_ << " rpm\n"
      << "=======================================================\n"
      << "[CUT-DEMO:" << phase_name(phase_) << "] > " << std::flush;
  }

  static std::string yesno(bool value)
  {
    return value ? "YES" : "NO";
  }

  static std::string format_number(double value, int precision)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  static std::string phase_name(Phase p)
  {
    switch (p) {
      case Phase::IDLE: return "IDLE";
      case Phase::ARM_HOME_PREP: return "ARM_HOME_PREP";
      case Phase::OFFBOARD_PRESTREAM: return "OFFBOARD_PRESTREAM";
      case Phase::TAKEOFF: return "TAKEOFF";
      case Phase::OBSERVE: return "OBSERVE";
      case Phase::ARC_SCAN: return "ARC_SCAN";
      case Phase::TARGET_LOCK: return "TARGET_LOCK";
      case Phase::FAR_APPROACH: return "FAR_APPROACH";
      case Phase::ARM_PREWORK: return "ARM_PREWORK";
      case Phase::NEAR_APPROACH: return "NEAR_APPROACH";
      case Phase::SAW_SPINUP: return "SAW_SPINUP";
      case Phase::CUT_IN: return "CUT_IN";
      case Phase::CUT_WAIT: return "CUT_WAIT";
      case Phase::RETREAT_NEAR: return "RETREAT_NEAR";
      case Phase::RETREAT_FAR: return "RETREAT_FAR";
      case Phase::ARM_HOME_POST: return "ARM_HOME_POST";
      case Phase::RETURN_HOME: return "RETURN_HOME";
      case Phase::LANDING: return "LANDING";
      case Phase::HOLD: return "HOLD";
      case Phase::PAUSED: return "PAUSED";
      case Phase::CUT_FAILED: return "CUT_FAILED";
      case Phase::DONE: return "DONE";
    }
    return "UNKNOWN";
  }

  // ============================================================
  // ROS members
  // ============================================================

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
  std::vector<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr>
    vehicle_status_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_sub_;
  rclcpp::Subscription<BranchModel>::SharedPtr branch_model_sub_;
  rclcpp::Subscription<ArmStatus>::SharedPtr arm_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cut_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cut_success_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_contact_sub_;

  rclcpp_action::Client<ArmMotion>::SharedPtr arm_client_;
  rclcpp::Client<SetSaw>::SharedPtr saw_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr perception_reset_client_;
  ArmGoalHandle::SharedPtr arm_goal_handle_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // ============================================================
  // Parameter storage
  // ============================================================

  double tree_hint_x_{4.0};
  double tree_hint_y_{0.1};
  double takeoff_height_{3.0};
  double observe_distance_{1.4};
  double observe_bearing_offset_deg_{17.0};
  double arc_half_angle_deg_{25.0};
  double arc_settle_sec_{1.0};

  int perception_min_points_{60};
  double perception_max_fit_rms_{0.03};
  int perception_stable_samples_{5};
  double perception_center_spread_max_{0.05};

  double cut_z_offset_{0.455};
  double tool_forward_offset_{0.19};
  double near_standoff_{0.20};
  double far_standoff_{0.75};
  double cut_in_penetration_{0.04};
  double cut_in_max_distance_{0.30};

  double takeoff_speed_{0.50};
  double observe_speed_{0.45};
  double far_speed_{0.55};
  double near_speed_{0.18};
  double cut_in_speed_{0.04};
  double retreat_speed_{0.12};
  double return_speed_{0.60};
  double yaw_rate_rad_s_{deg2rad(35.0)};
  double position_tolerance_{0.08};
  double near_position_tolerance_{0.04};
  double yaw_tolerance_rad_{deg2rad(5.0)};

  double saw_command_rpm_{1000.0};
  double saw_ready_rpm_{850.0};
  double saw_ready_hold_sec_{0.40};
  double cut_timeout_sec_{4.0};

  int offboard_prestream_cycles_{20};
  bool auto_land_after_cut_{true};
  bool reset_perception_before_scan_{true};

  std::string gazebo_odom_topic_;
  std::string px4_vehicle_odometry_topic_;
  std::vector<std::string> px4_vehicle_status_topics_;
  std::string px4_offboard_control_mode_topic_;
  std::string px4_trajectory_setpoint_topic_;
  std::string px4_vehicle_command_topic_;
  std::string vehicle_status_source_topic_;
  std::string branch_model_topic_;
  std::string arm_status_topic_;
  std::string cut_progress_topic_;
  std::string cut_success_topic_;
  std::string target_contact_topic_;

  // ============================================================
  // Runtime state
  // ============================================================

  Phase phase_{Phase::IDLE};
  Phase last_task_phase_{Phase::IDLE};
  Phase pending_after_offboard_{Phase::IDLE};
  Phase paused_phase_{Phase::IDLE};
  bool phase_initialized_{false};
  rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};

  bool auto_chain_{false};
  std::optional<Phase> manual_stop_after_;
  bool manual_retreat_to_home_arm_{false};
  bool manual_land_request_{false};
  bool paused_phase_valid_{false};
  bool auto_chain_before_pause_{false};
  std::optional<Phase> manual_stop_before_pause_;

  bool px4_odom_received_{false};
  bool vehicle_status_received_{false};
  bool world_odom_received_{false};
  bool home_world_valid_{false};
  bool frame_map_valid_{false};

  Vec3 px4_ned_;
  Vec3 px4_velocity_ned_;
  double px4_yaw_{0.0};
  std::uint8_t vehicle_nav_state_{0};
  std::uint8_t vehicle_arming_state_{0};

  Vec3 world_position_;
  Vec3 world_velocity_;
  double world_yaw_{0.0};
  Vec3 home_world_;
  double home_world_yaw_{0.0};

  double ned_offset_x_{0.0};
  double ned_offset_y_{0.0};
  double ned_offset_z_{0.0};
  double ned_yaw_offset_{0.0};
  double frame_map_error_m_{0.0};
  int frame_map_bad_counter_{0};

  bool offboard_streaming_{false};
  bool offboard_expected_{false};
  bool offboard_confirmed_once_{false};
  int offboard_loss_counter_{0};
  int prestream_counter_{0};
  bool offboard_mode_command_sent_{false};
  bool arm_on_offboard_entry_{false};
  rclcpp::Time offboard_command_time_{0, 0, RCL_ROS_TIME};

  Vec3 motion_target_world_;
  double motion_target_yaw_{0.0};
  double motion_speed_{0.0};
  double motion_tolerance_{0.08};
  bool motion_target_valid_{false};
  Vec3 commanded_sp_world_;
  double commanded_sp_yaw_{0.0};

  bool branch_model_received_{false};
  bool live_branch_valid_{false};
  BranchSample live_branch_;
  std::deque<BranchSample> branch_samples_;
  LockedTarget locked_target_;

  Vec3 observe_center_;
  double observe_center_bearing_{0.0};
  std::vector<Vec3> arc_points_;
  std::size_t arc_index_{0};
  std::optional<rclcpp::Time> arc_arrival_since_;
  rclcpp::Time target_lock_last_attempt_{0, 0, RCL_ROS_TIME};

  bool geometry_valid_{false};
  Vec3 insertion_direction_;
  double insertion_yaw_{0.0};
  Vec3 contact_base_;
  Vec3 near_approach_;
  Vec3 far_approach_;
  Vec3 cut_in_target_;
  Vec3 cut_in_start_world_;

  bool arm_status_received_{false};
  bool arm_busy_{false};
  std::string arm_task_{"UNKNOWN"};
  std::array<double, 3> arm_joint_actual_{{0.0, 0.0, 0.0}};
  double saw_command_actual_from_arm_{0.0};
  double saw_actual_rpm_{0.0};
  bool arm_action_done_{false};
  bool arm_action_failed_{false};
  std::string arm_action_message_;
  std::string arm_feedback_state_;
  float arm_feedback_progress_{0.0f};

  bool saw_request_accepted_{false};
  bool saw_request_failed_{false};
  std::optional<rclcpp::Time> saw_ready_since_;

  bool target_contact_{false};
  bool cut_success_{false};
  bool cut_failure_pending_pause_{false};
  double cut_progress_{0.0};
  rclcpp::Time cut_wait_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time landing_command_sent_time_{0, 0, RCL_ROS_TIME};

  // CLI / printing
  mutable std::mutex print_mutex_;
  std::mutex command_mutex_;
  std::deque<std::string> command_queue_;
  std::thread cli_thread_;
  std::atomic<bool> stop_cli_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardCuttingDemo>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
