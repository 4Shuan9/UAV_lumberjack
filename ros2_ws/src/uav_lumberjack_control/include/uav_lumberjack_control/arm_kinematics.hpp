#pragma once

#include <array>
#include <vector>

namespace uav_lumberjack_control
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3
{
  // Row-major storage.
  std::array<double, 9> data{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};

  double operator()(int row, int col) const { return data[3 * row + col]; }
  double & operator()(int row, int col) { return data[3 * row + col]; }
};

struct Pose3
{
  Vec3 position;
  Mat3 rotation;
};

struct JointAngles
{
  // Logical joint angles [rad], not Gazebo raw joint angles.
  double q2{0.0};
  double q3{0.0};
  double q4{0.0};
};

struct IkSolution
{
  JointAngles q;
  bool within_joint_limits{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

class ArmKinematics
{
public:
  static constexpr double PI = 3.14159265358979323846;
  static constexpr double DEG_TO_RAD = PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / PI;

  // Geometry derived from the current x500_lumberjack/model.sdf.
  static constexpr double BASE_TO_J2_Z = -0.110;       // base_link -> J2 [m]
  static constexpr double UPPER_ARM_LENGTH = 0.180;   // J2 -> J3 [m]
  static constexpr double FOREARM_LENGTH = 0.170;     // J3 -> J4 [m]
  static constexpr double J4_TO_WRIST_ORIGIN = 0.025; // J4 -> wrist_link origin [m]
  static constexpr double WRIST_TO_SAW_X = 0.060;     // wrist_link -> chainsaw_body [m]
  static constexpr double WRIST_TO_SAW_Y = 0.030;     // wrist_link -> chainsaw_body [m]

  // Logical joint limits used by arm_controller.cpp.
  static constexpr double J2_MIN = -165.0 * DEG_TO_RAD;
  static constexpr double J2_MAX = 15.0 * DEG_TO_RAD;
  static constexpr double J3_MIN = -150.0 * DEG_TO_RAD;
  static constexpr double J3_MAX = 150.0 * DEG_TO_RAD;
  static constexpr double J4_MIN = -180.0 * DEG_TO_RAD;
  static constexpr double J4_MAX = 180.0 * DEG_TO_RAD;

  // V1 TCP definition: chainsaw_body origin.
  // A later cutting-point offset can be composed after this transform without
  // changing the arm joint model.
  static Pose3 forward(const JointAngles & q);

  // Position-only IK for the V1 TCP. Returns all geometric candidates; use
  // within_joint_limits to select executable candidates.
  static std::vector<IkSolution> inversePosition(
    const Vec3 & target_position,
    double tolerance_m = 1e-6);

  // Exact pose IK. The arm has only three DOF, so the target orientation must
  // satisfy the mechanism form R = Ry(tool_pitch) * Rx(tool_roll).
  static std::vector<IkSolution> inversePose(
    const Pose3 & target_pose,
    double position_tolerance_m = 1e-6,
    double orientation_tolerance_rad = 1e-6);

  // Helper for CLI / future planners: construct the only orientation family
  // this 3-DOF arm can realize at chainsaw_body.
  static Mat3 makeToolRotation(double tool_pitch_rad, double tool_roll_rad);

  static bool withinJointLimits(const JointAngles & q, double eps = 1e-9);
  static JointAngles degreesToRadians(double q2_deg, double q3_deg, double q4_deg);
  static JointAngles radiansToDegrees(const JointAngles & q_rad);
  static double normalizeAngle(double angle_rad);

private:
  static Mat3 rotX(double angle_rad);
  static Mat3 rotY(double angle_rad);
  static Mat3 multiply(const Mat3 & a, const Mat3 & b);
  static Vec3 multiply(const Mat3 & r, const Vec3 & v);
  static Vec3 add(const Vec3 & a, const Vec3 & b);
  static Vec3 subtract(const Vec3 & a, const Vec3 & b);
  static double norm(const Vec3 & v);
  static double orientationError(const Mat3 & desired, const Mat3 & actual);
  static bool nearlySame(const JointAngles & a, const JointAngles & b, double tol = 1e-7);
};

}  // namespace uav_lumberjack_control
