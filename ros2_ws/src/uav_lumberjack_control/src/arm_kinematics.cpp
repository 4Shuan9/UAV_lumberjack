#include "uav_lumberjack_control/arm_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace uav_lumberjack_control
{

Mat3 ArmKinematics::rotX(double a)
{
  Mat3 r;
  const double c = std::cos(a);
  const double s = std::sin(a);
  r.data = {
    1.0, 0.0, 0.0,
    0.0, c, -s,
    0.0, s, c};
  return r;
}

Mat3 ArmKinematics::rotY(double a)
{
  Mat3 r;
  const double c = std::cos(a);
  const double s = std::sin(a);
  r.data = {
    c, 0.0, s,
    0.0, 1.0, 0.0,
    -s, 0.0, c};
  return r;
}

Mat3 ArmKinematics::multiply(const Mat3 & a, const Mat3 & b)
{
  Mat3 out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = 0.0;
      for (int k = 0; k < 3; ++k) {
        out(r, c) += a(r, k) * b(k, c);
      }
    }
  }
  return out;
}

Vec3 ArmKinematics::multiply(const Mat3 & r, const Vec3 & v)
{
  return {
    r(0, 0) * v.x + r(0, 1) * v.y + r(0, 2) * v.z,
    r(1, 0) * v.x + r(1, 1) * v.y + r(1, 2) * v.z,
    r(2, 0) * v.x + r(2, 1) * v.y + r(2, 2) * v.z};
}

Vec3 ArmKinematics::add(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 ArmKinematics::subtract(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double ArmKinematics::norm(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double ArmKinematics::normalizeAngle(double a)
{
  while (a > PI) a -= 2.0 * PI;
  while (a <= -PI) a += 2.0 * PI;
  return a;
}

JointAngles ArmKinematics::degreesToRadians(double q2_deg, double q3_deg, double q4_deg)
{
  return {q2_deg * DEG_TO_RAD, q3_deg * DEG_TO_RAD, q4_deg * DEG_TO_RAD};
}

JointAngles ArmKinematics::radiansToDegrees(const JointAngles & q)
{
  return {q.q2 * RAD_TO_DEG, q.q3 * RAD_TO_DEG, q.q4 * RAD_TO_DEG};
}

bool ArmKinematics::withinJointLimits(const JointAngles & q, double eps)
{
  return q.q2 >= J2_MIN - eps && q.q2 <= J2_MAX + eps &&
         q.q3 >= J3_MIN - eps && q.q3 <= J3_MAX + eps &&
         q.q4 >= J4_MIN - eps && q.q4 <= J4_MAX + eps;
}

Mat3 ArmKinematics::makeToolRotation(double tool_pitch_rad, double tool_roll_rad)
{
  return multiply(rotY(tool_pitch_rad), rotX(tool_roll_rad));
}

Pose3 ArmKinematics::forward(const JointAngles & q)
{
  // Logical-angle model derived from the baked HOME geometry in model.sdf:
  //   J2 physical pitch = -q2
  //   forearm physical pitch = -(q2 + q3)
  //   wrist physical roll relative to forearm = q4
  //   chainsaw_body has an additional fixed +90 deg roll.
  const double q23 = q.q2 + q.q3;
  const Mat3 r_upper = rotY(-q.q2);
  const Mat3 r_forearm = rotY(-q23);
  const Mat3 r_wrist = multiply(r_forearm, rotX(q.q4));

  Vec3 p{0.0, 0.0, BASE_TO_J2_Z};
  p = add(p, multiply(r_upper, Vec3{UPPER_ARM_LENGTH, 0.0, 0.0}));
  p = add(p, multiply(r_forearm, Vec3{FOREARM_LENGTH + J4_TO_WRIST_ORIGIN, 0.0, 0.0}));
  p = add(p, multiply(r_wrist, Vec3{WRIST_TO_SAW_X, WRIST_TO_SAW_Y, 0.0}));

  Pose3 out;
  out.position = p;
  out.rotation = multiply(r_forearm, rotX(q.q4 + PI / 2.0));
  return out;
}

double ArmKinematics::orientationError(const Mat3 & desired, const Mat3 & actual)
{
  // angle(R_desired^T * R_actual)
  double trace = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) {
      trace += desired(k, i) * actual(k, i);
    }
  }
  const double cos_angle = std::clamp((trace - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cos_angle);
}

bool ArmKinematics::nearlySame(const JointAngles & a, const JointAngles & b, double tol)
{
  return std::abs(normalizeAngle(a.q2 - b.q2)) < tol &&
         std::abs(normalizeAngle(a.q3 - b.q3)) < tol &&
         std::abs(normalizeAngle(a.q4 - b.q4)) < tol;
}

std::vector<IkSolution> ArmKinematics::inversePosition(
  const Vec3 & target,
  double tolerance_m)
{
  std::vector<IkSolution> solutions;

  // Because J2/J3 rotate about Y, the only lateral displacement of the V1 TCP
  // comes from the 30 mm wrist->saw Y offset rotated by J4:
  //   y_tcp = 0.030 * cos(q4)
  if (std::abs(target.y) > WRIST_TO_SAW_Y + tolerance_m) {
    return solutions;
  }

  const double cos_q4 = std::clamp(target.y / WRIST_TO_SAW_Y, -1.0, 1.0);
  const double q4_abs = std::acos(cos_q4);
  std::vector<double> q4_candidates{q4_abs, -q4_abs};
  if (std::abs(q4_abs) < 1e-10 || std::abs(q4_abs - PI) < 1e-10) {
    q4_candidates.resize(1);
  }

  const double px = target.x;
  const double pz = target.z - BASE_TO_J2_Z;
  const double first = UPPER_ARM_LENGTH;
  const double fixed_x = FOREARM_LENGTH + J4_TO_WRIST_ORIGIN + WRIST_TO_SAW_X;

  for (double q4 : q4_candidates) {
    const double second_z = WRIST_TO_SAW_Y * std::sin(q4);
    const double second = std::hypot(fixed_x, second_z);
    const double delta = std::atan2(second_z, fixed_x);

    const double r2 = px * px + pz * pz;
    double cos_elbow = (r2 - first * first - second * second) / (2.0 * first * second);
    if (cos_elbow < -1.0 - 1e-10 || cos_elbow > 1.0 + 1e-10) {
      continue;
    }
    cos_elbow = std::clamp(cos_elbow, -1.0, 1.0);

    const double elbow_abs = std::acos(cos_elbow);
    std::vector<double> elbow_candidates{elbow_abs, -elbow_abs};
    if (std::abs(elbow_abs) < 1e-10 || std::abs(elbow_abs - PI) < 1e-10) {
      elbow_candidates.resize(1);
    }

    for (double elbow : elbow_candidates) {
      const double q2 = std::atan2(pz, px) -
        std::atan2(second * std::sin(elbow), first + second * std::cos(elbow));
      const double q3 = elbow - delta;

      JointAngles q{
        normalizeAngle(q2),
        normalizeAngle(q3),
        normalizeAngle(q4)};

      const Pose3 fk = forward(q);
      const double position_error = norm(subtract(fk.position, target));
      if (position_error > std::max(tolerance_m, 1e-8)) {
        continue;
      }

      bool duplicate = false;
      for (const auto & existing : solutions) {
        if (nearlySame(existing.q, q)) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) continue;

      solutions.push_back({q, withinJointLimits(q), position_error, 0.0});
    }
  }

  return solutions;
}

std::vector<IkSolution> ArmKinematics::inversePose(
  const Pose3 & target,
  double position_tolerance_m,
  double orientation_tolerance_rad)
{
  std::vector<IkSolution> solutions;

  // R = Ry(a) * Rx(b), with a = -(q2+q3), b = q4 + pi/2.
  const double a = std::atan2(-target.rotation(2, 0), target.rotation(0, 0));
  const double b = std::atan2(-target.rotation(1, 2), target.rotation(1, 1));
  const double q23 = -a;
  const double q4 = normalizeAngle(b - PI / 2.0);

  // Reject orientations outside this 3-DOF mechanism family.
  const Mat3 reconstructed_orientation = makeToolRotation(a, b);
  if (orientationError(target.rotation, reconstructed_orientation) > orientation_tolerance_rad) {
    return solutions;
  }

  const Mat3 r_forearm = rotY(-q23);
  const Mat3 r_wrist = multiply(r_forearm, rotX(q4));

  Vec3 known_offset = multiply(
    r_forearm,
    Vec3{FOREARM_LENGTH + J4_TO_WRIST_ORIGIN, 0.0, 0.0});
  known_offset = add(
    known_offset,
    multiply(r_wrist, Vec3{WRIST_TO_SAW_X, WRIST_TO_SAW_Y, 0.0}));

  const Vec3 base_to_j2{0.0, 0.0, BASE_TO_J2_Z};
  const Vec3 first_link_vector = subtract(subtract(target.position, base_to_j2), known_offset);

  // The first link must lie in the X-Z plane with exact length L1.
  if (std::abs(first_link_vector.y) > position_tolerance_m ||
      std::abs(norm(first_link_vector) - UPPER_ARM_LENGTH) > position_tolerance_m) {
    return solutions;
  }

  const double q2 = std::atan2(first_link_vector.z, first_link_vector.x);
  const double q3 = normalizeAngle(q23 - q2);

  JointAngles q{
    normalizeAngle(q2),
    q3,
    q4};

  const Pose3 fk = forward(q);
  const double position_error = norm(subtract(fk.position, target.position));
  const double orientation_error = orientationError(target.rotation, fk.rotation);

  if (position_error <= position_tolerance_m &&
      orientation_error <= orientation_tolerance_rad) {
    solutions.push_back({q, withinJointLimits(q), position_error, orientation_error});
  }

  return solutions;
}

}  // namespace uav_lumberjack_control
