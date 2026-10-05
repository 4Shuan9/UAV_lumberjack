#include "uav_lumberjack_control/arm_kinematics.hpp"

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>

using uav_lumberjack_control::ArmKinematics;
using uav_lumberjack_control::IkSolution;
using uav_lumberjack_control::JointAngles;
using uav_lumberjack_control::Pose3;
using uav_lumberjack_control::Vec3;

namespace
{

double parseDouble(const char * text)
{
  char * end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    throw std::runtime_error(std::string("Invalid number: ") + text);
  }
  return value;
}

void printUsage()
{
  std::cout <<
    "UAV_lumberjack arm kinematics V1 (logical angles, TCP=chainsaw_body origin)\n\n"
    "Usage:\n"
    "  arm_kinematics_cli fk <q2_deg> <q3_deg> <q4_deg>\n"
    "  arm_kinematics_cli ik <x_m> <y_m> <z_m>\n"
    "  arm_kinematics_cli ikpose <x_m> <y_m> <z_m> <tool_pitch_deg> <tool_roll_deg>\n"
    "  arm_kinematics_cli selftest\n\n"
    "Orientation convention for ikpose:\n"
    "  R_tcp = Ry(tool_pitch) * Rx(tool_roll)\n"
    "  FK gives tool_pitch = -(q2+q3), tool_roll = q4+90 deg.\n";
}

void printPose(const Pose3 & pose)
{
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "TCP position [m] : ["
            << pose.position.x << ", "
            << pose.position.y << ", "
            << pose.position.z << "]\n";
  std::cout << "TCP rotation R   :\n";
  for (int r = 0; r < 3; ++r) {
    std::cout << "  [";
    for (int c = 0; c < 3; ++c) {
      if (c) std::cout << ", ";
      std::cout << std::setw(10) << pose.rotation(r, c);
    }
    std::cout << "]\n";
  }
}

void printSolutions(const std::vector<IkSolution> & solutions)
{
  if (solutions.empty()) {
    std::cout << "IK: no geometric solution.\n";
    return;
  }

  std::size_t valid_count = 0;
  for (std::size_t i = 0; i < solutions.size(); ++i) {
    const auto deg = ArmKinematics::radiansToDegrees(solutions[i].q);
    std::cout << std::fixed << std::setprecision(3)
              << "candidate " << (i + 1) << ": "
              << "q2=" << deg.q2 << " deg, "
              << "q3=" << deg.q3 << " deg, "
              << "q4=" << deg.q4 << " deg  "
              << (solutions[i].within_joint_limits ? "[VALID]" : "[OUT_OF_LIMIT]")
              << "  pos_err=" << std::setprecision(9) << solutions[i].position_error_m << " m";
    if (solutions[i].orientation_error_rad > 0.0) {
      std::cout << "  rot_err=" << solutions[i].orientation_error_rad << " rad";
    }
    std::cout << "\n";
    if (solutions[i].within_joint_limits) ++valid_count;
  }
  std::cout << "valid candidates: " << valid_count << " / " << solutions.size() << "\n";
}

bool near(double a, double b, double tol)
{
  return std::abs(a - b) <= tol;
}

bool runRoundTrip(const JointAngles & q, const char * name)
{
  const Pose3 pose = ArmKinematics::forward(q);
  const auto solutions = ArmKinematics::inversePose(pose, 1e-9, 1e-7);
  if (solutions.empty()) {
    std::cerr << "[FAIL] " << name << ": pose IK returned no solution\n";
    return false;
  }

  for (const auto & s : solutions) {
    if (near(ArmKinematics::normalizeAngle(s.q.q2 - q.q2), 0.0, 1e-7) &&
        near(ArmKinematics::normalizeAngle(s.q.q3 - q.q3), 0.0, 1e-7) &&
        near(ArmKinematics::normalizeAngle(s.q.q4 - q.q4), 0.0, 1e-7)) {
      std::cout << "[PASS] " << name << " FK -> IK round trip\n";
      return true;
    }
  }

  std::cerr << "[FAIL] " << name << ": pose IK did not recover original logical angles\n";
  return false;
}

int selfTest()
{
  bool ok = true;

  const JointAngles home = ArmKinematics::degreesToRadians(-30.0, -150.0, -90.0);
  const Pose3 home_pose = ArmKinematics::forward(home);

  // Exact value independently reconstructed from the current SDF HOME link poses.
  ok &= near(home_pose.position.x, -0.0991155, 2e-6);
  ok &= near(home_pose.position.y, 0.0, 2e-9);
  ok &= near(home_pose.position.z, -0.1700000, 2e-6);
  if (ok) {
    std::cout << "[PASS] HOME FK matches current SDF geometry\n";
  } else {
    std::cerr << "[FAIL] HOME FK does not match current SDF geometry\n";
  }

  ok &= runRoundTrip(home, "HOME");
  ok &= runRoundTrip(ArmKinematics::degreesToRadians(-60.0, 60.0, 0.0), "PREWORK");
  ok &= runRoundTrip(ArmKinematics::degreesToRadians(-45.0, -45.0, 0.0), "CUT_A");
  ok &= runRoundTrip(ArmKinematics::degreesToRadians(-30.0, -60.0, 0.0), "CUT_B");

  const auto prework_pose = ArmKinematics::forward(
    ArmKinematics::degreesToRadians(-60.0, 60.0, 0.0));
  const auto position_solutions = ArmKinematics::inversePosition(prework_pose.position, 1e-9);
  bool recovered_prework = false;
  for (const auto & s : position_solutions) {
    const auto deg = ArmKinematics::radiansToDegrees(s.q);
    if (s.within_joint_limits && near(deg.q2, -60.0, 1e-5) &&
        near(deg.q3, 60.0, 1e-5) && near(deg.q4, 0.0, 1e-5)) {
      recovered_prework = true;
      break;
    }
  }
  if (recovered_prework) {
    std::cout << "[PASS] PREWORK position IK contains original solution\n";
  } else {
    std::cerr << "[FAIL] PREWORK position IK missed original solution\n";
    ok = false;
  }

  std::cout << (ok ? "SELFTEST: PASS\n" : "SELFTEST: FAIL\n");
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 2) {
      printUsage();
      return 1;
    }

    const std::string command = argv[1];

    if (command == "fk") {
      if (argc != 5) {
        printUsage();
        return 1;
      }
      const JointAngles q = ArmKinematics::degreesToRadians(
        parseDouble(argv[2]), parseDouble(argv[3]), parseDouble(argv[4]));
      std::cout << "joint limits      : "
                << (ArmKinematics::withinJointLimits(q) ? "VALID" : "OUT_OF_LIMIT") << "\n";
      printPose(ArmKinematics::forward(q));
      return 0;
    }

    if (command == "ik") {
      if (argc != 5) {
        printUsage();
        return 1;
      }
      const Vec3 target{parseDouble(argv[2]), parseDouble(argv[3]), parseDouble(argv[4])};
      printSolutions(ArmKinematics::inversePosition(target));
      return 0;
    }

    if (command == "ikpose") {
      if (argc != 7) {
        printUsage();
        return 1;
      }
      Pose3 target;
      target.position = {parseDouble(argv[2]), parseDouble(argv[3]), parseDouble(argv[4])};
      target.rotation = ArmKinematics::makeToolRotation(
        parseDouble(argv[5]) * ArmKinematics::DEG_TO_RAD,
        parseDouble(argv[6]) * ArmKinematics::DEG_TO_RAD);
      printSolutions(ArmKinematics::inversePose(target));
      return 0;
    }

    if (command == "selftest") {
      return selfTest();
    }

    printUsage();
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 2;
  }
}
