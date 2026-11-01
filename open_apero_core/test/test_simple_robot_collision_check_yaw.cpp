/**
 * @file test_simple_robot_collision_check_yaw.cpp
 * @brief Orientation-dependent traversability deduction and explicit override.
 */

#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "open_apero_core/plugin/robot_model/simple_robot.h"

#include "plugin_test_stubs.h"

namespace open_apero {
namespace {

/// ConfigAgent with no budget and the default seed; it keeps the warnings the robot logs.
class LoggingAgent : public ConfigAgent {
 public:
  LoggingAgent() {
    setCostBudget(0.0);
    setRandomSeed(-1);
  }
};

class SimpleRobotProbe : public robot_model::SimpleRobot {
 public:
  explicit SimpleRobotProbe(AgentInterface& agent) : robot_model::SimpleRobot(agent) {}

  void setSphereCenters(const std::vector<Eigen::Vector3d>& collision, const std::vector<Eigen::Vector3d>& contact) {
    p_collision_spheres_.clear();
    p_contact_spheres_.clear();
    for (const auto& center : collision) {
      GeometricSphere sphere;
      sphere.center = center;
      p_collision_spheres_.push_back(sphere);
    }
    for (const auto& center : contact) {
      GeometricSphere sphere;
      sphere.center = center;
      p_contact_spheres_.push_back(sphere);
    }
  }

  bool deduce(std::string* source) const { return deduceCollisionCheckYaw(source); }
  void configure(Configurable::ParamMap* params) { configureCollisionCheckYaw(params); }
  bool effective() const { return isTraversabilityOrientationRelated(); }
};

TEST(CollisionCheckYaw, DeducesCenteredSpheresAsOrientationIndependent) {
  LoggingAgent agent;
  SimpleRobotProbe robot(agent);
  robot.setSphereCenters({Eigen::Vector3d(0.0, 0.0, 0.4)}, {Eigen::Vector3d(0.0, 0.0, 0.0)});

  std::string source;
  EXPECT_FALSE(robot.deduce(&source));
  EXPECT_EQ(source, "centered sphere geometry");
}

TEST(CollisionCheckYaw, DeducesEitherOffAxisSphereSetAsOrientationDependent) {
  LoggingAgent agent;
  SimpleRobotProbe robot(agent);
  std::string source;

  robot.setSphereCenters({Eigen::Vector3d(0.1, 0.0, 0.4)}, {});
  EXPECT_TRUE(robot.deduce(&source));
  EXPECT_EQ(source, "off-axis collision spheres");

  robot.setSphereCenters({}, {Eigen::Vector3d(0.0, -0.1, 0.0)});
  EXPECT_TRUE(robot.deduce(&source));
  EXPECT_EQ(source, "off-axis contact spheres");
}

TEST(CollisionCheckYaw, MissingParameterUsesTheDeduction) {
  LoggingAgent agent;
  SimpleRobotProbe robot(agent);
  robot.setSphereCenters({}, {Eigen::Vector3d(0.1, 0.0, 0.0)});
  Configurable::ParamMap params;

  robot.configure(&params);

  EXPECT_TRUE(robot.effective());
  EXPECT_TRUE(agent.warnings.empty());
}

TEST(CollisionCheckYaw, MatchingParameterIsAcceptedWithoutWarning) {
  LoggingAgent agent;
  SimpleRobotProbe robot(agent);
  robot.setSphereCenters({}, {Eigen::Vector3d(0.1, 0.0, 0.0)});
  Configurable::ParamMap params{{"collision_check_yaw", "1"}};

  robot.configure(&params);

  EXPECT_TRUE(robot.effective());
  EXPECT_TRUE(agent.warnings.empty());
}

TEST(CollisionCheckYaw, MismatchingParameterWarnsAndOverridesTheDeduction) {
  LoggingAgent agent;
  SimpleRobotProbe robot(agent);
  robot.setSphereCenters({}, {Eigen::Vector3d(0.1, 0.0, 0.0)});
  Configurable::ParamMap params{{"collision_check_yaw", "0"}};

  robot.configure(&params);

  EXPECT_FALSE(robot.effective());
  ASSERT_EQ(agent.warnings.size(), 1u);
  EXPECT_NE(agent.warnings.front().find("collision_check_yaw=false"), std::string::npos);
  EXPECT_NE(agent.warnings.front().find("deduced value true"), std::string::npos);
  EXPECT_NE(agent.warnings.front().find("off-axis contact spheres"), std::string::npos);
  EXPECT_NE(agent.warnings.front().find("honoring the configured value"), std::string::npos);
}

TEST(CollisionCheckYaw, TrueOverrideCanAlsoReplaceAFalseDeduction) {
  LoggingAgent agent;
  SimpleRobotProbe robot(agent);
  robot.setSphereCenters({Eigen::Vector3d(0.0, 0.0, 0.4)}, {});
  Configurable::ParamMap params{{"collision_check_yaw", "1"}};

  robot.configure(&params);

  EXPECT_TRUE(robot.effective());
  ASSERT_EQ(agent.warnings.size(), 1u);
  EXPECT_NE(agent.warnings.front().find("deduced value false"), std::string::npos);
  EXPECT_NE(agent.warnings.front().find("centered sphere geometry"), std::string::npos);
}

}  // namespace
}  // namespace open_apero
