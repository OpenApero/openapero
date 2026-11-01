/**
 * @file test_utils.cpp
 * @brief Quaternion ingress validation: isUnitQuaternion decides what may enter
 *        the planner as an orientation. It validates only — a non-unit
 *        quaternion is the producer's bug and is rejected, never repaired.
 *        Also covers the orientation decompositions applied at that boundary
 *        (eulerAnglesZyxFromQuaternion, yawPitchFromQuaternion,
 *        Agent::zeroSensorRoll) and rateStep.
 */

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/agent/agent.h"
#include "open_apero_core/common/typedef.h"
#include "open_apero_core/common/utils.h"
#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace {

constexpr double kDefaultQuaternionTolerance = 1e-7;

Eigen::Quaterniond yawQuat(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

// Eigen's eulerAngles(2, 1, 0), with its fold of yaw into [0, pi] undone: a negative yaw comes back as the
// equivalent (yaw + pi, +-pi - pitch, roll +- pi).
Eigen::Vector3d unfoldedEigenEulerAnglesZyx(const Eigen::Quaterniond& q) {
  Eigen::Vector3d zyx = q.toRotationMatrix().eulerAngles(2, 1, 0);
  if (std::abs(zyx(1)) > M_PI / 2.0) {
    zyx = {zyx(0) - M_PI, std::copysign(M_PI, zyx(1)) - zyx(1), zyx(2) - std::copysign(M_PI, zyx(2))};
  }
  return zyx;
}

// Agent::zeroSensorRoll is protected and static; deriving reaches it without
// constructing an agent.
struct RollNormalizer : Agent {
  using Agent::zeroSensorRoll;
};

TEST(IsUnitQuaternion, AcceptsAUnitQuaternion) {
  EXPECT_TRUE(isUnitQuaternion(yawQuat(0.7)));
  EXPECT_TRUE(isUnitQuaternion(Eigen::Quaterniond::Identity()));
}

TEST(IsUnitQuaternion, AcceptsRepresentationRoundingOnly) {
  // Exercise the actual ROS-style float32 representation round-trip rather
  // than simulating it by scaling a double-precision quaternion.
  const Eigen::Quaternionf float_payload = yawQuat(1.2).cast<float>();
  const Eigen::Quaterniond rounded = float_payload.cast<double>();

  ASSERT_GT(std::abs(rounded.norm() - 1.0), 0.0);
  ASSERT_LE(std::abs(rounded.norm() - 1.0), kDefaultQuaternionTolerance);
  EXPECT_TRUE(isUnitQuaternion(rounded));
}

TEST(IsUnitQuaternion, AcceptsTheLargestRepresentableNormWithinTheDefaultBoundary) {
  // 1.0 + 1e-7 rounds just outside the mathematical boundary. Its immediate
  // predecessor is the largest representable norm on the accepted side.
  const double boundary_norm = std::nextafter(1.0 + kDefaultQuaternionTolerance, 1.0);
  const Eigen::Quaterniond at_boundary(boundary_norm, 0.0, 0.0, 0.0);

  ASSERT_LE(std::abs(at_boundary.norm() - 1.0), kDefaultQuaternionTolerance);
  EXPECT_TRUE(isUnitQuaternion(at_boundary));
}

TEST(IsUnitQuaternion, RejectsTheFirstRepresentableNormBeyondTheDefaultBoundary) {
  const double outside_norm = 1.0 + kDefaultQuaternionTolerance;
  const Eigen::Quaterniond outside(outside_norm, 0.0, 0.0, 0.0);

  ASSERT_GT(std::abs(outside.norm() - 1.0), kDefaultQuaternionTolerance);
  EXPECT_FALSE(isUnitQuaternion(outside));
}

TEST(IsUnitQuaternion, RejectsAFiveEMinusSevenDeviation) {
  Eigen::Quaterniond scaled = yawQuat(1.2);
  scaled.coeffs() *= 1.0 + 5e-7;

  ASSERT_GT(std::abs(scaled.norm() - 1.0), kDefaultQuaternionTolerance);
  EXPECT_FALSE(isUnitQuaternion(scaled));
}

TEST(IsUnitQuaternion, RejectsAGenuinelyDenormalizedQuaternion) {
  // This is a producer bug, not rounding; it must be reported, not repaired.
  Eigen::Quaterniond scaled = yawQuat(1.2);
  scaled.coeffs() *= 1.0 + 1e-3;
  EXPECT_FALSE(isUnitQuaternion(scaled));
}

TEST(IsUnitQuaternion, NegatedQuaternionIsTheSameRotationAndStaysUnit) {
  Eigen::Quaterniond q = yawQuat(0.4);
  q.coeffs() *= -1.0;
  EXPECT_TRUE(isUnitQuaternion(q));
}

TEST(IsUnitQuaternion, RejectsZeroNorm) {
  EXPECT_FALSE(isUnitQuaternion(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)));
}

TEST(IsUnitQuaternion, RejectsNanAndInf) {
  EXPECT_FALSE(isUnitQuaternion(Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 1.0)));
  EXPECT_FALSE(isUnitQuaternion(Eigen::Quaterniond(1.0, std::numeric_limits<double>::infinity(), 0.0, 0.0)));
}

// Whatever the gate accepts is a rotation up to representation rounding, so the
// euler conversion, which assumes a unit quaternion, recovers its angles to that rounding.
TEST(IsUnitQuaternion, AcceptedQuaternionSurvivesTheEulerConversion) {
  Eigen::Quaterniond q(Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  q.coeffs() *= 1.0 + 9e-8;  // close to the accepted side of the default boundary

  ASSERT_LE(std::abs(q.norm() - 1.0), kDefaultQuaternionTolerance);
  ASSERT_TRUE(isUnitQuaternion(q));
  const Eigen::Vector3d zyx = eulerAnglesZyxFromQuaternion(q);
  EXPECT_NEAR(zyx(0), 0.9, 1e-6);
  EXPECT_NEAR(zyx(1), 0.3, 1e-6);
  EXPECT_NEAR(zyx(2), 0.0, 1e-6);
}

// Eigen's decomposition once unfolded, on random rotations and on norms the ingress gate accepts; q and -q give the
// same angles, and yaw is yawFromQuaternion exactly, as Agent::zeroSensorRoll documents.
TEST(EulerAnglesZyxFromQuaternion, MatchesEigensDecompositionWithoutTheFold) {
  std::mt19937 rng(7);
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  for (int i = 0; i < 20000; ++i) {
    Eigen::Quaterniond q = Eigen::Quaterniond(normal(rng), normal(rng), normal(rng), normal(rng)).normalized();
    if (i % 2 == 1) {
      q.coeffs() *= 1.0 + 9e-8 * uniform(rng);
    }
    const Eigen::Vector3d zyx = eulerAnglesZyxFromQuaternion(q);
    const Eigen::Vector3d eigen_zyx = unfoldedEigenEulerAnglesZyx(q);

    EXPECT_NEAR(angleDifference(zyx(0), eigen_zyx(0)), 0.0, 1e-13) << "q " << q.coeffs().transpose();
    EXPECT_NEAR(zyx(1), eigen_zyx(1), 1e-13) << "q " << q.coeffs().transpose();
    EXPECT_NEAR(angleDifference(zyx(2), eigen_zyx(2)), 0.0, 1e-13) << "q " << q.coeffs().transpose();
    EXPECT_EQ(zyx(0), yawFromQuaternion(q));
    EXPECT_EQ(eulerAnglesZyxFromQuaternion(Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z())), zyx);
  }
}

// The planner's level sensor (roll 0): every yaw, including just above -pi, and every pitch up to 1e-3 from straight
// up or down come back as given.
TEST(EulerAnglesZyxFromQuaternion, RecoversLevelSensorAnglesAcrossTheYawWrap) {
  std::vector<double> yaws = {-M_PI, std::nextafter(-M_PI, 0.0), -M_PI + 1e-9, -M_PI + 5e-7, -1e-15, 0.0, 1e-15, M_PI - 1e-12, M_PI};
  for (int i = -16; i <= 16; ++i) {
    yaws.push_back(i * M_PI / 16.5);
  }
  std::vector<double> pitches = {M_PI / 2.0 - 1e-3, -(M_PI / 2.0 - 1e-3)};
  for (int i = -10; i <= 10; ++i) {
    pitches.push_back(i * 0.155);
  }
  for (const double yaw : yaws) {
    for (const double pitch : pitches) {
      SCOPED_TRACE(testing::Message() << "yaw " << yaw << ", pitch " << pitch);
      const Eigen::Vector3d zyx = eulerAnglesZyxFromQuaternion(quaternionFromEulerAnglesZyx(Eigen::Vector3d(yaw, pitch, 0.0)));
      EXPECT_NEAR(angleDifference(zyx(0), yaw), 0.0, 1e-12);
      EXPECT_NEAR(zyx(1), pitch, 1e-12);
      EXPECT_NEAR(zyx(2), 0.0, 1e-12);
    }
  }
}

// Up to and at gimbal lock, for both pitch signs, the angles recompose the rotation and
// stay in range. At exact lock roll is 0 and yaw carries yaw - roll (pitch pi/2) or
// yaw + roll (pitch -pi/2); well away from it the input angles come back individually.
TEST(EulerAnglesZyxFromQuaternion, RecomposesTheRotationThroughGimbalLock) {
  for (const double sign : {1.0, -1.0}) {
    for (const double offset : {0.3, 1e-3, 1e-6, 1e-9, 1e-12, 0.0}) {
      const double pitch = sign * (M_PI / 2.0 - offset);
      for (const double yaw : {-3.0, -0.7, 0.0, 1.2, M_PI}) {
        for (const double roll : {-2.5, 0.0, 0.05, 1.9}) {
          SCOPED_TRACE(testing::Message() << "yaw " << yaw << ", pitch " << pitch << ", roll " << roll);
          const Eigen::Quaterniond q = quaternionFromEulerAnglesZyx(Eigen::Vector3d(yaw, pitch, roll));
          const Eigen::Vector3d zyx = eulerAnglesZyxFromQuaternion(q);

          EXPECT_LT(Eigen::AngleAxisd(q.inverse() * quaternionFromEulerAnglesZyx(zyx)).angle(), 1e-9);
          EXPECT_LE(std::abs(zyx(0)), M_PI);
          EXPECT_LE(std::abs(zyx(1)), M_PI / 2.0);
          EXPECT_LE(std::abs(zyx(2)), M_PI);
          if (offset == 0.0) {
            EXPECT_EQ(zyx(1), pitch);
            EXPECT_EQ(zyx(2), 0.0);
            EXPECT_NEAR(angleDifference(zyx(0), yaw - sign * roll), 0.0, 1e-12);
          } else if (offset >= 1e-3) {
            EXPECT_NEAR(angleDifference(zyx(0), yaw), 0.0, 1e-9);
            EXPECT_NEAR(zyx(1), pitch, 1e-9);
            EXPECT_NEAR(angleDifference(zyx(2), roll), 0.0, 1e-9);
          }
        }
      }
    }
  }
}

TEST(YawPitchFromQuaternion, AgreesWithTheZyxDecompositionAwayFromGimbalLock) {
  for (int yaw_index = -8; yaw_index <= 8; ++yaw_index) {
    for (int pitch_index = -5; pitch_index <= 5; ++pitch_index) {
      const double yaw = yaw_index * M_PI / 8.0;
      const double pitch = pitch_index * 0.3;  // |pitch| <= 1.5 < pi/2
      const Eigen::Quaterniond q = quaternionFromEulerAnglesZyx(Eigen::Vector3d(yaw, pitch, 0.0));

      const Eigen::Vector2d yaw_pitch = yawPitchFromQuaternion(q);
      EXPECT_NEAR(angleDifference(yaw_pitch(0), yaw), 0.0, 1e-12) << "yaw " << yaw << ", pitch " << pitch;
      EXPECT_NEAR(yaw_pitch(1), pitch, 1e-12) << "yaw " << yaw << ", pitch " << pitch;
    }
  }
}

// Close to straight down with roll present, the ZYX decomposition still recovers all
// three angles, and the forward-axis yaw and pitch match it.
TEST(YawPitchFromQuaternion, AgreesWithTheZyxDecompositionNearGimbalLockWithRoll) {
  const double pitch = M_PI / 2.0 - 1e-4;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()));

  ASSERT_TRUE(isUnitQuaternion(q));
  const Eigen::Vector3d zyx = eulerAnglesZyxFromQuaternion(q);
  EXPECT_NEAR(zyx(0), 0.7, 1e-9);
  EXPECT_NEAR(zyx(1), pitch, 1e-9);
  EXPECT_NEAR(zyx(2), 0.05, 1e-9);

  const Eigen::Vector2d yaw_pitch = yawPitchFromQuaternion(q);
  EXPECT_NEAR(yaw_pitch(0), zyx(0), 1e-9);
  EXPECT_NEAR(yaw_pitch(1), zyx(1), 1e-9);
}

TEST(ZeroSensorRoll, RemovesRollNearGimbalLock) {
  const double pitch = M_PI / 2.0 - 1e-4;
  RobotState state;
  state.setSensorOrientation(
      Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX())));

  RollNormalizer::zeroSensorRoll(&state);

  const Eigen::Quaterniond& normalized = state.getSensorOrientation();
  const Eigen::Vector2d yaw_pitch = yawPitchFromQuaternion(normalized);
  EXPECT_NEAR(yaw_pitch(0), 0.7, 1e-9);
  EXPECT_NEAR(yaw_pitch(1), pitch, 1e-9);
  // Roll-free: the sensor's lateral axis stays horizontal.
  EXPECT_NEAR((normalized * Eigen::Vector3d::UnitY()).z(), 0.0, 1e-9);
  // The two yaw readings agree exactly, as Agent::zeroSensorRoll documents.
  EXPECT_EQ(state.getSensorYaw(), state.getSensorEulerAnglesZyx()(0));
}

// Every remainder up to a full turn, both signs: ceil(|remaining| / max_step) steps of the remainder's sign, none above
// the rate, none below the pose tolerance unless the phase is a single step, summing to the remainder.
TEST(RateStep, SweepCoversEveryRemainderWithinTheRates) {
  const double min_last_step = orientationDotEpsilonToAngleThreshold(kEpsilon);
  for (const double max_step : {0.045, 0.32}) {
    for (int i = 0; 0.001 + i * 1e-4 <= 2.0 * M_PI; ++i) {
      for (const double sign : {1.0, -1.0}) {
        const double remaining = sign * (0.001 + i * 1e-4);
        std::vector<double> steps;
        for (double r = remaining; std::abs(r) > 1e-12; r -= steps.back()) {
          ASSERT_LT(steps.size(), 1000u) << "remaining " << remaining << ", max_step " << max_step;
          steps.push_back(rateStep(r, max_step, min_last_step));
          ASSERT_NE(steps.back(), 0.0) << "remaining " << remaining << ", max_step " << max_step;
        }
        ASSERT_EQ(static_cast<double>(steps.size()), std::ceil(std::abs(remaining) / max_step - kEpsilon))
            << "remaining " << remaining << ", max_step " << max_step;
        double sum = 0.0;
        for (const double step : steps) {
          ASSERT_LE(std::abs(step), max_step * (1.0 + kEpsilon)) << "remaining " << remaining << ", max_step " << max_step;
          ASSERT_TRUE(steps.size() == 1u || std::abs(step) >= min_last_step) << "remaining " << remaining << ", max_step " << max_step;
          ASSERT_GT(step * remaining, 0.0) << "remaining " << remaining << ", max_step " << max_step;
          sum += step;
        }
        ASSERT_NEAR(sum, remaining, 1e-12) << "max_step " << max_step;
      }
    }
  }
}

}  // namespace
}  // namespace open_apero
