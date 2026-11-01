/**
 * @file test_observed_viewpoints.cpp
 * @brief The observed-viewpoint predicate, the store behind it, and the HistoryData that feeds it.
 *
 * Every added pose is stored, whatever the thresholds, and while enabled a pose is observed exactly
 * when a stored viewpoint lies closer than both thresholds to it, orientation measured as one rotation.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/state/history_data.h"
#include "open_apero_core/state/observed_viewpoints.h"

namespace open_apero {
namespace {

/// @brief Round thresholds, so a test's margins read as margins rather than as arithmetic.
constexpr double kPositionThreshold = 1.0;
constexpr double kAngleThreshold = 0.4;

/// @brief Far below either threshold, far above the rounding of the distance and the angle.
constexpr double kMargin = 1e-6;

Eigen::Quaterniond yawPitch(double yaw, double pitch) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
}

Transform3d poseAt(const Eigen::Vector3d& position, double yaw, double pitch = 0.0) {
  return Transform3d{position, yawPitch(yaw, pitch)};
}

/// @brief Independent oracle for the angle ObservedViewpoints measures.
double angleBetween(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  const double angle = std::abs(Eigen::AngleAxisd(a.inverse() * b).angle());
  return std::min(angle, 2.0 * M_PI - angle);
}

class ObservedViewpointsTest : public ::testing::Test {
 protected:
  void SetUp() override { viewpoints_.setThresholds(kPositionThreshold, kAngleThreshold); }

  bool isObserved(const Eigen::Vector3d& position, double yaw, double pitch = 0.0) const {
    return viewpoints_.isObserved(position, yawPitch(yaw, pitch));
  }

  ObservedViewpoints viewpoints_;
};

TEST_F(ObservedViewpointsTest, ADisabledIndexStoresEveryPoseAndAnswersNothing) {
  ObservedViewpoints viewpoints;
  EXPECT_FALSE(viewpoints.isEnabled());
  const std::vector<Transform3d> added{poseAt({0.0, 0.0, 0.0}, 0.0), poseAt({0.0, 0.0, 0.0}, 0.0), poseAt({2.0, 0.0, 0.0}, 1.0)};
  for (const Transform3d& pose : added) {
    viewpoints.add(pose);
  }

  ASSERT_EQ(viewpoints.size(), added.size());
  for (std::size_t i = 0; i < added.size(); ++i) {
    EXPECT_TRUE(viewpoints.getViewpoint(i).isSamePose(added[i])) << "entry " << i;
  }
  EXPECT_FALSE(viewpoints.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0))) << "not even exactly at a stored pose";
  std::vector<std::size_t> near{7u};
  viewpoints.getViewpointIndicesNear({0.0, 0.0, 0.0}, 10.0, &near);
  EXPECT_TRUE(near.empty());
}

// The thresholds decide only the answers, so poses stored while disabled are answered over once enabled,
// and disabling again keeps them.
TEST_F(ObservedViewpointsTest, ThresholdsSetLaterAnswerOverThePosesStoredWhileDisabled) {
  ObservedViewpoints viewpoints;
  viewpoints.add(poseAt({0.0, 0.0, 0.0}, 0.0));
  viewpoints.add(poseAt({3.0, 0.0, 0.0}, 0.0));
  ASSERT_FALSE(viewpoints.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0)));

  viewpoints.setThresholds(kPositionThreshold, kAngleThreshold);

  ASSERT_EQ(viewpoints.size(), 2u);
  EXPECT_TRUE(viewpoints.isObserved({0.5, 0.0, 0.0}, yawPitch(0.1, 0.0)));
  EXPECT_TRUE(viewpoints.isObserved({3.5, 0.0, 0.0}, yawPitch(0.1, 0.0)));
  EXPECT_FALSE(viewpoints.isObserved({1.5, 0.0, 0.0}, yawPitch(0.0, 0.0)));
  std::vector<std::size_t> near;
  viewpoints.getViewpointIndicesNear({0.0, 0.0, 0.0}, 10.0, &near);
  EXPECT_EQ(near, (std::vector<std::size_t>{0u, 1u}));

  viewpoints.setThresholds(0.0, 0.0);

  EXPECT_EQ(viewpoints.size(), 2u);
  EXPECT_FALSE(viewpoints.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0)));
  viewpoints.getViewpointIndicesNear({0.0, 0.0, 0.0}, 10.0, &near);
  EXPECT_TRUE(near.empty());
}

// One threshold alone would match on position alone or on orientation alone, and a position threshold
// of at most 0.01 reads as an unbounded radius, so the index refuses all three and stays as it was.
TEST_F(ObservedViewpointsTest, RefusesAnInvalidThresholdPair) {
  ObservedViewpoints viewpoints;
  viewpoints.setThresholds(kPositionThreshold, kAngleThreshold);
  viewpoints.add(poseAt({0.0, 0.0, 0.0}, 0.0));
  ASSERT_EQ(viewpoints.size(), 1u);

  EXPECT_THROW(viewpoints.setThresholds(kPositionThreshold, 0.0), std::invalid_argument);
  EXPECT_THROW(viewpoints.setThresholds(0.0, kAngleThreshold), std::invalid_argument);
  EXPECT_THROW(viewpoints.setThresholds(0.01, kAngleThreshold), std::invalid_argument);
  EXPECT_DOUBLE_EQ(viewpoints.getPositionThreshold(), kPositionThreshold);
  EXPECT_DOUBLE_EQ(viewpoints.getAngleThreshold(), kAngleThreshold);
  EXPECT_EQ(viewpoints.size(), 1u);

  std::string why;
  EXPECT_FALSE(ObservedViewpoints::isValidThresholdPair(kPositionThreshold, 0.0, &why));
  EXPECT_FALSE(why.empty());
  EXPECT_TRUE(ObservedViewpoints::isValidThresholdPair(0.0, 0.0));
  EXPECT_NO_THROW(viewpoints.setThresholds(0.0, 0.0));
  EXPECT_FALSE(viewpoints.isEnabled());
  EXPECT_EQ(viewpoints.size(), 1u);
}

TEST_F(ObservedViewpointsTest, ObservesAPoseOnlyWithinBothThresholds) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0));

  EXPECT_TRUE(isObserved({0.9, 0.0, 0.0}, 0.0));
  EXPECT_TRUE(isObserved({0.9, 0.0, 0.0}, 0.39));

  EXPECT_FALSE(isObserved({1.1, 0.0, 0.0}, 0.0)) << "too far away, however well aligned";
  EXPECT_FALSE(isObserved({0.9, 0.0, 0.0}, 0.41)) << "close enough, but facing elsewhere";
  EXPECT_FALSE(isObserved({1.1, 0.0, 0.0}, 0.41));
}

TEST_F(ObservedViewpointsTest, PitchCountsAsMuchAsYaw) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0, 0.0));

  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, 0.0, 0.39));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.0, 0.41)) << "same position and heading, pitched past the angle";
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.0, -0.41)) << "looking up counts the same as looking down";

  // A yaw and a pitch each inside the angle compose to one rotation outside it.
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, 0.3, 0.0));
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, 0.0, 0.3));
  ASSERT_GT(angleBetween(yawPitch(0.0, 0.0), yawPitch(0.3, 0.3)), kAngleThreshold);
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.3, 0.3));
}

TEST_F(ObservedViewpointsTest, ReadsAQuaternionAndItsNegationAsOneOrientation) {
  ObservedViewpoints viewpoints;
  viewpoints.setThresholds(kPositionThreshold, 1e-3);
  const Eigen::Quaterniond orientation = yawPitch(2.0, 0.4);
  viewpoints.add(Transform3d{Eigen::Vector3d::Zero(), orientation});

  const Eigen::Quaterniond negated(-orientation.w(), -orientation.x(), -orientation.y(), -orientation.z());
  EXPECT_TRUE(viewpoints.isObserved(Eigen::Vector3d::Zero(), negated));
}

// The thresholds are exact and strict: a hair inside both is observed, a hair beyond either is not.
TEST_F(ObservedViewpointsTest, ObservesExactlyWithinTheThresholds) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0, 0.0));

  EXPECT_TRUE(isObserved({kPositionThreshold - kMargin, 0.0, 0.0}, kAngleThreshold - kMargin));
  EXPECT_TRUE(isObserved({0.0, kPositionThreshold - kMargin, 0.0}, 0.0, kAngleThreshold - kMargin));

  // Position, as one Euclidean distance.
  EXPECT_FALSE(isObserved({kPositionThreshold + kMargin, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(isObserved({kPositionThreshold, 0.0, 0.0}, 0.0)) << "the threshold itself is outside";
  const double diagonal = kPositionThreshold / std::sqrt(2.0);
  EXPECT_TRUE(isObserved({diagonal - kMargin, diagonal - kMargin, 0.0}, 0.0));
  EXPECT_FALSE(isObserved({diagonal + kMargin, diagonal + kMargin, 0.0}, 0.0)) << "inside on each axis, beyond as a distance";

  // Yaw alone.
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, kAngleThreshold - kMargin));
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, -(kAngleThreshold - kMargin)));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, kAngleThreshold + kMargin));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, -(kAngleThreshold + kMargin)));

  // Pitch alone.
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, 0.0, kAngleThreshold - kMargin));
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, 0.0, -(kAngleThreshold - kMargin)));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.0, kAngleThreshold + kMargin));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.0, -(kAngleThreshold + kMargin)));
}

// A caller's angle threshold replaces the configured one; the position threshold and the enabled state stay.
TEST_F(ObservedViewpointsTest, ACallersAngleThresholdReplacesTheConfiguredOne) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0, 0.0));
  constexpr double kTighter = 0.1;

  EXPECT_TRUE(viewpoints_.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, kTighter - kMargin), kTighter));
  EXPECT_FALSE(viewpoints_.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, kTighter + kMargin), kTighter));
  EXPECT_FALSE(viewpoints_.isObserved({0.0, 0.0, 0.0}, yawPitch(kTighter + kMargin, 0.0), kTighter));
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, 0.0, kTighter + kMargin)) << "the configured angle still admits it";
  EXPECT_TRUE(viewpoints_.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, kAngleThreshold + kMargin), 2.0 * kAngleThreshold))
      << "a looser angle is the caller's too";

  EXPECT_TRUE(viewpoints_.isObserved({kPositionThreshold - kMargin, 0.0, 0.0}, yawPitch(0.0, 0.0), kTighter));
  EXPECT_FALSE(viewpoints_.isObserved({kPositionThreshold + kMargin, 0.0, 0.0}, yawPitch(0.0, 0.0), kTighter))
      << "the position threshold stays the configured one";

  ObservedViewpoints disabled;
  disabled.add(poseAt({0.0, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(disabled.isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0), kTighter)) << "disabled answers nothing";
}

TEST_F(ObservedViewpointsTest, StoresEveryAddedPose) {
  const std::vector<Transform3d> added{poseAt({0.0, 0.0, 0.0}, 0.0), poseAt({0.0, 0.0, 0.0}, 0.0), poseAt({1e-3, 0.0, 0.0}, 1e-3),
                                       poseAt({0.0, 0.0, 0.0}, 0.0, 1e-3)};
  for (const Transform3d& pose : added) {
    viewpoints_.add(pose);
  }

  ASSERT_EQ(viewpoints_.size(), added.size()) << "however close, a repeat included";
  for (std::size_t i = 0; i < added.size(); ++i) {
    EXPECT_TRUE(viewpoints_.getViewpoint(i).isSamePose(added[i])) << "entry " << i;
  }
}

// A sweep in 0.15 rad steps: every yaw it passed through is observed, and coverage ends one
// threshold past its last pose.
TEST_F(ObservedViewpointsTest, ASweepIsObservedUpToOneThresholdPastItsEnds) {
  constexpr int kNumSteps = 8;
  constexpr double kStep = 0.15;
  for (int i = 0; i < kNumSteps; ++i) {
    viewpoints_.add(poseAt({0.0, 0.0, 0.0}, kStep * i));
  }
  const double last_yaw = kStep * (kNumSteps - 1);

  EXPECT_EQ(viewpoints_.size(), static_cast<std::size_t>(kNumSteps));
  for (int i = 0; i < kNumSteps; ++i) {
    EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, kStep * i)) << "yaw " << kStep * i;
  }
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, last_yaw + kAngleThreshold - kMargin));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, last_yaw + kAngleThreshold + kMargin));
  EXPECT_TRUE(isObserved({0.0, 0.0, 0.0}, -(kAngleThreshold - kMargin)));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, -(kAngleThreshold + kMargin)));
}

// Any stored viewpoint counts, not only the latest one or the ones near it in time.
TEST_F(ObservedViewpointsTest, AnEarlierViewpointStillObservesWhatOnlyItCovers) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0));
  viewpoints_.add(poseAt({5.0, 0.0, 0.0}, 0.0));
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 2.0));

  EXPECT_TRUE(isObserved({0.5, 0.0, 0.0}, 0.1)) << "only the first viewpoint covers this pose";
  EXPECT_FALSE(isObserved({0.5, 0.0, 0.0}, 1.0));
}

// The predicate is exactly "some added pose lies closer than both thresholds", checked against a
// brute-force oracle over every added pose on a grid of query poses.
TEST_F(ObservedViewpointsTest, AgreesWithABruteForceOracleOverEveryAddedPose) {
  const std::vector<Transform3d> added{poseAt({0.0, 0.0, 0.0}, 0.0), poseAt({0.0, 0.0, 0.0}, 0.3), poseAt({0.3, 0.0, 0.0}, 0.1),
                                       poseAt({2.0, 0.0, 0.0}, 1.0)};
  for (const Transform3d& pose : added) {
    viewpoints_.add(pose);
  }

  int num_observed = 0;
  int num_unobserved = 0;
  for (const double x : {-1.5, -0.5, 0.2, 0.4, 1.2, 2.4, 3.5}) {
    for (const double y : {-1.0, 0.0, 0.3, 0.8}) {
      for (const double yaw : {0.0, 0.15, 0.55, 1.0, 2.0, 3.0}) {
        const Eigen::Vector3d position(x, y, 0.0);
        bool expected = false;
        for (const Transform3d& pose : added) {
          const double distance = (position - pose.position).norm();
          const double angle = angleBetween(yawPitch(yaw, 0.0), pose.orientation);
          // Off the thresholds, so rounding cannot decide a case.
          ASSERT_GT(std::abs(distance - kPositionThreshold), 1e-3);
          ASSERT_GT(std::abs(angle - kAngleThreshold), 1e-3);
          expected |= distance < kPositionThreshold && angle < kAngleThreshold;
        }

        EXPECT_EQ(isObserved(position, yaw), expected) << "at [" << position.transpose() << "] yaw " << yaw;
        ++(expected ? num_observed : num_unobserved);
      }
    }
  }

  // Both answers have to have been exercised, or the loop proved nothing.
  EXPECT_GT(num_observed, 0);
  EXPECT_GT(num_unobserved, 0);
}

TEST_F(ObservedViewpointsTest, ViewpointsAreAppendOnlyAndClearEmptiesThem) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0));
  viewpoints_.add(poseAt({2.0, 0.0, 0.0}, 0.0));
  const std::vector<Transform3d> before{viewpoints_.getViewpoint(0), viewpoints_.getViewpoint(1)};

  viewpoints_.add(poseAt({4.0, 0.0, 0.0}, 0.0));

  ASSERT_EQ(viewpoints_.size(), 3u);
  for (std::size_t i = 0; i < before.size(); ++i) {
    EXPECT_TRUE(viewpoints_.getViewpoint(i).isSamePose(before[i])) << "entry " << i << " moved";
  }
  EXPECT_NEAR(viewpoints_.getViewpoint(2).position.x(), 4.0, 1e-12);

  viewpoints_.clear();

  EXPECT_EQ(viewpoints_.size(), 0u);
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.0));
  EXPECT_TRUE(viewpoints_.isEnabled()) << "the thresholds outlive the viewpoints";

  // The index over the emptied store is fresh: a re-added pose is found, and found once.
  viewpoints_.add(poseAt({4.0, 0.0, 0.0}, 0.0));
  EXPECT_EQ(viewpoints_.size(), 1u);
  EXPECT_TRUE(isObserved({4.0, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(isObserved({0.0, 0.0, 0.0}, 0.0));
}

// A radius query by position alone, for consumers that test what a viewpoint had in view rather than how it was aimed.
TEST_F(ObservedViewpointsTest, ViewpointIndicesNearListsTheStoredViewpointsWithinTheRadiusInAscendingOrder) {
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 0.0));
  viewpoints_.add(poseAt({3.0, 0.0, 0.0}, 0.0));
  viewpoints_.add(poseAt({0.0, 0.0, 0.0}, 2.0, 0.9));
  viewpoints_.add(poseAt({0.0, 1.5, 0.0}, 0.0));

  std::vector<std::size_t> near{7u};
  viewpoints_.getViewpointIndicesNear({0.2, 0.0, 0.0}, 0.5, &near);
  EXPECT_EQ(near, (std::vector<std::size_t>{0u, 2u})) << "both poses at the origin, however aimed, and nothing else";

  viewpoints_.getViewpointIndicesNear({0.2, 0.0, 0.0}, 0.1, &near);
  EXPECT_TRUE(near.empty());
  viewpoints_.getViewpointIndicesNear({0.0, 0.0, 0.0}, 10.0, &near);
  EXPECT_EQ(near, (std::vector<std::size_t>{0u, 1u, 2u, 3u}));

  ObservedViewpoints disabled;
  near.assign(1, 0u);
  disabled.getViewpointIndicesNear({0.0, 0.0, 0.0}, 10.0, &near);
  EXPECT_TRUE(near.empty());
}

TEST_F(ObservedViewpointsTest, NewThresholdsKeepTheStoredViewpointsAndAnswerOverThem) {
  const Transform3d stored = poseAt({0.0, 0.0, 0.0}, 0.0);
  viewpoints_.add(stored);
  ASSERT_TRUE(isObserved({0.5, 0.0, 0.0}, 0.3));

  viewpoints_.setThresholds(0.2, 0.2);

  ASSERT_EQ(viewpoints_.size(), 1u);
  EXPECT_TRUE(viewpoints_.getViewpoint(0).isSamePose(stored));
  EXPECT_DOUBLE_EQ(viewpoints_.getPositionThreshold(), 0.2);
  EXPECT_DOUBLE_EQ(viewpoints_.getAngleThreshold(), 0.2);
  EXPECT_FALSE(isObserved({0.5, 0.0, 0.0}, 0.0)) << "beyond the new position threshold";
  EXPECT_FALSE(isObserved({0.1, 0.0, 0.0}, 0.3)) << "beyond the new angle threshold";
  EXPECT_TRUE(isObserved({0.1, 0.0, 0.0}, 0.1));
}

TEST(HistoryDataObservedViewpoints, AddObservedPoseFeedsTheIndex) {
  HistoryData history;
  history.getObservedViewpoints().setThresholds(kPositionThreshold, kAngleThreshold);

  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  history.addObservedPose(poseAt({0.2, 0.0, 0.0}, 0.05));
  history.addObservedPose(poseAt({3.0, 0.0, 0.0}, 0.0));
  history.addObservedPose(poseAt({3.0, 0.0, 0.0}, 0.0));

  EXPECT_EQ(history.getObservedViewpoints().size(), 3u) << "the record drops the consecutive repeat, and only that";
  EXPECT_TRUE(history.getObservedViewpoints().isObserved({3.0, 0.0, 0.0}, yawPitch(0.0, 0.0)));
  EXPECT_FALSE(history.getObservedViewpoints().isObserved({1.5, 0.0, 0.0}, yawPitch(0.0, 0.0)));
}

// Thresholds set after the first observations answer over them.
TEST(HistoryDataObservedViewpoints, TheRecordKeepsEveryPoseWhateverItsThresholds) {
  HistoryData history;
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));

  ASSERT_EQ(history.getObservedViewpoints().size(), 1u) << "a disabled record drops the consecutive repeat too";
  EXPECT_FALSE(history.getObservedViewpoints().isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0)));

  history.getObservedViewpoints().setThresholds(kPositionThreshold, kAngleThreshold);

  EXPECT_EQ(history.getObservedViewpoints().size(), 1u);
  EXPECT_TRUE(history.getObservedViewpoints().isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0)));
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  EXPECT_EQ(history.getObservedViewpoints().size(), 1u) << "the repeat check spans the change of thresholds";
}

TEST(HistoryDataObservedViewpoints, ResetEpisodeClearsTheIndexAndKeepsItsThresholds) {
  HistoryData history;
  history.getObservedViewpoints().setThresholds(kPositionThreshold, kAngleThreshold);
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  ASSERT_EQ(history.getObservedViewpoints().size(), 1u);

  history.resetEpisode();

  EXPECT_EQ(history.getObservedViewpoints().size(), 0u);
  EXPECT_FALSE(history.getObservedViewpoints().isObserved({0.0, 0.0, 0.0}, yawPitch(0.0, 0.0)));
  EXPECT_DOUBLE_EQ(history.getObservedViewpoints().getPositionThreshold(), kPositionThreshold);
  EXPECT_DOUBLE_EQ(history.getObservedViewpoints().getAngleThreshold(), kAngleThreshold);

  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  EXPECT_EQ(history.getObservedViewpoints().size(), 1u) << "the next episode is indexed from empty";
}

}  // namespace
}  // namespace open_apero
