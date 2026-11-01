/**
 * @file test_history_data.cpp
 * @brief The observation record's contract: what it holds and when it grows.
 *
 * Every observed pose comes from a frame the map reported integrating, and the record only grows
 * until the episode resets, whatever its thresholds.
 */

#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "open_apero_core/geometry/transform3d.h"
#include "open_apero_core/graph/edge.h"
#include "open_apero_core/state/history_data.h"
#include "open_apero_core/state/observed_viewpoints.h"
#include "open_apero_core/state/robot_state.h"

namespace open_apero {
namespace {

Transform3d poseAt(const Eigen::Vector3d& position, double yaw) {
  return Transform3d{position, Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))};
}

RobotState stateAt(const Eigen::Vector3d& position, double yaw) {
  RobotState state;
  state.setSensorPosition(position);
  state.setSensorOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
  return state;
}

Edge edgeThrough(const RobotTrajectory& trajectory) {
  Edge edge;
  edge.trajectory = trajectory;
  edge.cost = 1.0;
  return edge;
}

TEST(HistoryDataObservations, StartsEmpty) {
  HistoryData history;
  EXPECT_EQ(history.getObservedViewpoints().size(), 0u);
}

// The thresholds are off by default, and the record keeps every pose regardless.
TEST(HistoryDataObservations, KeepsEveryConfirmedPoseInOrder) {
  HistoryData history;
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  history.addObservedPose(poseAt({1.0, 0.0, 0.0}, 0.5));
  history.addObservedPose(poseAt({2.0, 0.0, 0.0}, 1.0));

  const ObservedViewpoints& observed = history.getObservedViewpoints();
  ASSERT_FALSE(observed.isEnabled());
  ASSERT_EQ(observed.size(), 3u);
  EXPECT_NEAR(observed.getViewpoint(0).position.x(), 0.0, 1e-12);
  EXPECT_NEAR(observed.getViewpoint(1).position.x(), 1.0, 1e-12);
  EXPECT_NEAR(observed.getViewpoint(2).position.x(), 2.0, 1e-12);
}

// A vetoed follow-up command leaves the cursor on the waypoint that just settled,
// so the same pose can be confirmed twice; it is recorded once.
TEST(HistoryDataObservations, CollapsesAPoseConfirmedTwiceInARow) {
  HistoryData history;
  history.addObservedPose(poseAt({1.0, 2.0, 3.0}, 0.4));
  history.addObservedPose(poseAt({1.0, 2.0, 3.0}, 0.4));
  EXPECT_EQ(history.getObservedViewpoints().size(), 1u);

  // Same position, different heading: a genuinely different viewpoint.
  history.addObservedPose(poseAt({1.0, 2.0, 3.0}, 1.4));
  EXPECT_EQ(history.getObservedViewpoints().size(), 2u);

  // And returning to the first pose later is a new entry: only the immediately
  // preceding duplicate is the re-settle artifact.
  history.addObservedPose(poseAt({1.0, 2.0, 3.0}, 0.4));
  ASSERT_EQ(history.getObservedViewpoints().size(), 3u);
  EXPECT_TRUE(history.getObservedViewpoints().getViewpoint(2).isSamePose(poseAt({1.0, 2.0, 3.0}, 0.4)));
}

// The two records answer different questions. Executing an edge says where the
// robot went; only an explicit confirmation says where it looked from.
TEST(HistoryDataObservations, ExecutingAnEdgeObservesNothingByItself) {
  HistoryData history;
  history.addVisitedEdge(edgeThrough({stateAt({0.0, 0.0, 0.0}, 0.0), stateAt({1.0, 0.0, 0.0}, 0.0)}));

  EXPECT_EQ(history.getObservedViewpoints().size(), 0u);
  EXPECT_EQ(history.size(), 2);
}

// A consumer bookmarks how many entries it has read, so the record must never
// shrink except at an episode boundary.
TEST(HistoryDataObservations, GrowsMonotonicallyWithinAnEpisode) {
  HistoryData history;
  std::size_t previous = 0;
  for (int i = 0; i < 5; ++i) {
    history.addObservedPose(poseAt({static_cast<double>(i), 0.0, 0.0}, 0.0));
    history.addVisitedEdge(
        edgeThrough({stateAt({static_cast<double>(i), 0.0, 0.0}, 0.0), stateAt({static_cast<double>(i) + 1.0, 0.0, 0.0}, 0.0)}));
    EXPECT_GE(history.getObservedViewpoints().size(), previous);
    previous = history.getObservedViewpoints().size();
  }
  EXPECT_EQ(history.getObservedViewpoints().size(), 5u);
}

TEST(HistoryDataObservations, ResetEpisodeForgetsEveryObservation) {
  HistoryData history;
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  history.addVisitedEdge(edgeThrough({stateAt({0.0, 0.0, 0.0}, 0.0), stateAt({1.0, 0.0, 0.0}, 0.0)}));
  ASSERT_EQ(history.getObservedViewpoints().size(), 1u);

  history.resetEpisode();

  EXPECT_EQ(history.getObservedViewpoints().size(), 0u);
  EXPECT_EQ(history.size(), 0);

  // The repeat check reads the cleared record, so the last pose of the old episode is recorded again.
  history.addObservedPose(poseAt({0.0, 0.0, 0.0}, 0.0));
  EXPECT_EQ(history.getObservedViewpoints().size(), 1u);
}

// Orientation is half of what makes a viewpoint, so a record that kept only the
// position would call two opposite views of a room the same observation.
TEST(HistoryDataObservations, KeepsTheFullPoseNotJustThePosition) {
  HistoryData history;
  const Eigen::Quaterniond orientation(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  history.addObservedPose(Transform3d{Eigen::Vector3d(1.0, 2.0, 3.0), orientation});

  ASSERT_EQ(history.getObservedViewpoints().size(), 1u);
  EXPECT_TRUE(history.getObservedViewpoints().getViewpoint(0).orientation.isApprox(orientation));
  EXPECT_TRUE(history.getObservedViewpoints().getViewpoint(0).position.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

}  // namespace
}  // namespace open_apero
