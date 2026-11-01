#pragma once

/**
 * @file ros_agent_metrics.h
 * @brief RosAgent's metrics: AgentMetrics plus the ROS-time fields.
 */

#include <open_apero_core/agent/agent_metrics.h>

namespace open_apero {
namespace ros {

/**
 * @brief Extends AgentMetrics with ROS-specific timing fields.
 */
struct RosAgentMetrics : AgentMetrics {
  double replan_time_ros_s = 0.0;  ///< ROS-time elapsed since the previous replan call [s] (ros::Time::now() delta).
  double ros_callback_s = 0.0;     ///< Accumulated time spent inside ros::spinOnce() during the cycle.
  double ros_time_s = 0.0;         ///< ROS clock time at this cycle [s] (ros::Time::now()); row identity, not core-visible.

  void resetCycle() override {
    AgentMetrics::resetCycle();
    replan_time_ros_s = 0.0;
    ros_callback_s = 0.0;
  }

  void declareFields(MetricsFields& fields) const override {
    AgentMetrics::declareFields(fields);
    fields.add("ReplanTimeRos", &replan_time_ros_s, "s");
    fields.add("RosCallbacks", &ros_callback_s, "s");
    fields.add("RosTimeSec", &ros_time_s, "s");
  }
};

}  // namespace ros
}  // namespace open_apero
