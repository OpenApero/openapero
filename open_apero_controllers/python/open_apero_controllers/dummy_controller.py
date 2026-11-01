"""Dummy controller that replays Open Apero agent trajectories through TF broadcasts."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from open_apero_msgs.msg import CommandResult, RobotState, RobotTrajectory
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header


class DummyController:
    """Replays planned trajectories through TF broadcasts, bypassing physics."""

    def __init__(self):
        rospy.init_node("dummy_controller_node")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_frame")
        self.camera_optical_frame = rospy.get_param("~camera_optical_frame", "camera_optical_frame")
        self.speedup_factor = rospy.get_param("~speedup_factor", 1.0)
        self.publish_camera_transform = rospy.get_param("~publish_camera_transform", "none")

        self.initial_robot_state = RobotState()
        self.initial_robot_state.base_transform.translation.x = rospy.get_param(
            "/initial_position_x"
        )
        self.initial_robot_state.base_transform.translation.y = rospy.get_param(
            "/initial_position_y"
        )
        self.initial_robot_state.base_transform.translation.z = rospy.get_param(
            "/initial_position_z"
        )
        self.initial_robot_state.base_transform.rotation.x = rospy.get_param(
            "/initial_quaternion_x"
        )
        self.initial_robot_state.base_transform.rotation.y = rospy.get_param(
            "/initial_quaternion_y"
        )
        self.initial_robot_state.base_transform.rotation.z = rospy.get_param(
            "/initial_quaternion_z"
        )
        self.initial_robot_state.base_transform.rotation.w = rospy.get_param(
            "/initial_quaternion_w"
        )

        self.initial_robot_state.sensor_transform = self.initial_robot_state.base_transform

        self.robot_state_pub = rospy.Publisher("robot_state", RobotState, queue_size=1)
        self.command_result_pub = rospy.Publisher("command/result", CommandResult, queue_size=1)
        self.camera_optical_frame_transform_pub = rospy.Publisher(
            "/camera_transform", TransformStamped, queue_size=1
        )

        self.tf_br = tf2_ros.TransformBroadcaster()

        # Convert ROS optical axes (+Z forward, +X right, +Y down) to the
        # base/camera convention (+X forward, +Y left, +Z up).
        self.R_camera_optical = R.from_euler("ZYX", [-math.pi / 2.0, 0.0, -math.pi / 2.0])

        self.publish_initial_state_timer = rospy.Timer(
            rospy.Duration(0.1), self.publish_initial_state_callback
        )
        self.traj_sub = rospy.Subscriber(
            "/command/trajectory", RobotTrajectory, self.trajectory_callback
        )

    def publish_initial_state_callback(self, event):
        """Timer callback that publishes the initial pose until a trajectory is received."""
        self.publish_robot_state(self.initial_robot_state)
        rospy.loginfo_once(
            "[DummyController] Publishing the initial pose until the first trajectory."
        )

    def trajectory_callback(self, msg: RobotTrajectory):
        """Handle incoming trajectory by sequentially publishing each waypoint pose."""
        if self.publish_initial_state_timer.is_alive():
            self.publish_initial_state_timer.shutdown()

        current_time = 0.0
        result_header = None
        for robot_state in msg.points:
            target_time = robot_state.time_from_start.to_sec() / self.speedup_factor
            rospy.sleep(target_time - current_time)
            result_header = self.publish_robot_state(robot_state)
            current_time = target_time

        if result_header is not None:
            command_result = CommandResult()
            command_result.header = result_header
            command_result.command_sequence = msg.command_sequence
            command_result.state = msg.points[-1]
            self.command_result_pub.publish(command_result)

    def publish_robot_state(self, robot_state: RobotState) -> Header:
        """Broadcast TF and publish a robot state, returning their shared stamped header."""
        header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)

        rot = robot_state.sensor_transform.rotation
        optical_quat = (R.from_quat([rot.x, rot.y, rot.z, rot.w]) * self.R_camera_optical).as_quat()
        optical_transform = TransformStamped()
        optical_transform.transform.translation = robot_state.sensor_transform.translation
        optical_transform.transform.rotation.x = optical_quat[0]
        optical_transform.transform.rotation.y = optical_quat[1]
        optical_transform.transform.rotation.z = optical_quat[2]
        optical_transform.transform.rotation.w = optical_quat[3]

        self.tf_br.sendTransform(
            [
                self._make_transform(header, self.base_frame, robot_state.base_transform),
                self._make_transform(header, self.camera_frame, robot_state.sensor_transform),
                self._make_transform(
                    header, self.camera_optical_frame, optical_transform.transform
                ),
            ]
        )

        self.robot_state_pub.publish(robot_state)

        # Habitat renders a frame only when it receives this transform.
        if self.publish_camera_transform == "all":
            optical_transform.header = header
            optical_transform.child_frame_id = self.camera_optical_frame
            self.camera_optical_frame_transform_pub.publish(optical_transform)

        return header

    def _make_transform(self, header: Header, child_frame_id: str, transform) -> TransformStamped:
        ts = TransformStamped()
        ts.header = header
        ts.child_frame_id = child_frame_id
        ts.transform = transform
        return ts


def main():
    DummyController()
    rospy.spin()


if __name__ == "__main__":
    main()
