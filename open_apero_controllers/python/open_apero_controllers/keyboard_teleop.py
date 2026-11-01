"""Command the agent interactively with single-waypoint trajectories.

Keys: w/s move forward/back along the heading, a/d turn left/right, r/f move up/down; ESC or
Ctrl-C quits.
The start pose is the experiment's /initial_position_* and /initial_quaternion_* (yaw only),
defaulting to (4, 4, 1) facing +x; the trajectory frame is ~map_frame (default "map").
"""

import math
import os
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Transform
from open_apero_msgs.msg import RobotState, RobotTrajectory
from scipy.spatial.transform import Rotation as R

STEP_SIZE = 0.5  # Forward/back step along the heading [m].
ROTATION_STEP = 0.1  # Turn step [rad].
HEIGHT_STEP = 0.2  # Up/down step [m].


class KeyboardTeleop:
    """Interactive keyboard controller that publishes single-waypoint trajectories."""

    def __init__(self):
        rospy.init_node("keyboard_teleop_node")
        self.traj_pub = rospy.Publisher("/command/trajectory", RobotTrajectory, queue_size=1)
        self.rate = rospy.Rate(10)
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.x = rospy.get_param("/initial_position_x", 4.0)
        self.y = rospy.get_param("/initial_position_y", 4.0)
        self.z = rospy.get_param("/initial_position_z", 1.0)
        quaternion = [
            rospy.get_param("/initial_quaternion_" + axis, value)
            for axis, value in (("x", 0.0), ("y", 0.0), ("z", 0.0), ("w", 1.0))
        ]
        self.yaw = R.from_quat(quaternion).as_euler("ZYX")[0]

    def get_key(self):
        """Capture a single keypress; an escape sequence (e.g. an arrow key) is returned whole."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = os.read(fd, 1)
            if key == b"\x1b":
                # The rest of an escape sequence is already pending; a lone ESC has none.
                while select.select([fd], [], [], 0.0)[0]:
                    key += os.read(fd, 1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key.decode(errors="replace")

    def publish_trajectory(self):
        """Publish the current pose as a single-point RobotTrajectory."""
        traj = RobotTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = self.map_frame

        point = RobotState()

        transform = Transform()
        transform.translation.x = self.x
        transform.translation.y = self.y
        transform.translation.z = self.z

        quat = R.from_euler("z", self.yaw).as_quat()
        transform.rotation.x = quat[0]
        transform.rotation.y = quat[1]
        transform.rotation.z = quat[2]
        transform.rotation.w = quat[3]

        point.base_transform = transform
        point.sensor_transform = transform  # This teleop assumes a base-aligned sensor.
        point.time_from_start = rospy.Duration(0.1)

        traj.points.append(point)
        self.traj_pub.publish(traj)

    def run(self):
        """Main loop: read key presses and publish corresponding trajectory commands."""
        print("w/s: forward/back, a/d: turn left/right, r/f: up/down, ESC or Ctrl-C: quit.")
        while not rospy.is_shutdown():
            key = self.get_key()

            if key in ("\x1b", "\x03"):  # A lone ESC or Ctrl-C.
                break
            elif key == "w":
                self.x += STEP_SIZE * math.cos(self.yaw)
                self.y += STEP_SIZE * math.sin(self.yaw)
            elif key == "s":
                self.x -= STEP_SIZE * math.cos(self.yaw)
                self.y -= STEP_SIZE * math.sin(self.yaw)
            elif key == "a":
                self.yaw += ROTATION_STEP
            elif key == "d":
                self.yaw -= ROTATION_STEP
            elif key == "r":  # rise
                self.z += HEIGHT_STEP
            elif key == "f":  # fall
                self.z -= HEIGHT_STEP

            self.publish_trajectory()
            self.rate.sleep()


def main():
    teleop = KeyboardTeleop()
    try:
        teleop.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
