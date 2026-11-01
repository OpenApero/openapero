"""Render ROS camera poses in Habitat for Open Apero.

habitat_sim is imported only where the simulator is built, so this module, and the pure functions
``resolve_scene``, ``process_depth`` and ``camera_intrinsics``, import without it.
"""

import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from queue import Empty, Queue

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image as ROS_Image
from std_msgs.msg import String

# An HM3D id is `<5 digits>-<11 chars>`, e.g. 00014-nYYcLpSzihC.
HM3D_SCENE_ID = re.compile(r"\d{5}-(\w{11})")


def resolve_scene(scene_id, mesh_folder):
    """Return the scene Habitat loads for a scene id.

    Args:
        scene_id: An HM3D id, a dataset scene handle (e.g. HSSD's ``102816036``) or a path.
        mesh_folder: Folder holding one ``<id>/<11 chars>.glb`` subfolder per HM3D scene.

    Returns:
        ``<mesh_folder>/<id>/<11 chars>.glb`` for an HM3D id; any other value is passed to
        Habitat unchanged.
    """
    match = HM3D_SCENE_ID.fullmatch(scene_id)
    if match is None:
        return scene_id
    return f"{mesh_folder}/{scene_id}/{match.group(1)}.glb"


def process_depth(depth, min_depth, max_depth, far_sentinel):
    """Replace the invalid and out-of-range readings of a planar depth image.

    Args:
        depth: Depth image [m], planar z.
        min_depth: Readings below it are replaced with 0 (no return) [m].
        max_depth: Readings above it are replaced with ``far_sentinel`` [m].
        far_sentinel: Value written for readings beyond ``max_depth`` [m].

    Returns:
        A float32 copy of ``depth`` with NaN and infinite readings replaced with 0, then readings
        above ``max_depth`` with ``far_sentinel``, then readings below ``min_depth`` with 0.
    """
    processed = np.array(depth, dtype=np.float32)
    processed[~np.isfinite(processed)] = 0.0
    processed[processed > max_depth] = far_sentinel
    processed[processed < min_depth] = 0.0
    return processed


def camera_intrinsics(width, height, hfov_deg):
    """Return the pinhole intrinsics of a Habitat camera.

    Args:
        width: Image width [px].
        height: Image height [px].
        hfov_deg: Horizontal field of view [deg].

    Returns:
        ``(fx, fy, cx, cy)`` [px]. Habitat renders square pixels, so ``fy == fx``; the principal
        point is the image center in 0-indexed pixel coordinates.
    """
    fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    return fx, fx, (width - 1) / 2.0, (height - 1) / 2.0


@dataclass(frozen=True)
class CameraConfig:
    """Sensor geometry and which sensors to render."""

    width: int
    height: int
    hfov: float
    color_sensor: bool
    depth_sensor: bool

    @classmethod
    def from_rosparams(cls):
        """Build the camera configuration from private rosparams."""
        return cls(
            width=rospy.get_param("~width", 480),
            height=rospy.get_param("~height", 640),
            hfov=rospy.get_param("~hfov", 42.0),
            color_sensor=rospy.get_param("~color_sensor", True),
            depth_sensor=rospy.get_param("~depth_sensor", True),
        )


@dataclass(frozen=True)
class DepthConfig:
    """Depth post-processing range."""

    min_depth: float
    max_depth: float
    far_depth_sentinel: float

    @classmethod
    def from_rosparams(cls):
        """Build the depth configuration from private rosparams."""
        return cls(
            min_depth=rospy.get_param("~min_depth", 0.0),
            max_depth=rospy.get_param("~max_depth", 5.0),
            far_depth_sentinel=rospy.get_param("~far_depth_sentinel", 6.0),
        )


def make_habitat_cfg(camera, scene, scene_dataset_config_file):
    """Create a Habitat simulator configuration.

    Args:
        camera: CameraConfig describing sensor geometry and which sensors to
            enable.
        scene: Path to the scene asset (``.glb``) or a scene identifier.
        scene_dataset_config_file: Path to the dataset config Habitat uses to
            resolve scene assets and semantic metadata.

    Returns:
        A habitat_sim.Configuration object ready to initialize a simulator.
    """
    import habitat_sim.agent

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.frustum_culling = True
    sim_cfg.gpu_device_id = 0
    sim_cfg.scene_id = scene
    sim_cfg.scene_dataset_config_file = scene_dataset_config_file

    sensor_specs = []

    def create_camera_spec(**kw_args):
        """Create a camera sensor specification with the given overrides.

        Args:
            **kw_args: Keyword arguments to set on the CameraSensorSpec,
                such as uuid, hfov, sensor_type, and sensor_subtype.

        Returns:
            A configured habitat_sim.CameraSensorSpec instance.
        """
        camera_sensor_spec = habitat_sim.CameraSensorSpec()
        camera_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
        camera_sensor_spec.resolution = [camera.height, camera.width]
        camera_sensor_spec.position = [0, 0, 0]
        for k in kw_args:
            setattr(camera_sensor_spec, k, kw_args[k])
        return camera_sensor_spec

    if camera.color_sensor:
        sensor_specs.append(
            create_camera_spec(
                uuid="color_sensor",
                hfov=camera.hfov,
                sensor_type=habitat_sim.SensorType.COLOR,
                sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
            )
        )

    if camera.depth_sensor:
        sensor_specs.append(
            create_camera_spec(
                uuid="depth_sensor",
                hfov=camera.hfov,
                sensor_type=habitat_sim.SensorType.DEPTH,
                channels=1,
                sensor_subtype=habitat_sim.SensorSubType.PINHOLE,
            )
        )

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(sim_cfg, [agent_cfg])


# Warnings Magnum's importers print once per texture and glTF node of a scene (about 1600 for an
# HSSD scene); no Magnum or habitat-sim setting turns them off.
ASSET_WARNINGS = (
    r"Trade::BasisImporter::openData\(\): the image was not encoded Y-flipped"
    r"|Trade::GltfImporter::scene\(\): node \d+ extras \S+ property is .*, skipping$"
)

# Copies stdin to stderr line by line, dropping the lines that match argv[1]; ignores SIGINT and
# ends at EOF, so it outlives the node's own shutdown output.
_STDERR_FILTER = """
import re, signal, sys
signal.signal(signal.SIGINT, signal.SIG_IGN)
drop = re.compile(sys.argv[1].encode())
for line in sys.stdin.buffer:
    if not drop.match(line.rstrip(b"\\n")):
        sys.stderr.buffer.write(line)
        sys.stderr.buffer.flush()
"""


def filter_stderr(pattern=ASSET_WARNINGS):
    """Route this process's stderr, C++ output included, through a child process that drops the
    lines matching ``pattern``. The filter is a separate process, so a write never waits on this
    interpreter's lock."""
    original = os.dup(2)
    try:
        child = subprocess.Popen(
            [sys.executable, "-c", _STDERR_FILTER, pattern], stdin=subprocess.PIPE, stderr=original
        )
    finally:
        os.close(original)
    sys.stderr.flush()
    os.dup2(child.stdin.fileno(), 2)
    child.stdin.close()
    return child


class HabitatRenderer:
    """Render queued ROS camera transforms and publish Habitat observations."""

    def __init__(self):
        import habitat_sim

        rospy.init_node("habitat_renderer_node")

        # Map ROS's Z-up world frame into Habitat's Y-up world frame.
        self.R_habitat_world_to_ros_world = R.from_euler(
            "ZYX", [0.0, math.pi / 2.0, -math.pi / 2.0]
        )

        # ROS optical: +Z forward, +X right, +Y down. Habitat: -Z forward,
        # +X right, +Y up.
        self.R_optical_frame_to_habitat_camera = R.from_euler("ZYX", [0, 0, math.pi])

        self.camera = CameraConfig.from_rosparams()
        self.depth = DepthConfig.from_rosparams()

        rgb_image_topic = rospy.get_param("~rgb_image_topic", "/open_apero_camera/rgb_image")
        depth_image_topic = rospy.get_param("~depth_image_topic", "/open_apero_camera/depth_image")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/open_apero_camera/camera_info")
        camera_transform_topic = rospy.get_param("~camera_transform_topic", "/camera_transform")
        simulation_ready_topic = rospy.get_param(
            "~simulation_ready_topic", "/open_apero/simulation_ready"
        )

        if self.camera.color_sensor:
            self.color_image_pub = rospy.Publisher(
                rgb_image_topic, ROS_Image, queue_size=2, latch=True
            )
        if self.camera.depth_sensor:
            self.depth_image_pub = rospy.Publisher(
                depth_image_topic, ROS_Image, queue_size=2, latch=True
            )
        self.camera_info_pub = rospy.Publisher(
            camera_info_topic, CameraInfo, queue_size=2, latch=True
        )

        # Bound the rendering backlog; the callback blocks if rendering falls
        # ten poses behind rather than consuming unbounded memory.
        self.transform_queue = Queue(maxsize=10)

        model_type = rospy.get_param("~model_type", "ground_truth")
        if model_type != "ground_truth":
            raise ValueError(
                f"Unknown sensor model {model_type!r}; the only model is 'ground_truth'"
            )

        # Private paths override experiment-wide paths.
        scene_dataset_config_file = rospy.get_param(
            "~scene_dataset_config_file", rospy.get_param("/scene_dataset_config_file", "")
        )
        scene_id = str(rospy.get_param("~scene_id", "102816036"))
        mesh_folder = rospy.get_param("~mesh_folder", rospy.get_param("/mesh_folder", ""))
        scene = resolve_scene(scene_id, mesh_folder)

        self.camera_optical_frame_id = rospy.get_param(
            "~camera_optical_frame", "camera_optical_frame"
        )
        self.map_frame_id = rospy.get_param("~map_frame", "map")

        # Intrinsics are static; build the CameraInfo message once and restamp
        # it per frame.
        self.camera_info_msg = self.make_camera_info()

        self.cfg = make_habitat_cfg(self.camera, scene, scene_dataset_config_file)
        self.sim = habitat_sim.Simulator(self.cfg)

        # Register last so callbacks see fully initialized state.
        self.camera_optical_sub = rospy.Subscriber(
            camera_transform_topic, TransformStamped, self.camera_transform_callback
        )

        self.ready_pub = rospy.Publisher(simulation_ready_topic, String, queue_size=1, latch=True)
        self.ready_pub.publish(String("Simulation Ready"))
        rospy.loginfo("Habitat simulator ready")

    def close(self):
        """Close the Habitat simulator."""
        self.sim.close()
        rospy.loginfo("Habitat simulator closed")

    def make_image_msg(self, data, encoding, step, stamp):
        """Assemble a ROS Image message sharing this camera's frame and size.

        Args:
            data: Raw image bytes in row-major order.
            encoding: sensor_msgs image encoding string (e.g. ``rgb8``).
            step: Row length in bytes.
            stamp: ROS timestamp to stamp the message.

        Returns:
            A populated sensor_msgs/Image message.
        """
        image = ROS_Image()
        image.header.frame_id = self.camera_optical_frame_id
        image.header.stamp = stamp
        image.height = self.camera.height
        image.width = self.camera.width
        image.encoding = encoding
        image.step = step
        image.is_bigendian = False
        image.data = data
        return image

    def update_color_observation(self, obs, now):
        """Publish a Habitat color observation as an rgb8 ROS Image.

        Args:
            obs: Dictionary of sensor observations from Habitat.
            now: ROS timestamp to stamp the published message.
        """
        # Habitat returns RGBA; drop the alpha channel to get rgb8 bytes.
        rgb = obs["color_sensor"][:, :, :3].tobytes()
        self.color_image_pub.publish(self.make_image_msg(rgb, "rgb8", 3 * self.camera.width, now))

    def update_depth_observation(self, obs, now):
        """Publish a Habitat depth observation as 32FC1 after ``process_depth``.

        Args:
            obs: Dictionary of sensor observations from Habitat.
            now: ROS timestamp to stamp the published message.
        """
        depth = process_depth(
            obs["depth_sensor"],
            self.depth.min_depth,
            self.depth.max_depth,
            self.depth.far_depth_sentinel,
        )
        data = depth.tobytes()
        self.depth_image_pub.publish(self.make_image_msg(data, "32FC1", self.camera.width * 4, now))

    def make_camera_info(self):
        """Build the static CameraInfo message from the pinhole intrinsics.

        Returns:
            A CameraInfo message with intrinsics filled in; the header stamp is
            set per frame by publish_camera_info.
        """
        fx, fy, cx, cy = camera_intrinsics(self.camera.width, self.camera.height, self.camera.hfov)

        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_optical_frame_id
        camera_info.width = self.camera.width
        camera_info.height = self.camera.height
        camera_info.distortion_model = "plumb_bob"
        camera_info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return camera_info

    def publish_camera_info(self, now):
        """Restamp and publish the pre-built CameraInfo message.

        Args:
            now: ROS timestamp to stamp the published message.
        """
        self.camera_info_msg.header.stamp = now
        self.camera_info_pub.publish(self.camera_info_msg)

    def camera_transform_callback(self, msg: TransformStamped):
        """Validate and enqueue a camera transform for the render loop.

        Args:
            msg: A TransformStamped message with the camera optical frame
                pose expressed in the world frame.
        """
        if msg.child_frame_id != self.camera_optical_frame_id:
            rospy.logerr(
                "Received transform with unexpected child frame_id: %s", msg.child_frame_id
            )
            return

        if msg.header.frame_id != self.map_frame_id:
            rospy.logerr(
                "Received transform with unexpected header frame_id: %s", msg.header.frame_id
            )
            return

        self.transform_queue.put(msg)

    def render_at(self, msg: TransformStamped):
        """Render Habitat observations at the pose in a TransformStamped.

        Converts the camera position and orientation from the ROS coordinate
        system to the Habitat coordinate system, updates the simulator agent
        state, and retrieves new sensor observations.

        Args:
            msg: A TransformStamped with the camera optical frame pose expressed
                in the world frame.

        Returns:
            Dictionary of sensor observations from Habitat.
        """
        position = msg.transform.translation
        orientation = msg.transform.rotation
        ros_position = np.array([position.x, position.y, position.z])
        ros_quaternion = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

        # Express the ROS-world position in Habitat's world axes.
        habitat_camera_position = self.R_habitat_world_to_ros_world.apply(ros_position)

        # Compose local camera-axis conversion, the requested ROS pose, and the
        # ROS-world-to-Habitat-world conversion (rightmost rotation acts first).
        habitat_camera_quaternion = (
            self.R_habitat_world_to_ros_world
            * ros_quaternion
            * self.R_optical_frame_to_habitat_camera
        ).as_quat()

        agent_state = self.sim.get_agent(0).get_state()
        for sensor in agent_state.sensor_states:
            agent_state.sensor_states[sensor].position = habitat_camera_position
            agent_state.sensor_states[sensor].rotation = habitat_camera_quaternion

        self.sim.get_agent(0).set_state(agent_state, infer_sensor_states=False)
        return self.sim.get_sensor_observations()

    def run_rendering(self):
        """Run the main rendering loop.

        Blocks on the transform queue for new camera poses and renders
        observations as they arrive. Must run in a single thread because the
        Habitat simulator is not thread-safe.
        https://github.com/facebookresearch/habitat-sim/issues/312
        """
        while not rospy.is_shutdown():
            try:
                msg = self.transform_queue.get(timeout=0.1)
            except Empty:
                continue

            now = msg.header.stamp
            observations = self.render_at(msg)
            self.publish_camera_info(now)
            if self.camera.color_sensor:
                self.update_color_observation(observations, now)
            if self.camera.depth_sensor:
                self.update_depth_observation(observations, now)


def main():
    """Entry point for the Habitat ROS renderer node."""
    filter_stderr()
    renderer = HabitatRenderer()
    try:
        renderer.run_rendering()
    finally:
        renderer.close()


if __name__ == "__main__":
    main()
