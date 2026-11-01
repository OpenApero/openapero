"""Mission coordinator ROS node for Open Apero experiments.

Manages the lifecycle of a planning experiment by creating the timestamped run
directory, waiting for the planner (and, in simulation, the simulator) to become
ready, starting the planner, enforcing time limits and detecting collisions. The
planner writes the results into the run directory.

Set ~wait_for_simulation to false for real-robot runs, where no simulator
publishes the "simulation_ready" message.
"""

import datetime
import os
import subprocess

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class MissionCoordinator(object):
    """Coordinates an Open Apero planning experiment.

    Handles the run directory, startup synchronization (with the simulator when
    there is one), planner launch with configurable delays, time limit
    enforcement and collision detection.
    """

    def __init__(self):
        """Initialize the mission coordinator node and start the experiment."""
        rospy.init_node("mission_coordinator_node")

        self.ns_planner = rospy.get_param("~ns_planner", "/open_apero_node")
        self.planner_delay = rospy.get_param("~delay", 0.0)
        self.startup_timeout = rospy.get_param("~startup_timeout", 0.0)
        self.time_limit = rospy.get_param("~time_limit", 0.0)
        # Real-robot runs have no simulator to announce itself.
        self.wait_for_simulation = rospy.get_param("~wait_for_simulation", True)

        self.start_planning_time = None

        self.runs_dir = os.path.expanduser(rospy.get_param("~runs_dir", ""))
        if not self.runs_dir:
            rospy.logfatal(
                "~runs_dir is required: the parent of the stamped run directories, as "
                "run_experiment_sim.launch sets it "
                "(<data_root>/<scene_dataset>/<scene_id>/<planner>)."
            )
            raise SystemExit(1)
        if not os.path.isdir(self.runs_dir):
            os.makedirs(self.runs_dir, exist_ok=True)
            rospy.loginfo("Runs directory created at '%s'." % self.runs_dir)

        self.run_dir = os.path.join(
            self.runs_dir, datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        os.mkdir(self.run_dir)
        self.log_banner_message("Stamped run directory created:\n" + self.run_dir)
        rospy.set_param(self.ns_planner + "/run_dir", self.run_dir)

        self.collided = False
        self.collision_sub = rospy.Subscriber(
            "collision", String, self.collision_callback, queue_size=10
        )

        self.start_experiment()

    def start_experiment(self):
        """Wait for the experiment to be ready, then start planning.

        Waits for the simulation ready message (simulation runs only) and the
        planner toggle service, applies any configured delay, starts the planner,
        dumps ROS parameters, and begins time limit monitoring.
        """
        if self.wait_for_simulation:
            rospy.loginfo("Waiting for simulation to setup...")
            if self.startup_timeout > 0.0:
                try:
                    rospy.wait_for_message("simulation_ready", String, self.startup_timeout)
                except rospy.ROSException:
                    self.stop_experiment(
                        "Simulation startup failed (timeout after %.1fs)." % self.startup_timeout
                    )
                    return
            else:
                rospy.wait_for_message("simulation_ready", String)
            self.log_banner_message("Simulation is ready.")
        else:
            self.log_banner_message("Not waiting for a simulation (~wait_for_simulation is false).")

        rospy.loginfo("Waiting for planner to be ready...")
        if self.startup_timeout > 0.0:
            try:
                rospy.wait_for_service(self.ns_planner + "/toggle_running", self.startup_timeout)
            except rospy.ROSException:
                self.stop_experiment(
                    "Planner startup failed (timeout after %.1fs)." % self.startup_timeout
                )
                return
        else:
            rospy.wait_for_service(self.ns_planner + "/toggle_running")
        self.log_banner_message("Planner service is online.")

        if self.planner_delay > 0:
            rospy.loginfo(
                "Extra wait time to ensure communication is stable (%.1f seconds)..."
                % self.planner_delay
            )
            rospy.sleep(self.planner_delay)

        toggle_planner_srv = rospy.ServiceProxy(self.ns_planner + "/toggle_running", SetBool)
        start_response = toggle_planner_srv(True)
        if not start_response.success:
            # The planner refused to start (e.g. run-output configuration failed).
            # Abort loudly rather than record an unlogged or partially-logged run.
            self.stop_experiment("Planner refused to start: %s" % start_response.message)
            return
        self.start_planning_time = rospy.get_time()
        self.log_banner_message("Planner has started. Planning is in progress.")

        subprocess.check_call(
            [
                "rosparam",
                "dump",
                os.path.join(self.run_dir, "rosparams.yaml"),
                "/",
            ]
        )
        rospy.loginfo("Dumped the parameter server into 'rosparams.yaml'.")

        rospy.Timer(rospy.Duration(1.0), self.check_time_limit_callback)

    def check_time_limit_callback(self, _):
        """Timer callback that stops the experiment if the time limit is reached.

        Args:
            _: Unused timer event argument.
        """
        if self.time_limit > 0.0:
            elapsed = rospy.get_time() - self.start_planning_time
            if elapsed >= self.time_limit * 60.0:
                self.stop_experiment(
                    "Time limit reached (%.1f min)." % self.time_limit, level="warn"
                )

    def stop_experiment(self, reason, level="error"):
        """Stop the planner and shut down the ROS node.

        Args:
            reason: Human-readable string describing why the experiment stopped.
            level: Log level of the banner, as log_banner_message takes it.
        """
        self.log_banner_message("Experiment stopped: " + reason, level=level)

        # Before planning started the service may not exist, and calling it would raise.
        if self.start_planning_time is not None:
            toggle_planner_srv = rospy.ServiceProxy(self.ns_planner + "/toggle_running", SetBool)
            toggle_planner_srv(False)

        rospy.signal_shutdown(reason)

    def collision_callback(self, _):
        """Stop the experiment on the first message on ``collision``.

        No node in this repository publishes it; a collision monitor (real robot or a
        physics simulator) may publish any ``std_msgs/String`` there to end the run.

        Args:
            _: Unused message argument.
        """
        if not self.collided:
            self.collided = True
            self.stop_experiment("Collision detected during execution.")

    def log_banner_message(self, msg, level="info", min_width=60):
        """Log a message formatted inside an ASCII banner.

        Args:
            msg: The message text to display, may contain newlines.
            level: Log level string, one of 'info', 'warn', or 'error'.
            min_width: Minimum character width of the banner.
        """
        lines = msg.split("\n")
        width = max(max(len(line) for line in lines) + 4, min_width)
        banner = "\n" + "*" * width
        for line in lines:
            banner += f"\n* {line.center(width - 4)} *"
        banner += f"\n{'*' * width}"
        if level == "info":
            rospy.loginfo(banner)
        elif level == "warn":
            rospy.logwarn(banner)
        elif level == "error":
            rospy.logerr(banner)


def main():
    MissionCoordinator()
    rospy.spin()


if __name__ == "__main__":
    main()
