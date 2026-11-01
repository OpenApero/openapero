"""Run-directory lifecycle and startup sequencing of the mission coordinator.

The coordinator is the only place that creates the timestamped run directory and
hands it to the planner. Getting the order wrong (publishing run_dir after
toggle_running) makes the planner reject an empty run_dir, so the ordering is
asserted explicitly here.

The implementation lives in the installed package
(``open_apero_ros.mission_coordinator``), so each test stubs rospy and the
message/service types into ``sys.modules``, imports the module fresh so its
body binds this test's stubs, and restores the interpreter state afterwards.
Written with plain functions and no pytest fixtures.
"""

import importlib
import os
import shutil
import sys
import tempfile
import types

_MODULE = "open_apero_ros.mission_coordinator"


class _RospyStub(object):
    """Minimal rospy replacement that records the calls the tests care about."""

    class ROSException(Exception):
        pass

    class ROSInterruptException(Exception):
        pass

    def __init__(self, params, service_up=True):
        self._params = params
        self._service_up = service_up
        self.calls = []  # ordered log of significant interactions
        self.waited_for = []  # topics passed to wait_for_message
        self.set_params = {}

    # --- parameters --------------------------------------------------------
    def get_param(self, name, default=None):
        return self._params.get(name, default)

    def set_param(self, name, value):
        self.set_params[name] = value
        self.calls.append(("set_param", name))

    # --- node lifecycle ----------------------------------------------------
    def init_node(self, *_a, **_k):
        pass

    def on_shutdown(self, *_a, **_k):
        pass

    def signal_shutdown(self, *_a, **_k):
        self.calls.append(("signal_shutdown", None))

    def is_shutdown(self):
        return False

    # --- logging -----------------------------------------------------------
    def loginfo(self, *_a, **_k):
        pass

    def logwarn(self, message, *_a, **_k):
        self.calls.append(("logwarn", message))

    def logerr(self, message, *_a, **_k):
        self.calls.append(("logerr", message))

    def logfatal(self, message, *_a, **_k):
        self.calls.append(("logfatal", message))

    # --- comms -------------------------------------------------------------
    def wait_for_message(self, topic, *_a, **_k):
        self.waited_for.append(topic)
        self.calls.append(("wait_for_message", topic))

    def wait_for_service(self, name, *_a, **_k):
        self.calls.append(("wait_for_service", name))
        if not self._service_up:
            raise self.ROSException("timeout exceeded while waiting for service " + name)

    def ServiceProxy(self, name, _type):
        def _call(_value):
            self.calls.append(("toggle_running", name))
            return types.SimpleNamespace(success=True, message="")

        return _call

    def Subscriber(self, *_a, **_k):
        return None

    def Timer(self, *_a, **_k):
        return None

    def Duration(self, *_a, **_k):
        return None

    def sleep(self, *_a, **_k):
        pass

    def get_time(self):
        return 0.0


def _stub_modules(rospy_stub):
    """Return the sys.modules entries the coordinator module imports."""
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    std_msgs.msg = std_msgs_msg

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.SetBool = type("SetBool", (), {})
    std_srvs.srv = std_srvs_srv

    return {
        "rospy": rospy_stub,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }


def _run_coordinator(service_up=True, **extra_params):
    """Construct a MissionCoordinator against stubs; return (coordinator, stub, root).

    A parameter passed as None is left unset. With service_up false, waiting for
    the planner service times out. The caller must pass the root to _cleanup()
    to remove it.
    """
    root = tempfile.mkdtemp(prefix="open_apero_coordinator_test_")
    params = {
        "~ns_planner": "/open_apero_node",
        "~delay": 0.0,
        "~startup_timeout": 0.0,
        "~time_limit": 0.0,
        "~runs_dir": os.path.join(root, "HSSD", "nbs"),
    }
    params.update(extra_params)
    params = {name: value for name, value in params.items() if value is not None}
    rospy_stub = _RospyStub(params, service_up)

    stubs = _stub_modules(rospy_stub)
    saved = {name: sys.modules.get(name) for name in stubs}
    # A previous test left the module cached bound to its own stub instance;
    # drop it so the import below re-executes the module body against this
    # test's stubs, and put whatever was there back afterwards.
    saved_module = sys.modules.pop(_MODULE, None)
    sys.modules.update(stubs)

    # module.subprocess IS the stdlib module, so patching check_call leaks into
    # the rest of the test process unless it is put back.
    import subprocess

    saved_check_call = subprocess.check_call
    try:
        module = importlib.import_module(_MODULE)
        # rosparam dump shells out; there is no roscore in a unit test.
        subprocess.check_call = lambda *_a, **_k: 0
        coordinator = module.MissionCoordinator()
    except BaseException:
        shutil.rmtree(root, ignore_errors=True)
        raise
    finally:
        subprocess.check_call = saved_check_call
        sys.modules.pop(_MODULE, None)
        if saved_module is not None:
            sys.modules[_MODULE] = saved_module
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous

    return coordinator, rospy_stub, root


def _cleanup(root):
    """Remove the temp root holding the coordinator's run directory."""
    shutil.rmtree(root, ignore_errors=True)


def test_waits_for_simulation_by_default():
    """Simulation runs must keep blocking on the simulation_ready message."""
    coordinator, rospy_stub, root = _run_coordinator()
    try:
        assert rospy_stub.waited_for == ["simulation_ready"]
    finally:
        _cleanup(root)


def test_skips_simulation_wait_when_disabled():
    """Real-robot runs have no simulator, so the wait must be skipped."""
    coordinator, rospy_stub, root = _run_coordinator(**{"~wait_for_simulation": False})
    try:
        assert rospy_stub.waited_for == []
    finally:
        _cleanup(root)


def test_run_dir_published_before_planner_starts():
    """The planner needs its run_dir before toggle_running, or it refuses to start."""
    coordinator, rospy_stub, root = _run_coordinator(**{"~wait_for_simulation": False})
    try:
        kinds = [name for name, _ in rospy_stub.calls]
        assert "set_param" in kinds
        assert "toggle_running" in kinds
        assert kinds.index("set_param") < kinds.index("toggle_running")
    finally:
        _cleanup(root)


def test_creates_timestamped_run_directory():
    """A unique timestamped directory is created under the configured runs directory."""
    coordinator, rospy_stub, root = _run_coordinator(**{"~wait_for_simulation": False})
    try:
        published = rospy_stub.set_params["/open_apero_node/run_dir"]
        assert published == coordinator.run_dir
        assert os.path.isdir(published)
        # Nested one level below the configured folder, and named as a timestamp.
        parent, stamp = os.path.split(published)
        assert parent == os.path.join(root, "HSSD", "nbs")
        assert len(stamp) == len("20260726_120000")
        assert stamp[8] == "_"
    finally:
        _cleanup(root)


def test_missing_runs_dir_fails_without_creating_a_directory():
    """No ~runs_dir is a fatal error, not a relative directory in the working directory."""
    cwd = os.getcwd()
    scratch = tempfile.mkdtemp(prefix="open_apero_coordinator_cwd_")
    os.chdir(scratch)
    try:
        try:
            _run_coordinator(**{"~runs_dir": None, "~wait_for_simulation": False})
        except SystemExit as error:
            assert error.code == 1
        else:
            raise AssertionError("MissionCoordinator started without ~runs_dir")
        assert os.listdir(scratch) == []
    finally:
        os.chdir(cwd)
        shutil.rmtree(scratch, ignore_errors=True)


def test_startup_timeout_shuts_down_without_calling_the_absent_planner_service():
    """A planner that never came up has no toggle_running to call; shutdown must still happen."""
    coordinator, rospy_stub, root = _run_coordinator(
        service_up=False, **{"~wait_for_simulation": False, "~startup_timeout": 5.0}
    )
    try:
        kinds = [name for name, _ in rospy_stub.calls]
        assert "toggle_running" not in kinds
        assert kinds[-2:] == ["logerr", "signal_shutdown"]
        assert "Planner startup failed" in rospy_stub.calls[-2][1]
    finally:
        _cleanup(root)


def test_a_started_run_stops_the_planner_before_shutting_down():
    """Once planning has started, stopping the experiment toggles the planner off."""
    coordinator, rospy_stub, root = _run_coordinator(**{"~wait_for_simulation": False})
    try:
        coordinator.stop_experiment("Collision detected during execution.")
        kinds = [name for name, _ in rospy_stub.calls]
        assert kinds[-3:] == ["logerr", "toggle_running", "signal_shutdown"]
    finally:
        _cleanup(root)


def test_the_time_limit_stops_the_run_with_a_warning_not_an_error():
    """The time limit is a configured end of the run."""
    coordinator, rospy_stub, root = _run_coordinator(
        **{"~wait_for_simulation": False, "~time_limit": 1.0}
    )
    try:
        started = len(rospy_stub.calls)
        rospy_stub.get_time = lambda: 59.0
        coordinator.check_time_limit_callback(None)
        assert rospy_stub.calls[started:] == []

        rospy_stub.get_time = lambda: 60.0
        coordinator.check_time_limit_callback(None)
        kinds = [name for name, _ in rospy_stub.calls]
        assert kinds[-3:] == ["logwarn", "toggle_running", "signal_shutdown"]
        assert "Time limit reached (1.0 min)." in rospy_stub.calls[-3][1]
        assert "logerr" not in kinds
    finally:
        _cleanup(root)
