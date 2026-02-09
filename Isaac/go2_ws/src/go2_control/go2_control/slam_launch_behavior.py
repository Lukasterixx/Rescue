import os
import subprocess
import atexit
import py_trees
from py_trees.common import Status

class SLAMLaunchBehavior(py_trees.behaviour.Behaviour):
    """
    A behaviour that launches `ros2 launch lidarslam lidarslam.launch.py`
    when ticking begins, stays RUNNING, and shuts down the process on halt.
    Ensures SLAM is terminated on exit via an atexit handler.
    """
    def __init__(self, name="SLAMLauncher"):
        super().__init__(name)
        self.slam_proc = None

    def setup(self, **kwargs):
        # nothing to setup ahead of time
        return True

    def initialise(self):
        if self.slam_proc is None:
            cmd = ["ros2", "launch", "lidarslam", "lidarslam.launch.py"]
            self.logger.info(f"{self.name}: starting SLAM stack")
            self.slam_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=os.environ.copy()
            )
            # register cleanup at interpreter exit
            atexit.register(self._kill_slam)

    def update(self):
        # If process died, return FAILURE so the tree can react,
        # otherwise keep reporting RUNNING.
        if self.slam_proc and self.slam_proc.poll() is not None:
            self.logger.error(f"{self.name}: SLAM process exited unexpectedly")
            return Status.FAILURE
        return Status.RUNNING

    def terminate(self, new_status):
        # invoked when tree halts or is shut down
        if self.slam_proc and self.slam_proc.poll() is None:
            self.logger.info(f"{self.name}: terminating SLAM stack")
            self.slam_proc.terminate()
            try:
                self.slam_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.logger.warn(f"{self.name}: SLAM did not exit, killing")
                self.slam_proc.kill()
        self.slam_proc = None

    def _kill_slam(self):
        """Cleanup SLAM process on interpreter exit."""
        if self.slam_proc and self.slam_proc.poll() is None:
            try:
                self.slam_proc.terminate()
            except Exception:
                pass
