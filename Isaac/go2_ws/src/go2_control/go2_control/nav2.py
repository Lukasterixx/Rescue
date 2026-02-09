import os
import subprocess

import py_trees
from py_trees_ros.actions import ActionClient
from nav2_msgs.action import NavigateToPose
import geometry_msgs.msg

class Nav2BringupBehavior(py_trees.behaviour.Behaviour):
    """
    Launches the nav2_bringup.launch.py via ros2 launch.
    """
    def __init__(self, name="Nav2Bringup"):
        super().__init__(name)
        self.proc = None

    def setup(self, timeout_sec):
        # find the launch file installed in your workspace
        prefix = os.getenv('COLCON_PREFIX_PATH').split(':')[0]
        launch = os.path.join(prefix, 'share', 'go2_control',
                              'launch', 'nav2_bringup.launch.py')
        self.proc = subprocess.Popen(['ros2', 'launch', launch])
        return True

    def update(self):
        # keep running until user kills it
        return py_trees.common.Status.RUNNING


class Nav2GoalBehavior(ActionClient):
    """
    Sends a NavigateToPose goal and waits for completion.
    """
    def __init__(self, name="Nav2Navigate", goal_pose=None):
        # Prepare the goal message
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose if goal_pose is not None else geometry_msgs.msg.PoseStamped()
        # Initialize the base ActionClient: name, action type, action namespace, and initial goal
        super().__init__(name, NavigateToPose, 'navigate_to_pose', goal)
        # Store for reference
        self.goal_msg = goal

    def on_tick(self):
        # Send the goal each tick until accepted
        self.logger.debug(f"{self.name}: sending goal")
        self.send_goal(self.goal_msg)

    def on_feedback(self, feedback_msg):
        # Log remaining distance
        dist = feedback_msg.feedback.distance_remaining
        self.logger.debug(f"{self.name}: {dist:.2f}m remaining")

    def on_result(self, result):
        # Navigation finished
        self.logger.info(f"{self.name}: navigation result: {result.status}")
