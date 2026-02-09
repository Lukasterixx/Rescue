# walk_forward_behavior.py

import py_trees
from geometry_msgs.msg import Twist

class WalkForwardBehavior(py_trees.behaviour.Behaviour):
    """
    A py_trees.Behaviour that publishes a constant forward Twist.
    It does not itself check 'permission'; it always publishes when ticked.
    """
    def __init__(self, name: str, publisher, twist_msg: Twist):
        super(WalkForwardBehavior, self).__init__(name)
        self.publisher = publisher
        self.twist_msg = twist_msg

    def setup(self, **kwargs) -> bool:
        # One‐time setup; nothing to do here
        return True

    def initialise(self) -> None:
        # Called when this behaviour transitions to RUNNING; not used here
        pass

    def update(self) -> py_trees.common.Status:
        # Publish the forward command
        self.publisher.publish(self.twist_msg)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.stop_twist = Twist()
        self.publisher.publish(self.stop_twist)
        self.logger.debug(f"{self.name}: terminate → publishing zero Twist")