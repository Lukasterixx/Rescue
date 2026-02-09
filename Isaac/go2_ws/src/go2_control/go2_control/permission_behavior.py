# permission_behavior.py

import py_trees

class PermissionBehavior(py_trees.behaviour.Behaviour):
    """
    A placeholder condition that controls “permission” to walk.
    For now, it always returns SUCCESS; replace update() logic later
    with a real check (e.g. blackboard flag, topic, service, etc.).
    """
    def __init__(self, name: str):
        super(PermissionBehavior, self).__init__(name)

    def setup(self, **kwargs) -> bool:
        # One‐time setup; nothing to do here
        return True

    def initialise(self) -> None:
        # Called when this behaviour transitions to RUNNING; not used here
        pass

    def update(self) -> py_trees.common.Status:
        # Always grant permission for now
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # Cleanup if needed; not used for this placeholder
        pass
