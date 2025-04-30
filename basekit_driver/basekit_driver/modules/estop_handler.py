from rclpy.node import Node
from std_msgs.msg import Bool

from basekit_driver.communication.communication import Communication


class EStopHandler:
    """Handle the estop."""

    def __init__(self, node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._software_estop: bool = False
        self.subscription = node.create_subscription(
            Bool, 'emergency_stop', self.callback, 10
        )

    def update(self, estop: bool):
        """Update estop position."""
        self._software_estop = estop

    def send(self):
        """Send estop command."""
        return f"en3.level({'false' if self._software_estop else 'true'})"

    def callback(self, msg: Bool):
        """Implement a callback for the estop."""
        self.update(msg.data)
        self._comm.send(self.send())
