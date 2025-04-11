""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from rclpy.node import Node
from std_msgs.msg import String

from field_friend_driver.communication.communication import Communication


class ConfigurationHandler:
    """Handle lizard configuration file."""

    def __init__(self, node: Node, comm: Communication, filename: str):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._filename = filename
        self._software_estop: bool = False
        self._subscription = node.create_subscription(
            String, 'configure', self.handle_configure, 10
        )

    def handle_configure(self, msg: String):
        """Push startup.liz to microcontroller."""
        with open(self._filename) as f:
            self._comm.send('!-')
            for line in f.read().splitlines():
                self._comm.send(f'!+{line}')
            self._comm.send('!.')
            self._comm.send('core.restart()')
