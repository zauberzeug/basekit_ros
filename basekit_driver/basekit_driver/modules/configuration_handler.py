import os

from rclpy.node import Node
from std_msgs.msg import String

from basekit_driver.communication.communication import Communication


class ConfigurationHandler:
    """Handle lizard configuration file."""

    def __init__(self, node: Node, comm: Communication, filename: str = None):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._filename = filename
        self._software_estop: bool = False
        self._subscription = node.create_subscription(
            String, 'configure', self.handle_configure, 10
        )

    def handle_configure(self, msg: String):
        """Push startup.liz to microcontroller if available."""
        if not self._filename:
            self._logger.warn('No startup file configured. Skipping configuration.')
            return

        if not os.path.exists(self._filename):
            self._logger.error(f'Startup file {self._filename} not found!')
            return

        try:
            with open(self._filename) as f:
                self._logger.info(f'Applying configuration from {self._filename}')
                self._comm.send('!-')
                for line in f.read().splitlines():
                    self._comm.send(f'!+{line}')
                self._comm.send('!.')
                self._comm.send('core.restart()')
                self._logger.info('Configuration applied successfully')
        except Exception as e:
            self._logger.error(f'Failed to apply configuration: {str(e)}')
