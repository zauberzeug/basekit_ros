#!/usr/bin/env python3

""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from field_friend_driver.communication.serial_communication import SerialCommunication
from field_friend_driver.modules.bms_handler import BMSHandler
from field_friend_driver.modules.configuration_handler import ConfigurationHandler
from field_friend_driver.modules.estop_handler import EStopHandler
from field_friend_driver.modules.odom_handler import OdomHandler
from field_friend_driver.modules.twist_handler import TwistHandler
from field_friend_driver.modules.yaxis_handler import YAxisHandler
from field_friend_driver.modules.zaxis_handler import ZAxisHandler


class FieldFriendDriver(Node):
    """Field friend node handler."""

    def __init__(self):
        super().__init__('field_friend_driver_node')

        # Get startup file from parameters
        self.declare_parameter('startup_file', '')
        startup_file = self.get_parameter('startup_file').value

        self._serial_communication = SerialCommunication(self)

        self._odom_handler = OdomHandler(self, self._serial_communication)
        self._bms_handler = BMSHandler(self, self._serial_communication)
        self._twist_handler = TwistHandler(self, self._serial_communication)
        self._estop_handler = EStopHandler(self, self._serial_communication)
        self._yaxis_handler = YAxisHandler(self, self._serial_communication)
        self._zaxis_handler = ZAxisHandler(self, self._serial_communication)
        self._configuration_handler = ConfigurationHandler(
            self, self._serial_communication, startup_file)

        self.read_timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        """Read data from the serial communication."""
        self._serial_communication.read()


def main(args=None):
    """Implmenets main function call."""
    rclpy.init(args=args)

    try:
        field_friend_driver = FieldFriendDriver()

        executor = SingleThreadedExecutor()
        executor.add_node(field_friend_driver)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
