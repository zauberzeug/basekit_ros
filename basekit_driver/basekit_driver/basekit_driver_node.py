#!/usr/bin/env python3

""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from basekit_driver.communication.serial_communication import SerialCommunication
from basekit_driver.modules.bms_handler import BMSHandler
from basekit_driver.modules.configuration_handler import ConfigurationHandler
from basekit_driver.modules.estop_handler import EStopHandler
from basekit_driver.modules.odom_handler import OdomHandler
from basekit_driver.modules.twist_handler import TwistHandler
from basekit_driver.modules.yaxis_handler import YAxisHandler
from basekit_driver.modules.zaxis_handler import ZAxisHandler


class BasekitDriver(Node):
    """Basekit node handler."""

    def __init__(self):
        super().__init__('basekit_driver_node')

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
        basekit_driver = BasekitDriver()

        executor = SingleThreadedExecutor()
        executor.add_node(basekit_driver)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
