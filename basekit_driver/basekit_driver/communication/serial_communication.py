#!/usr/bin/env python3
""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

import os
from functools import reduce
from operator import ixor
from threading import Lock
from typing import Any, List

import rclpy
import serial
from rclpy.node import Node

from .communication import Communication


class CoreData():
    """Handles data from the core esp."""

    def __init__(self, name: str, pos: int, type: str, default: Any) -> None:
        self._name = name
        self._pos = pos
        self._default = default
        self._type = type

    def get_name(self) -> str:
        """Get name."""
        return self._name

    def get_type(self) -> str:
        """Get type."""
        return self._type

    def get_pos(self) -> int:
        """Get position in array"""
        return self._pos

    def get_default(self) -> Any:
        """Get default value"""
        return self._default


class SerialCommunication(Communication):
    """Handle serial communication."""

    def __init__(self, node: Node):
        super().__init__()
        self._logger = node.get_logger()
        self._logger.info('Init serial communication')
        self.open_port()
        self.mutex = Lock()
        self.init_core_data(node)

    def init_core_data(self, node: Node):
        """Initialize the core data struct. """

        self._core_data_list = []

        node.declare_parameter('read_data.list', rclpy.Parameter.Type.STRING_ARRAY)
        # Get the parameter
        data_list = node.get_parameter('read_data.list').value
        self._logger.info(f'Received list of strings: {data_list}')

        pos = 0
        for data in data_list:
            node.declare_parameter('read_data.' + data + '.type', rclpy.Parameter.Type.STRING)
            type_str = node.get_parameter('read_data.' + data + '.type').value
            if (type_str == "bool"):
                node.declare_parameter('read_data.' + data + '.default', rclpy.Parameter.Type.BOOL)
            elif (type_str == "int"):
                node.declare_parameter(
                    'read_data.' + data + '.default',
                    rclpy.Parameter.Type.INTEGER)
            elif (type_str == "double"):
                node.declare_parameter(
                    'read_data.' + data + '.default',
                    rclpy.Parameter.Type.DOUBLE)
            default = node.get_parameter('read_data.' + data + '.default').value
            self._core_data_list.append(CoreData(data, pos, type_str, default))
            pos = pos + 1

        self._core_data = {}
        for data in self._core_data_list:
            self._core_data[data.get_name()] = data.get_default()

    def enable(self):
        """
        Enable serial communication.

        Enable serial communication. There we need to call the flash
        python script from the lizard driver.
        """
        self._logger.info('Enable esp')
        command = '/root/.lizard/flash.py enable'
        os.system(command)
        self._logger.info('Esp is now enabled')

    def open_port(self):
        """Open port to device."""
        try:
            self.enable()
            # self.port.open()
            self.port = serial.Serial('/dev/ttyTHS0', 115200)
        except serial.SerialException:
            self._logger.error('Could not open serial communication!')
            self.port = None

    def calculate_checksum(self, line: str) -> int:
        """Calculate checkusm of line."""
        return reduce(ixor, map(ord, line))

    def append_checksum(self, line: str) -> str:
        """Append checksum to the line."""
        checksum = self.calculate_checksum(line)
        line = f'{line}@{checksum:02x}\n'
        return line

    def send(self, line: str) -> None:
        """Send message to serial device."""
        # line = f"wheels.speed({cmd_msg.linear.x:3f}, {cmd_msg.angular.z:.3f})"
        if self.port is not None:
            line = self.append_checksum(line)
            self.mutex.acquire()
            self.port.write(line.encode())
            self.mutex.release()
        else:
            self._logger.warning('No Port open')

    def validate_checksum(self, line: str) -> bool:
        """Validate checksum."""
        line, checksum = line.split('@', 1)
        return self.calculate_checksum(line) == int(checksum, 16)

    def handle_core_message(self, words: List[str]) -> None:
        """Handle core message."""
        # self._logger.info(f'{words}')

        words.pop(0)

        # self._logger.error(f"{words}")
        for data in self._core_data_list:
            if (data.get_type() == "bool"):
                value = words[data.get_pos()]
                if value == "true":
                    value = True
                elif value == "false":
                    value = False
                else:
                    value = bool(float(words[data.get_pos()]) > 0.5)
            elif (data.get_type() == "int"):
                value = int(words[data.get_pos()])
            elif (data.get_type() == "double"):
                value = float(words[data.get_pos()])
            else:
                return

            self._core_data[data.get_name()] = value
        self.notify_core_observers(self._core_data)

    def handle_expander_message(self, words: List[str]):
        """Handle expander message."""
        if len(words) < 2:
            return
        if words[1] == 'bms':
            self.notify_bms_observers(words[2:])

    def read(self) -> None:
        """Read from serial device."""
        try:
            self.mutex.acquire()
            buffer = self.port.read_all().decode(errors='replace')
        finally:
            self.mutex.release()

        # Split lines if we found multiple lines
        lines = buffer.split('\n')
        for line in lines:
            # self._logger.info(f'{line}')
            line = line.rstrip()
            if line[-3:-2] == '@' and line.count('@') == 1:
                if not self.validate_checksum(line):
                    return
                line = line[:-3]
            words = line.split()
            try:
                if not any(words):
                    return
                if words[0] == 'core':
                    self.handle_core_message(words)
                elif words[0] == 'expander:':
                    self.handle_expander_message(words)
                elif words[0] == 'error':
                    self._logger.error(f'{line}')
            except BaseException:
                self._logger.error(
                    f'General exception in the following line: {line} from the following buffer {buffer}')
