"""Check that the driver picks up a soft e-stop that was latched before it started."""

import time
from unittest.mock import MagicMock

import pytest
import rclpy
from devkit_driver.modules import EStopHandler
from devkit_driver.qos import SAFETY_QOS
from rclpy.node import Node
from std_msgs.msg import Bool


def test_late_joining_driver_receives_latched_soft_estop(monkeypatch: pytest.MonkeyPatch) -> None:
    callback = MagicMock()
    monkeypatch.setattr(EStopHandler, 'soft_estop_callback', callback)
    rclpy.init()
    try:
        node = Node('test_estop_handler')
        publisher = node.create_publisher(Bool, 'estop/soft', SAFETY_QOS)
        publisher.publish(Bool(data=True))

        EStopHandler(node, MagicMock())
        deadline = time.monotonic() + 5.0
        while not callback.called and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert callback.called
        assert callback.call_args.args[0].data is True
    finally:
        rclpy.shutdown()
