from rclpy.node import Node
from rosys.automation import AppButton, Automator
from rosys.automation.app_controls_ import AppControls
from rosys.hardware import BluetoothHardware, RobotBrain
from std_msgs.msg import Empty

from .base import Handler


def create_app_controls(robot_brain: RobotBrain, bluetooth: BluetoothHardware) -> AppControls:
    """Build the RoSys-level `AppControls` object.

    Must be called synchronously right after `System(...)`, before the ROS node thread starts -
    not from `DevkitDriver.__init__`. `AppControls` subscribes to `robot_brain.ESP_CONNECTED` in
    its constructor, a one-shot event that fires once `RobotBrain`'s own `rosys.on_startup(self.
    enable_esp)` completes; the ROS thread needs to spin up rclpy/DDS first, so by the time it
    would construct this, that event has already fired and passed. Missing it leaves the app's
    button toolbar permanently empty (never synced) even though driving and telemetry, which don't
    depend on it, work fine - constructing it here instead, synchronously alongside `System(...)`,
    guarantees the subscription is in place before RoSys's startup tasks get to run at all.
    """
    return AppControls(robot_brain, Automator(None, notify=False), bluetooth=bluetooth)


class AppControlsHandler(Handler):
    """Expose the weeding screw's home routine as a mobile-app button.

    Ports the "reference implement" icon from the old feldfreund app - lost when the app/BLE
    integration wasn't carried over during the ROS migration - but as a ROS publisher rather than
    a direct call into the implement: pressing the button publishes to the same `weeding_screw/home`
    topic the web dashboard's Home button already uses (see `WeedingScrewHandler`), so both sources
    drive the implement through the one existing command path instead of a second, app-specific one.
    """

    def __init__(self, node: Node, app_controls: AppControls):
        super().__init__(node)
        self._home_publisher = node.create_publisher(Empty, 'weeding_screw/home', 10)
        app_controls.extra_buttons['reference_implement'] = AppButton('build', released=self._handle_reference)

    def _handle_reference(self) -> None:
        self._home_publisher.publish(Empty())
