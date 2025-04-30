import math
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from gps_msgs.msg import GPSFix
from nicegui import Client, app, ui, ui_run
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # Create reliable QoS profile for emergency stop
        estop_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.estop_publisher = self.create_publisher(Bool, 'emergency_stop', estop_qos)

        self.subscription = self.create_subscription(GPSFix, 'gpsfix', self.store_gps, 1)
        self.battery_subscription = self.create_subscription(BatteryState, 'battery_state', self.store_battery, 1)
        self.latest_gps = None
        self.latest_battery = None
        self.timer = self.create_timer(2.0, self.update_gps_ui)
        self.params_available = False
        self.estop_active = False

        with Client.auto_index_client:
            with ui.row().classes('items-stretch w-[48rem] gap-3'):  # Add gap between items
                with ui.card().classes('flex-1 text-center items-center'):  # Use flex-1 for equal width distribution
                    ui.label('Control').classes('text-2xl')
                    ui.joystick(color='blue', size=50,
                                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                on_end=lambda _: self.send_speed(0.0, 0.0))
                    ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')
                    self.estop_button = ui.button('EMERGENCY STOP', color='blue').classes(
                        'mt-4 text-white font-bold w-40 min-h-[3rem]')
                    self.estop_button.on('click', self.toggle_estop)
                with ui.card().classes('flex-1 text-center items-center'):
                    ui.label('Data').classes('text-2xl')
                    ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                    slider_props = 'readonly selection-color=transparent'
                    self.linear = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                    self.angular = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('Battery').classes('text-xs mb-[-1.4em]')
                    self.battery = ui.label('---')
                with ui.card().classes('flex-1 text-center items-center'):
                    ui.label('Safety').classes('text-2xl')
                    ui.label('Bumpers').classes('text-xs mb-[-1.4em]')
                    self.bumper_front_top = ui.label('Front Top: ---').classes('text-sm')
                    self.bumper_front_bottom = ui.label('Front Bottom: ---').classes('text-sm')
                    self.bumper_back = ui.label('Back: ---').classes('text-sm')
                    ui.label('E-Stops').classes('text-xs mb-[-1.4em] mt-4')
                    self.estop1 = ui.label('E-Stop 1: ---').classes('text-sm')
                    self.estop2 = ui.label('E-Stop 2: ---').classes('text-sm')
                    self.params_status = ui.label('Waiting for parameters...').classes('text-red-500 text-sm mt-2')
            with ui.card().classes('w-[48rem] items-center mt-3'):
                ui.label('GPS Map').classes('text-2xl')
                self.map = ui.leaflet(center=(48.137154, 11.576124), zoom=16).classes('w-full h-96')
                self.marker = self.map.marker(latlng=self.map.center)

    def toggle_estop(self) -> None:
        """Toggle the emergency stop state."""
        self.estop_active = not self.estop_active
        msg = Bool()
        msg.data = self.estop_active
        self.estop_publisher.publish(msg)

        # Update button appearance while maintaining size
        if self.estop_active:
            self.estop_button.props('color=red')
            self.estop_button.text = 'STOPPED'
            self.estop_button.classes('w-40 min-h-[3rem]')  # Maintain width and height
        else:
            self.estop_button.props('color=blue')
            self.estop_button.text = 'EMERGENCY STOP'
            self.estop_button.classes('w-40 min-h-[3rem]')  # Maintain width and height

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear.value = x
        self.angular.value = y
        self.cmd_vel_publisher.publish(msg)

    def store_gps(self, msg: GPSFix) -> None:
        """Store the latest GPS message."""
        self.latest_gps = msg

    def store_battery(self, msg: BatteryState) -> None:
        """Store the latest battery state."""
        self.latest_battery = msg
        if self.latest_battery is not None:
            self.battery.text = f'{self.latest_battery.percentage * 100:.1f}% ({self.latest_battery.voltage:.1f}V)'

    def update_gps_ui(self) -> None:
        """Update the UI with the latest GPS data every 2 seconds."""
        if self.latest_gps is not None:
            self.map.set_center((self.latest_gps.latitude, self.latest_gps.longitude))
            self.marker.move(self.latest_gps.latitude, self.latest_gps.longitude)

        # Update bumper states
        try:
            front_top = self.get_parameter('read_data.bumper_front_top_level').value
            front_bottom = self.get_parameter('read_data.bumper_front_bottom_level').value
            back = self.get_parameter('read_data.bumper_back_level').value
            estop1 = self.get_parameter('read_data.estop1_level').value
            estop2 = self.get_parameter('read_data.estop2_level').value

            self.bumper_front_top.text = f'Front Top: {"ACTIVE" if front_top else "inactive"}'
            self.bumper_front_bottom.text = f'Front Bottom: {"ACTIVE" if front_bottom else "inactive"}'
            self.bumper_back.text = f'Back: {"ACTIVE" if back else "inactive"}'
            self.estop1.text = f'E-Stop 1: {"ACTIVE" if estop1 else "inactive"}'
            self.estop2.text = f'E-Stop 2: {"ACTIVE" if estop2 else "inactive"}'

            if not self.params_available:
                self.params_available = True
                self.params_status.text = 'Parameters available'
                self.params_status.classes('text-green-500')
        except rclpy.exceptions.ParameterNotDeclaredException:
            if self.params_available:
                self.params_available = False
                self.params_status.text = 'Waiting for parameters...'
                self.params_status.classes('text-red-500')
            self.bumper_front_top.text = 'Front Top: ---'
            self.bumper_front_bottom.text = 'Front Bottom: ---'
            self.bumper_back.text = 'Back: ---'
            self.estop1.text = 'E-Stop 1: ---'
            self.estop2.text = 'E-Stop 2: ---'


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=9001)
