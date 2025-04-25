# BaseKit ROS

BaseKit ROS is a comprehensive ROS2 package that handles the communication and configuration of various field friend components:

- Communication with Lizard (ESP32) to control the Field Friend
- GNSS positioning system
- Camera systems (USB and AXIS cameras)

## Components

### Field Friend Driver

The Field Friend driver (based on [ATB Potsdam's field_friend_driver](https://github.com/ATB-potsdam-automation/field_friend_driver)) manages the communication with the ESP32 microcontroller running [Lizard](https://lizard.dev/) firmware - a domain-specific language for defining hardware behavior on embedded systems.

The package provides:

- `config/startup.liz`: Basic Lizard configuration for field friend robot
- `config/field_friend.yaml`: Corresponding ROS2 driver configuration

Basic movement control:
With the field friend driver running, the robot can now listen to cmd_vel commands. For example:

```bash
# Start robot movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Camera System

The camera system supports both USB cameras and AXIS cameras:

#### USB Camera

- Video streaming through ROS2 topics
- Camera configuration via `config/camera.yaml`
- Support for standard USB cameras using the `usb_cam` ROS2 package

#### AXIS Cameras

- Support for AXIS IP cameras using the [ROS2 AXIS camera driver](https://github.com/ros-drivers/axis_camera/tree/humble-devel)
- Multi-camera support with individual streams
- Camera configuration via `config/axis_camera.yaml`

#### Visualization

- Integration with [Foxglove Studio](https://foxglove.dev/) for remote camera viewing using [ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge)
- Support for compressed image transport

### GNSS (Rover)

The GNSS system uses the [Septentrio GNSS driver](https://github.com/septentrio-gnss/septentrio_gnss_driver) with our `config/rover.yaml` configuration. Available topics:

- `/pvtgeodetic`: Position, velocity, and time in geodetic coordinates
- `/poscovgeodetic`: Position covariance in geodetic coordinates
- `/velcovgeodetic`: Velocity covariance in geodetic coordinates
- `/atteuler`: Attitude in Euler angles
- `/attcoveuler`: Attitude covariance
- `/gpsfix`: Detailed GPS fix information including satellites and quality
- `/aimplusstatus`: AIM+ status information

## Docker Setup

### Using Docker Compose (Recommended)

1. Build and run the container:

```bash
cd docker
docker compose up --build
```

2. Run in detached mode:

```bash
docker compose up -d
```

3. Attach to running container:

```bash
docker compose exec basekit bash
```

4. Stop containers:

```bash
docker compose down
```

The Docker setup includes:

- All necessary ROS2 packages
- Lizard communication tools
- Camera drivers
- GNSS drivers
- Development tools

## Development

1. Create and activate virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
```

2. Install dependencies:

```bash
pip install -r requirements-dev.txt
```

3. Start the project:

```bash
./main.py
```

### Code Quality

This project uses pre-commit hooks for code quality:

1. Install pre-commit hooks:

```bash
pre-commit install
```

2. Run hooks manually:

```bash
pre-commit run --all-files
```

## Launch Files

The system can be started using different launch files:

- `basekit.launch.py`: Launches all components
- `field_friend.launch.py`: Launches only Field Friend driver
- `camera_system.launch.py`: Launches complete camera system (USB + AXIS)
- `usb_camera.launch.py`: Launches USB camera only
- `axis_cameras.launch.py`: Launches AXIS cameras only
- `rover.launch.py`: Launches GNSS system

To launch the complete system:

```bash
ros2 launch basekit_launch basekit.launch.py
```
