# basekit_ros

notes:

config file: what to do

for eg yaxis canopen motors etc needs ctrl to be enabled before they can move. This is just a thing for weed screw and other zz accessories.

```bash
yaxis_motor.set_ctrl_enable(true)
```

this is missing in the code!

u6 config runs with cmd_vel
to run with 0.2ms

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

to stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## camera

camera.launch uses cam usb

to remotely view it with foxglove:

```bash
ssh -L 8765:localhost:8765 <device_name>
```

## Docker

### Using Docker Compose (recommended)

Build and run the container:

```bash
cd docker
docker compose up --build
```

To run in detached mode:

```bash
docker compose up -d
```

To attach to a running container:

```bash
docker compose exec basekit bash
```

To stop:

```bash
docker compose down
```

### wip: Ff driver

field friend driver is directly added because of the changes that were already made.

source: https://github.com/ATB-potsdam-automation/field_friend_driver

## Development

1. create a virtual environment and activate it:

```bash
virtualenv .venv # or without virtualenv:
python -m venv .venv

source .venv/bin/activate # to activate your virtual environment
```

2. install dependencies:

```bash
pip install -r requirements-dev.txt
```

3. start your project:

```bash
./main.py
```

## pre-commit

[pre-commit](https://pre-commit.com/) is a tool to help you manage and run pre-commit hooks in your code.
It is used to check your code for e.g. extra whitespace or formatting errors before committing it.
Install the pre-commit hooks by running:

```bash
pre-commit install
```

You can also run the hooks manually by running:

```bash
pre-commit run --all-files
```
