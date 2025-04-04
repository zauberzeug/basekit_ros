FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
        ros-humble-domain-bridge \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-foxglove-bridge \
        ros-humble-xacro \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-msgs \
        python3-pip \
        git \
        nano \
        wget

COPY ./basekit/requirements-dev.txt /tmp/requirements.txt
RUN python3 -m pip install -r /tmp/requirements.txt && rm -f /tmp/requirements.txt
# https://github.com/zauberzeug/nicegui/issues/2354
RUN python3 -m pip install fastapi==0.108.0
RUN python3 -m pip install setuptools==58.2.0

COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

WORKDIR /workspace