FROM ros:humble

RUN apt-get update && apt-get install -y \
        ros-humble-domain-bridge \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-foxglove-bridge \
        ros-humble-xacro \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-msgs \
        ros-humble-tf-transformations \
        ros-humble-septentrio-gnss-driver \
        ros-humble-usb-cam \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-image-tools \
        ros-humble-compressed-image-transport \
        ros-humble-axis-camera \
        python3-pip \
        python3-serial \
        python3-requests \
        python3-yaml \
        git \
        nano \
        wget \
        iputils-ping

# Setup lizard
RUN mkdir -p /root/.lizard && \
    cd /root && \
    git clone https://github.com/zauberzeug/lizard.git && \
    cp lizard/flash.py /root/.lizard/flash.py && \
    cp lizard/esp.py /root/.lizard/esp.py && \
    chmod +x /root/.lizard/flash.py && \
    cd lizard && \
    pip install -r requirements.txt && \
    cd .. && \
    echo "export PYTHONPATH=/root/.lizard:$PYTHONPATH" >> /root/.bashrc

# https://github.com/zauberzeug/nicegui/issues/2354
RUN python3 -m pip install fastapi==0.108.0
RUN python3 -m pip install setuptools==58.2.0
RUN python3 -m pip install nicegui

COPY ./docker/bashrc /tmp/bashrc
RUN cat /tmp/bashrc >> /root/.bashrc && rm -f /tmp/bashrc

# Copy ROS packages
COPY ./basekit_driver /workspace/src/basekit_driver
COPY ./basekit_launch /workspace/src/basekit_launch
COPY ./basekit_ui /workspace/src/basekit_ui

# Build ROS packages
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select basekit_driver basekit_launch basekit_ui --symlink-install && \
    . install/setup.sh

WORKDIR /workspace
