FROM arm64v8/ros:humble-ros-base

# Systeemtools installeren
RUN apt update && apt install -y \
    curl nano gnupg2 lsb-release
RUN apt install wget

RUN rm /etc/apt/sources.list.d/ros2-latest.list

# ROS GPG key en repository toevoegen
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list\
    sudo rm /etc/apt/sources.list.d/ros*.list

# ROS extra packages installeren
RUN apt update && apt install -y \
    ros-humble-irobot-create-msgs
    

RUN apt update && apt install -y \
    ros-humble-depthai-ros

# Workspace aanmaken en create3_examples clonen
WORKDIR /root
RUN mkdir -p /root/create3_ws/src && \
    cd /root/create3_ws/src && \
    git clone https://github.com/iRobotEducation/create3_examples.git && \
    cd /root/create3_ws/src/create3_examples/ && \
    rm -rf create3_coverage create3_lidar_slam create3_republisher

# ROS packages installeren via rosdep
WORKDIR /root/create3_ws

# ROSDEP initialiseren
RUN rosdep init || true
RUN rosdep update

RUN git clone https://github.com/ros2/teleop_twist_keyboard.git src/teleop_twist_keyboard

RUN . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build





# Source automatisch de workspace
RUN echo "source /root/create3_ws/install/setup.bash" >> /root/.bashrc
RUN apt-get install -y python3 python3-pip
RUN pip install opencv-python numpy depthai
RUN apt install ros-humble-sensor-msgs-py

# Optioneel: voeg workspace toe
WORKDIR /root

COPY depth2scan.py /root/
COPY depth2PointCloud.py /root/
COPY OakRgbCam.py /root/
