FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt update && apt install -y ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-opencv \
    ros-${ROS_DISTRO}-rosbag2-py
RUN pip3 install opencv-python-headless 
WORKDIR /app

COPY video_to_rosbag2.py .

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash &&  python3 video_to_rosbag2.py"]