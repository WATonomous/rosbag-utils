# rosbag-utils

This repository contains a set of Docker containers to work with rosbag files:
- `video_to_rosbag2`: Convert a video (e.g. .mp4, .webm) to rosbag2 format.
- `extract_rosbag2`: Extract camera and lidar scans from a rosbag to .bin and .png files.
- `rosbag1_to_rosbag2`: Converts all ROS1 .bag files recursively in a directory to ROS2 .bag files.