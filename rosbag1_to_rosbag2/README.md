# Convert ROS1 bag to ROS2 bag

Converts all ROS1 .bag files recursively in a directory to ROS2 .bag files.

Build docker image:
```
docker build -t rosbag1_to_rosbag2 .
```

Then log into the converted docker container and run the following command to convert the ROS1 bag to ROS2 bag:
```
docker run -it --rm -v {absolute_path_to_rosbags1}:/rosbags1 -v {absolute_path_to_rosbags2}:/rosbags2 rosbag1_to_rosbag2
```

Example paths in WATcloud:
```
docker run -it --rm -v /mnt/wato-drive2/autodrive_rosbags:/rosbags1 -v /mnt/wato-drive2/rosbags2-tmp:/rosbags2 rosbag1_to_rosbag2
```
