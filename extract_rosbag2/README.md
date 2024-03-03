# Extract data from a rosbag2 file

Given a ROS 2 bag, finds all topics with camera (sensor_msgs/msg/Image) and lidar (sensor_msgs/msg/PointCloud2) messages and saves them into .jpg and .pcd files respectively.

## Build docker image

```bash
docker build -t extract_rosbag2 .
```

## Run the following command to extract the camera and lidar data from the rosbag2 file

The destination folder will create a folder with /camera and /lidar subfolders containing the extracted data.
```bash
docker run -it -v {absolute_path_to_rosbag2}:/input/rsbag -v {absolute_path_to_destination_folder}:/output extract_rosbag2
```

Example:
```
docker run  -v  /mnt/wato-drive2/rosbags2-tmp/year3/test_track_days/W20.4/traffic_light1/:/input/traffic_light1 -v /home/j89leung/data-test:/output/ extract_rosbag2
```