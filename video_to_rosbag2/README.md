# Convert video to ROS2 bag

Convert a video (e.g. .mp4, .webm) to rosbag2 format.

Build docker image:
```
docker build -t video_to_rosbag2_converter .
```

Run the following command to convert the video with source path `{absolute_path_to_video}` to rosbag2 with destination path `{absolute_path_to_rosbag}`:

```
docker run -it --rm -v {absolute_path_to_video_file}:/video -v {absolute_path_to_destination_folder}:/rosbag2 video_to_rosbag2_converter
```

Example 
```
docker run -it --rm -v /mnt/wato-drive2/perception_datasets/driving_in_waterloo/driving_waterloo_1920_1080.mp4:/video/video.mp4 -v /mnt/wato-drive2/perception_datasets/driving_in_waterloo/:/rosbag2 video_to_rosbag2_converter
```