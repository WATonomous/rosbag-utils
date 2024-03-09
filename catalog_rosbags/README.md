# Catalog rosbags

Lists out rosbags in a path (`/mnt/wato-drive2/rosbags2`) by message type.

Example for autodrive rosbags are saved in `autodrive_rosbags_output/` containing `camera_bags.csv` and `lidar_bags.csv`.

## Usage
```
docker build -t catalog_rosbags . && docker run -v /mnt/wato-drive2/rosbags2:/rosbags2 catalog_rosbags
```