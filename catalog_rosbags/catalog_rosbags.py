import os
import yaml
import pandas as pd

ros_bags_dir = "/rosbags2"
topic_types = set()

# Function to parse metadata.yaml files and extract information according to the given format
def parse_metadata(metadata_path):
    with open(metadata_path, 'r') as file:
        metadata = yaml.safe_load(file)
    # Convert from ns to s
    duration_seconds = metadata['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9 if metadata['rosbag2_bagfile_information']['duration']['nanoseconds'] else 0
    message_count = metadata['rosbag2_bagfile_information']['message_count']
    topics_with_message_count = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    return {
        'duration': duration_seconds,
        'message_count': message_count,
        'topics_with_message_count': topics_with_message_count
    }

def find_rosbags_with_messages(ros_bags_dir, message_patterns):
    bags_info = []
    for root, dirs, files in os.walk(ros_bags_dir):
        print(dirs, files)
        if 'metadata.yaml' in files:
            metadata_path = os.path.join(root, 'metadata.yaml')
            metadata = parse_metadata(metadata_path)
            print(files, metadata['topics_with_message_count'])
            for topic in metadata['topics_with_message_count']:
                topic_type = topic['topic_metadata']['type']
                print(topic_type)
                topic_types.add(topic_type)
                for pattern in message_patterns:
                    if pattern in topic_type and metadata['message_count'] is not None and metadata['message_count'] > 0:
                        bag_info = {
                            'path': root,
                            'topic_name': topic['topic_metadata']['name'],
                            'message_type': topic_type,
                            'rosbag_duration': metadata['duration'],
                            'message_count': metadata['message_count'],
                        }
                        bags_info.append(bag_info)
    return bags_info

camera_bags = find_rosbags_with_messages(ros_bags_dir, ["sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/CompressedImage"])
lidar_bags = find_rosbags_with_messages(ros_bags_dir, ["sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/LaserScan", "velodyne_msgs/msg/VelodyneScan"])

camera_bags_df = pd.DataFrame(camera_bags)
lidar_bags_df = pd.DataFrame(lidar_bags)

# sort each df by message_count
camera_bags_df = camera_bags_df.sort_values(by='message_count', ascending=False)
lidar_bags_df = lidar_bags_df.sort_values(by='message_count', ascending=False)

print(camera_bags_df)
print(lidar_bags_df)
print(topic_types)
camera_bags_df.to_csv('/rosbags2/camera_bags_catalog.csv', index=False)
lidar_bags_df.to_csv('/rosbags2/lidar_bags_catalog.csv', index=False)