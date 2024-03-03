import os
import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.info("Starting conversion")
print("Starting conversion")
# Environment variables for input and output directories
input_dir = "/input/"

# Directory setup
lidar_dir = "/output/lidar_data"
camera_dir = "/output/camera_images"
os.makedirs(lidar_dir, exist_ok=True)
os.makedirs(camera_dir, exist_ok=True)

bridge = CvBridge()

def extract_data(bag_file):
    storage_options = StorageOptions(uri=bag_file, storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get all topic names and types from the bag file
    all_topics_and_types = reader.get_all_topics_and_types()

    # Filter topics for PointCloud2 and Image messages
    lidar_topics = [topic.name for topic in all_topics_and_types if topic.type == 'sensor_msgs/msg/PointCloud2']
    camera_topics = [topic.name for topic in all_topics_and_types if topic.type == 'sensor_msgs/msg/Image']
    print(f"Lidar topics: {lidar_topics}")
    print(f"Camera topics: {camera_topics}")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        topic_filename = topic.replace("/", "_")
        if topic in lidar_topics:
            pc2_msg = deserialize_message(data, PointCloud2)
            # Convert and save PointCloud2 data as PCD
            o3d_pc = pointcloud2_to_o3d(pc2_msg)
            filename = os.path.join(lidar_dir, f"{topic_filename}_{t}.pcd")
            o3d.io.write_point_cloud(filename, o3d_pc)
        elif topic in camera_topics:
            image_msg = deserialize_message(data, Image)
            # Convert and save Image data as JPG
            filename = os.path.join(camera_dir, f"{topic_filename}_{t}.jpg")
            save_image(image_msg, filename)

    del reader  # Ensure to properly delete the reader object to close the bag file and release resources

def pointcloud2_to_o3d(msg):
    # Placeholder for actual conversion logic
    # Ensure this function correctly converts a ROS2 PointCloud2 message to an Open3D point cloud
    cloud_generator = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Create a structured numpy array directly from the generator
    cloud_points = np.array(list(cloud_generator), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    
    # Initialize an empty numpy array of shape (len(cloud_points), 3) and type float64
    np_points = np.zeros((len(cloud_points), 3), dtype=np.float64)

    # Fill the numpy array with data from the structured array
    np_points[:, 0] = cloud_points['x']
    np_points[:, 1] = cloud_points['y']
    np_points[:, 2] = cloud_points['z']

    print("shape is ", np_points.shape)
    

    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    print(np_points)

    # Assign points to Open3D PointCloud
    pcd.points = o3d.utility.Vector3dVector(np_points.copy())
    return pcd

def save_image(msg, filename):
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # Adjust encoding if necessary
    cv2.imwrite(filename, cv_image)

# Example usage
# extract_data("path_to_your_bag_file.bag")
# Find all directories in input_dir and run extract data
logger.info(f"Extracting data from {input_dir}")
print(f"Extracting data from {input_dir}")
for bag_file in os.listdir(input_dir):
    logger.info(f"Extracting data from {bag_file}")
    print(f"Extracting data from {bag_file}")
    extract_data(os.path.join(input_dir, bag_file))