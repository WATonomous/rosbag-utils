import cv2
import rclpy
import glob
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import os

# Constants
TOPIC_NAME = '/camera/center/image_color'
TOPIC_TYPE = 'sensor_msgs/msg/Image'
SOURCE_VIDEO_PATH = '/video'  # Adjust this path as needed
DESTINATION_BAG_PATH = '/rosbag2'  # Adjust this path as needed

def video_to_rosbag2(video_path, bag_path):
    # Initialize ROS 2
    rclpy.init(args=None)

    # Create a CvBridge to convert OpenCV images to ROS messages
    bridge = CvBridge()

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Cannot open video file {video_path}")

    # Set up the ROS 2 bag writer
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # Define the topic metadata for the image data
    topic_name = TOPIC_NAME
    topic_type = TOPIC_TYPE
    msg_type = get_message(topic_type)
    topic_metadata = TopicMetadata(name=topic_name, type=topic_type, serialization_format='cdr')
    writer.create_topic(topic_metadata)

    frame_id = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the OpenCV image to a ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_image.header.frame_id = str(frame_id)
        ros_image.header.stamp.sec = int(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000)

        # Serialize the ROS message and write it to the bag
        serialized_msg = serialize_message(ros_image)
        writer.write(topic_name, serialized_msg, ros_image.header.stamp.sec * 1_000_000_000 + ros_image.header.stamp.nanosec)

        # Log important info
        print(f"Writing frame {frame_id} to {bag_path}")

        frame_id += 1

    # Release resources
    cap.release()
    rclpy.shutdown()

def convert_all_videos(source_path, destination_path):
    # Find all .mp4 and .webm files in the source directory
    video_files = glob.glob(f"{source_path}/*.mp4") + glob.glob(f"{source_path}/*.webm")

    for video_file in video_files:
        # Generate a unique bag file path for each video
        bag_file_name = os.path.splitext(os.path.basename(video_file))[0]
        bag_path = os.path.join(destination_path, bag_file_name)

        print(f"Converting {video_file} to {bag_path}")
        video_to_rosbag2(video_file, bag_path)

if __name__ == "__main__":
    convert_all_videos(SOURCE_VIDEO_PATH, DESTINATION_BAG_PATH)