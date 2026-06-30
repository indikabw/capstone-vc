#!/usr/bin/env python3
import sys
import cv2
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 convert.py <bag_dir> <output.mp4>")
        sys.exit(1)

    bag_dir = sys.argv[1]
    output_mp4 = sys.argv[2]
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    image_topic = '/camera/image_raw'
    if image_topic not in type_map:
        print(f"Topic {image_topic} not found in bag")
        sys.exit(1)
        
    msg_type = get_message(type_map[image_topic])
    storage_filter = rosbag2_py.StorageFilter(topics=[image_topic])
    reader.set_filter(storage_filter)

    bridge = CvBridge()
    frames = []
    timestamps = []

    print(f"Reading frames from {bag_dir} ...")
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg = deserialize_message(data, msg_type)
        
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frames.append(cv_img)
        
        # Calculate simulation time stamp from the message header
        sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        timestamps.append(sim_time)

    if not frames:
        print("No frames found to convert.")
        sys.exit(1)

    # Compute simulation clock duration to set exact 1x playback speed
    duration = timestamps[-1] - timestamps[0]
    fps = len(frames) / duration if duration > 0 else 30.0
    
    print(f"Frames: {len(frames)}, Duration: {duration:.2f}s, Target FPS: {fps:.2f}")

    height, width = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_mp4, fourcc, fps, (width, height))
    
    for cv_img in frames:
        writer.write(cv_img)
        
    writer.release()
    print(f"Video saved successfully to {output_mp4}")

if __name__ == '__main__':
    main()
