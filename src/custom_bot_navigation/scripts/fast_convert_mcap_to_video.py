import sys
import cv2
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    if len(sys.argv) < 3:
        print("Usage: convert_on_vm.py <bag_dir> <output.mp4>")
        sys.exit(1)

    bag_dir = sys.argv[1]
    output_mp4 = sys.argv[2]
    
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_dir,
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if '/destination_camera/image_raw' not in type_map:
        print("Topic /destination_camera/image_raw not found in bag")
        sys.exit(1)
        
    msg_type_str = type_map['/destination_camera/image_raw']
    msg_type = get_message(msg_type_str)

    storage_filter = rosbag2_py.StorageFilter(topics=['/destination_camera/image_raw'])
    reader.set_filter(storage_filter)

    bridge = CvBridge()
    writer = None

    print(f"Converting {bag_dir} to {output_mp4} ...")
    count = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg = deserialize_message(data, msg_type)
        
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if writer is None:
            height, width = cv_img.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            writer = cv2.VideoWriter(output_mp4, fourcc, 30.0, (width, height))
            
        writer.write(cv_img)
        count += 1
        
    if writer is not None:
        writer.release()
    print(f"Wrote {count} frames to {output_mp4}")

if __name__ == '__main__':
    main()
