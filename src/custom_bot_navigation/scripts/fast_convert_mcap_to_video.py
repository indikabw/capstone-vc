import sys
import cv2
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def main():
    if len(sys.argv) < 3:
        print("Usage: convert_on_vm.py <bag_dir> <output.mp4> [topic]")
        sys.exit(1)

    bag_dir = sys.argv[1]
    output_mp4 = sys.argv[2]
    topic = sys.argv[3] if len(sys.argv) > 3 else '/camera/image_raw'
    
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
    
    if topic not in type_map:
        print(f"Topic {topic} not found in bag")
        sys.exit(1)
        
    msg_type_str = type_map[topic]
    msg_type = get_message(msg_type_str)

    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    bridge = CvBridge()
    frames = []
    timestamps = []

    print(f"Reading frames from {bag_dir} ...")
    count = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg = deserialize_message(data, msg_type)
        
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frames.append(cv_img)
        
        # Use header stamp (simulation time) to compute accurate sim-time duration
        sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        timestamps.append(sim_time)
        count += 1

    if count == 0:
        print("No frames found to convert.")
        sys.exit(1)

    # Calculate dynamic FPS based on simulation time span
    duration = timestamps[-1] - timestamps[0]
    if duration > 0:
        fps = count / duration
    else:
        fps = 30.0 # Fallback
    
    print(f"Read {count} frames. Sim duration: {duration:.2f}s. Calculated FPS: {fps:.2f}")

    height, width = frames[0].shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_mp4, fourcc, fps, (width, height))
    
    print(f"Writing to {output_mp4} at {fps:.2f} FPS...")
    for cv_img in frames:
        writer.write(cv_img)
        
    writer.release()
    print(f"Wrote {count} frames to {output_mp4}")

if __name__ == '__main__':
    main()
