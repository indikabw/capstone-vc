---
name: rosbag-video-converter
description: Guidelines for recording ROS2 topics into rosbags and converting image topics to MP4 videos with correct real-time playback speed in slow simulation environments.
---

# ROS2 Rosbag Video Recording & Conversion Guide

This skill handles recording image topics to MCAP rosbags and converting them into MP4 videos that play back at exactly 1x simulation real-time speed.

## Prerequisites
* **ROS 2 Environment:** The converting node must be executed in a sourced terminal:
  ```bash
  source /opt/ros/lyrical/setup.bash
  source ~/capstone-vc/install/setup.bash
  ```
* **Python Dependencies:** `cv_bridge` (for ROS/OpenCV image translation) and `rosbag2_py` (for reading bags) must be available.

## Workflow

### 1. Pre-Flight Topic Verification (CRITICAL)
Before starting a recording, you **must** verify that the target topic exists and is actively publishing data. Blindly recording a topic that doesn't exist or isn't bridging correctly will result in empty bags and wasted simulation runs.

Run the following inside the VM to verify:
```bash
# Check if the topic exists and has publishers
ros2 topic info /destination_camera/image_raw

# Verify data is actually flowing (should return 1 message and exit)
ros2 topic echo /destination_camera/image_raw --once > /dev/null
```
If `ros2 topic echo` hangs, the topic is not publishing data. You must fix the Gazebo bridge or simulation setup before recording.

### 2. Recording the Topic
To record a camera or image topic in the background, use `ros2 bag record`.
> [!IMPORTANT]
> Under low-performance environments (e.g., VMs using `llvmpipe` software rendering), always set the camera's update rate to a lower rate (e.g., 2–5Hz) and disable other unused rendering sensors to avoid CPU starvation.

```bash
BAG_NAME="scenario_run_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o $BAG_NAME --topics /destination_camera/image_raw &
BAG_PID=$!
```

### 2. Clean Shutdown & Reindexing
When stopping the recording, use a graceful interrupt. Because `rosbag2` runs in multiple threads, it must be terminated via `SIGINT`:
```bash
pkill -INT -f "rosbag2"
```

If the recorder was terminated before it could finalize the MCAP file footer, the bag will be corrupted (`File end magic is invalid`). You MUST reindex it before attempting conversion:
```bash
ros2 bag reindex <bag_dir_name>
```

### 3. Simulation Real-Time Conversion Script
Do NOT use a hardcoded framerate (like 30fps) for the output video. Doing so will speed up or slow down the playback relative to the actual simulation speed. Instead, calculate the frame rate dynamically based on the difference between the first and last frames' simulation clock timestamps.

Below is the standard python conversion script pattern:

```python
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
    
    image_topic = '/destination_camera/image_raw'
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
```
