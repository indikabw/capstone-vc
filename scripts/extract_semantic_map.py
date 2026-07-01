import xml.etree.ElementTree as ET
import json
import sys
import os

def extract_map(world_path, output_path):
    if not os.path.exists(world_path):
        print(f"Error: Could not find {world_path}")
        sys.exit(1)

    tree = ET.parse(world_path)
    root = tree.getroot()
    
    # In SDF, includes are usually under <world> -> <include>
    semantic_map = {}
    
    for include in root.findall(".//include"):
        name_elem = include.find("name")
        pose_elem = include.find("pose")
        
        if name_elem is not None and pose_elem is not None:
            name = name_elem.text.strip()
            pose_text = pose_elem.text.strip()
            # pose is "x y z roll pitch yaw"
            parts = pose_text.split()
            if len(parts) >= 6:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                roll = float(parts[3])
                pitch = float(parts[4])
                yaw = float(parts[5])
                
                semantic_map[name] = {
                    "position": {"x": x, "y": y, "z": z},
                    "orientation": {"roll": roll, "pitch": pitch, "yaw": yaw}
                }
                
    # Also extract light sources or other models if necessary, but includes cover most objects
    
    # Make sure output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    with open(output_path, 'w') as f:
        json.dump(semantic_map, f, indent=2)
        
    print(f"Extracted {len(semantic_map)} objects to {output_path}")

if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_file = os.path.join(base_dir, "src", "custom_bot_gazebo", "worlds", "small_house.world")
    output_file = os.path.join(base_dir, "src", "custom_bot_reasoning", "resource", "semantic_map.json")
    
    extract_map(world_file, output_file)
