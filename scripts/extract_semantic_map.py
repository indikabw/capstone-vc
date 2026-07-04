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
            parts = pose_text.split()
            if len(parts) >= 6:
                semantic_map[name] = {
                    "position": {"x": float(parts[0]), "y": float(parts[1]), "z": float(parts[2])},
                    "orientation": {"roll": float(parts[3]), "pitch": float(parts[4]), "yaw": float(parts[5])},
                    "size": {"dx": 0.0, "dy": 0.0, "dz": 0.0} # Default for includes
                }
                
    # Extract inline models and their geometries
    for model in root.findall(".//model"):
        name = model.get("name")
        pose_elem = model.find("pose")
        
        if name and pose_elem is not None:
            pose_text = pose_elem.text.strip()
            parts = pose_text.split()
            if len(parts) >= 6:
                dx, dy, dz = 0.0, 0.0, 0.0
                
                # Attempt to find geometry
                geom = model.find(".//geometry")
                if geom is not None:
                    box = geom.find("box/size")
                    cylinder_len = geom.find("cylinder/length")
                    cylinder_rad = geom.find("cylinder/radius")
                    
                    if box is not None:
                        sparts = box.text.strip().split()
                        if len(sparts) >= 3:
                            dx, dy, dz = float(sparts[0]), float(sparts[1]), float(sparts[2])
                    elif cylinder_len is not None and cylinder_rad is not None:
                        dz = float(cylinder_len.text.strip())
                        rad = float(cylinder_rad.text.strip())
                        dx, dy = rad * 2, rad * 2

                semantic_map[name] = {
                    "position": {"x": float(parts[0]), "y": float(parts[1]), "z": float(parts[2])},
                    "orientation": {"roll": float(parts[3]), "pitch": float(parts[4]), "yaw": float(parts[5])},
                    "size": {"dx": dx, "dy": dy, "dz": dz}
                }
    
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
