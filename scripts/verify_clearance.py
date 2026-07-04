#!/usr/bin/env python3
import subprocess
import sys
import re
import math
import argparse
try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    print("scipy is required. Please install it using: pip3 install scipy")
    sys.exit(1)

def get_gz_model_pose(model_name, link_name=None):
    """
    Retrieves the pose of a model (or link) from Gazebo using gz model CLI.
    Returns (pos, quat) where pos = [x, y, z] and quat = [x, y, z, w].
    Raises Exception if model not found or parsing fails.
    """
    cmd = ["gz", "model", "-m", model_name, "-p"]
    if link_name:
        cmd = ["gz", "model", "-m", model_name, "-l", link_name, "-p"]
        
    try:
        # Note: Depending on the Gazebo version, setup.bash needs to be sourced,
        # but we assume this script is run in an environment where `gz model` works.
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
            raise Exception(f"Command failed: {result.stderr}")
    except Exception as e:
        raise Exception(f"Failed to execute gz model: {e}")

    output = result.stdout
    
    # Parse protobuf-like output
    # Example format:
    # position { x: 1.0 y: 2.0 z: 3.0 }
    # orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }
    
    pos_x = re.search(r'position\s*\{.*?x:\s*([^\s}]+)', output, re.DOTALL)
    pos_y = re.search(r'position\s*\{.*?y:\s*([^\s}]+)', output, re.DOTALL)
    pos_z = re.search(r'position\s*\{.*?z:\s*([^\s}]+)', output, re.DOTALL)
    
    ori_x = re.search(r'orientation\s*\{.*?x:\s*([^\s}]+)', output, re.DOTALL)
    ori_y = re.search(r'orientation\s*\{.*?y:\s*([^\s}]+)', output, re.DOTALL)
    ori_z = re.search(r'orientation\s*\{.*?z:\s*([^\s}]+)', output, re.DOTALL)
    ori_w = re.search(r'orientation\s*\{.*?w:\s*([^\s}]+)', output, re.DOTALL)
    
    if not all([pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w]):
        raise Exception(f"Could not parse pose from output:\n{output}")
        
    pos = [float(pos_x.group(1)), float(pos_y.group(1)), float(pos_z.group(1))]
    quat = [float(ori_x.group(1)), float(ori_y.group(1)), float(ori_z.group(1)), float(ori_w.group(1))]
    
    return pos, quat

def compute_lowest_z(pos, quat, radius, length):
    """
    Computes the absolute lowest Z coordinate of a cylinder given its center pose, radius, and length.
    """
    rot = R.from_quat(quat)
    rot_matrix = rot.as_matrix()
    
    # The local Z-axis of the cylinder in world coordinates
    v_z = rot_matrix[:, 2]
    
    # The offset to the lowest point relative to the center
    # This accounts for the vertical extent (length/2) and the radial extent
    # min_z_offset = - (L/2) * abs(v_z_z) - R * sqrt(1 - v_z_z^2)
    v_z_z = v_z[2]
    radial_component = radius * math.sqrt(max(0, 1.0 - v_z_z**2))
    axial_component = (length / 2.0) * abs(v_z_z)
    
    min_z_offset = - axial_component - radial_component
    
    lowest_z = pos[2] + min_z_offset
    return lowest_z

def get_euclidean_distance(pos1, pos2):
    return math.sqrt(sum((a - b)**2 for a, b in zip(pos1, pos2)))

def main():
    parser = argparse.ArgumentParser(description="Verify if a cylinder is lifted off the floor.")
    parser.add_argument("--cylinder_model", default="red_cylinder", help="Gazebo model name of the cylinder")
    parser.add_argument("--robot_model", default="custom_bot", help="Gazebo model name of the robot")
    parser.add_argument("--robot_link", default="link5", help="Gazebo link name of the robot's end effector")
    parser.add_argument("--radius", type=float, default=0.015, help="Radius of the cylinder (meters)")
    parser.add_argument("--length", type=float, default=0.3, help="Length of the cylinder (meters)")
    parser.add_argument("--floor_z", type=float, default=0.0, help="Z height of the floor")
    parser.add_argument("--clearance_threshold", type=float, default=0.015, help="Minimum clearance from floor (meters) to be considered lifted")
    parser.add_argument("--anchor_threshold", type=float, default=0.20, help="Maximum distance between cylinder center and gripper (meters) to be considered anchored")
    args = parser.parse_args()
    
    try:
        print(f"Querying pose for cylinder: {args.cylinder_model}")
        cyl_pos, cyl_quat = get_gz_model_pose(args.cylinder_model)
        
        print(f"Cylinder Pose: Pos: {cyl_pos}, Quat: {cyl_quat}")
        
        lowest_z = compute_lowest_z(cyl_pos, cyl_quat, args.radius, args.length)
        print(f"Calculated Lowest Z of Cylinder: {lowest_z:.4f} m (Floor is at {args.floor_z} m)")
        
        is_clear = (lowest_z > args.floor_z + args.clearance_threshold)
        
        print(f"Querying pose for robot end effector: model={args.robot_model}, link={args.robot_link}")
        # Sometimes the robot model name varies or gripper link is different, handle graceful fallback if link query fails
        ee_pos, _ = get_gz_model_pose(args.robot_model, args.robot_link)
        
        dist = get_euclidean_distance(cyl_pos, ee_pos)
        print(f"Distance between cylinder and gripper: {dist:.4f} m (Threshold: {args.anchor_threshold} m)")
        
        is_anchored = (dist <= args.anchor_threshold)
        
        if is_clear and is_anchored:
            print("\n[SUCCESS] The cylinder is definitively lifted off the floor and anchored to the robot.")
            sys.exit(0)
        else:
            reasons = []
            if not is_clear:
                reasons.append(f"Lowest point ({lowest_z:.4f}m) is not strictly above floor clearance ({args.floor_z + args.clearance_threshold:.4f}m).")
            if not is_anchored:
                reasons.append(f"Cylinder is too far from gripper ({dist:.4f}m > {args.anchor_threshold}m). Gazebo physics glitch or dropped.")
            
            print("\n[FAILURE] Cylinder is NOT successfully lifted.")
            for r in reasons:
                print(f" - {r}")
            sys.exit(1)
            
    except Exception as e:
        print(f"\n[ERROR] Verification script failed to execute properly: {e}")
        # Return 2 for execution error vs 1 for validation failure
        sys.exit(2)

if __name__ == "__main__":
    main()
