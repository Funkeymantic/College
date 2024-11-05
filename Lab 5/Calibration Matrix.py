import cv2
import numpy as np
import json
import time
import sys
import subprocess
import os

sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

system_python = "/usr/bin/python3"
robot_ip = '172.29.208.124'
beaker = Robot(robot_ip)

# Define z-heights for the Beaker robot
z1, z11 = -30.0, -125.0
Home = [0.0, 0.0, 0.0, 0.0, -90, 30]
Die = [468, -5, -185, -179.9, 0.0, 30.0]
dice_beaker_cords: list = []

# Define 3x3 grid of dice positions within the boundary points
dice_positions = [
    ([283.75, 467.83, z1], [283.75, 467.83, z11]),
    ([467.5, 467.83, z1], [467.5, 467.83, z11]),
    ([651.25, 467.83, z1], [651.25, 467.83, z11]),
    ([283.75, 686.67, z1], [283.75, 686.67, z11]),
    ([467.5, 686.67, z1], [467.5, 686.67, z11]),
    ([651.25, 686.67, z1], [651.25, 686.67, z11]),
    ([283.75, 905.5, z1], [283.75, 905.5, z11]),
    ([467.5, 905.5, z1], [467.5, 905.5, z11]),
    ([651.25, 905.5, z1], [651.25, 905.5, z11])
]



# Function to retrieve robot coordinates
def get_robot_cords(robot_ip):
    robot = Robot(robot_ip)
    return robot.read_current_cartesian_pose()[:2]  # Get x, y coordinates only

# Function to capture image coordinates and compute the transformation matrix
def get_image_cords(image_path, robot_ip, output_file):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Image could not be loaded.")
        return None
    if not os.path.exists(image_path):
        print(f"Error: Image file {image_path} does not exist.")
        return None

    
    # Process image to find dice contours
    image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    image_hsv_blur = cv2.GaussianBlur(image_hsv, (5, 5), 1)
    lower_yellow = np.array([28, 255, 150])
    upper_yellow = np.array([50, 255, 255])
    mask = cv2.inRange(image_hsv_blur, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    dice_image_cords, dice_robot_cords = [], []
    
    # Process each contour to get dice coordinates
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:
            continue
        moment = cv2.moments(cnt)
        cx, cy = int(moment['m10'] / moment['m00']), int(moment['m01'] / moment['m00'])
        dice_image_cords.append([cx, cy])
        dice_robot_cords.append(get_robot_cords(robot_ip))

    # Calculate and save the transformation matrix
    src_cords, dst_cords = np.array(dice_image_cords), np.array(dice_robot_cords)
    transform_matrix, _ = cv2.findHomography(src_cords, dst_cords, cv2.RANSAC, confidence=0.9)
    transform_data = {
        "image_coords": dice_image_cords,
        "robot_coords": dice_robot_cords,
        "transformation_matrix": transform_matrix.tolist()
    }
    with open(output_file, "w") as f:
        json.dump(transform_data, f, indent=4)
    print(f"Transformation matrix and coordinates saved to {output_file}")
    return transform_matrix

# Main function
def main():
    beaker = Robot(robot_ip)  # Instantiate the Robot instance
    beaker.write_joint_pose(Home)  # Move robot to home position

    for i, (dice, dice_high) in enumerate(dice_positions, start=1):
        beaker.schunk_gripper('open')
        beaker.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])
        beaker.write_cartesian_position(Die)
        beaker.schunk_gripper('close')
        beaker.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])
        beaker.write_cartesian_position(dice)
        beaker.write_cartesian_position(dice_high)
        beaker.schunk_gripper('open')
        current_robot_coordinates = get_robot_cords(robot_ip)
        print(f"Robot coordinates: {current_robot_coordinates}")
        dice_beaker_cords.append(current_robot_coordinates)
        beaker.write_cartesian_position(dice)
        print(f"Dice {i} is done!")

    beaker.write_joint_pose(Home)

    current_robot_coordinates = get_robot_cords(beaker)


    output_file = "transform_data.json"
    
    # Run Dice_Identification.py to capture image
    try:
        subprocess.run([system_python, "/home/funkey/ME-559/College/Lab 5/Dice_Identification.py"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running Dice_Identification.py: {e}")

    
    # Process the captured image and save the transformation matrix
    get_image_cords(
        image_path="/home/funkey/ME-559/College/Lab 5/dice.png",
        robot_ip=robot_ip,
        output_file=output_file
        
    )
    
    

if __name__ == '__main__':
    main()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        