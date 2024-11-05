import json
import paho.mqtt.client as mqtt
import subprocess
import time
import numpy as np
import cv2 as cv
import sys
import os

# Path adjustments if needed
sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

# MQTT Broker details
BROKER_ADDRESS = "broker_ip_address"  # Replace with actual broker address
TOPIC = "robot/status"

# MQTT Client setup
client = mqtt.Client()
client.connect(BROKER_ADDRESS)

# Robot setup
robot_ip = '172.29.208.124'  # Beaker
robot = Robot(robot_ip)
Home = [0, 0, 0, 0, -90, 30]
Stack_Position = [468, -5, -185, -179.9, 0.0, 30.0]
dice_size = 80  # 80mm wide dice

# Variables to store state
Standby = False
Complete = False
Ready = False
Second = False

# Publish messages to MQTT
def publish_status(status):
    message = json.dumps({"status": status})
    client.publish(TOPIC, message)
    print(f"Published message: {message}")

# Run the Image Capture code
def run_image_capture():
    subprocess.run([sys.executable, "/home/funkey/ME-559/College/Lab 5/Image_Capture.py"])
    

# Run the Image Coordinates detection code
def run_image_coords():
    subprocess.run([sys.executable, "/home/funkey/ME-559/College/Lab 5/Image_Coords.py"])
    

# Load the pixel coordinates from Image_Coords JSON output
def load_pixel_coordinates(json_file="dice_positions.json"):
    with open(json_file, "r") as f:
        pixel_data = json.load(f)
    return pixel_data

# Load the transformation matrix
def load_transform_matrix(matrix_file="transform_matrix.json"):
    with open(matrix_file, "r") as f:
        matrix = json.load(f)
    return np.array(matrix, dtype="float32")

# Transform pixel coordinates to robot coordinates
def transform_pixel_to_robot(pixel_coord, homography_matrix):
    pixel_np = np.array([[pixel_coord]], dtype="float32")
    transformed_coord = cv.perspectiveTransform(pixel_np, homography_matrix)
    return transformed_coord[0][0]

# Filter dice based on section
def filter_dice_by_section(dice_data, section_bounds):
    x_min, y_min, x_max, y_max = section_bounds
    return [die for die in dice_data if x_min <= die["pixel_center"]["x"] <= x_max and y_min <= die["pixel_center"]["y"] <= y_max]

# Move robot to stack dice at specified location, accounting for rotation
def stack_dice(robot, dice_robot_coords, start_z=-185):
    for i, die in enumerate(dice_robot_coords):
        x, y = die['robot_position']['x'], die['robot_position']['y']
        rotation = die["rotation"]
        
        # Move to dice position and adjust orientation
        robot.write_cartesian_position([x, y, start_z, rotation, 0.0, 30.0])
        robot.schunk_gripper("close")
        
        # Move to stacking position, incrementing height for each dice
        robot.write_cartesian_position([Stack_Position[0], Stack_Position[1], start_z + i * dice_size, Stack_Position[3], Stack_Position[4], Stack_Position[5]])
        robot.schunk_gripper("open")
        time.sleep(0.25)

# Callback for MQTT message reception
def on_message(client, userdata, message):
    global standby_received
    payload = json.loads(message.payload.decode())
    if payload["status"] == "Standby":
        Standby = True
    elif payload["status"] == "Complete":
        Complete = True
    elif payload["status"] == "Ready":
        Ready = True

client.on_message = on_message
client.subscribe(TOPIC)

# Main execution function
def main():
    client.loop_start()
    robot.write_joint_pose(Home)
    publish_status("ready")

    while not Ready:
        time.sleep(0.25)

    # Step 1: Run Image Capture and Coordinates Detection
    run_image_capture()
    run_image_coords()

    # Step 2: Load pixel coordinates and transform matrix
    pixel_data = load_pixel_coordinates("dice_positions.json")
    homography_matrix = load_transform_matrix("transform_matrix.json")

    # Define section bounds and filter dice in section 1
    section_1_bounds = (0, 0, 1000, 1000)  # Define section bounds as needed
    section_1_dice = filter_dice_by_section(pixel_data, section_1_bounds)

    # Transform section 1 dice to robot coordinates
    section_1_robot_coords = []
    for die in section_1_dice:
        robot_position = transform_pixel_to_robot(die["pixel_center"], homography_matrix)
        section_1_robot_coords.append({
            "robot_position": {"x": robot_position[0], "y": robot_position[1]},
            "rotation": die["rotation"]
        })

    # Stack section 1 dice
    stack_dice(robot, section_1_robot_coords)
    time.sleep(.5)
    while not Standby:
        publish_status("standby")
    
    time.sleep(0.5)
    # Wait at home position until receiving 'complete' or proceed with section 2
    while Standby and not Complete:
        robot.write_joint_pose(Home)
        time.sleep(1)
        Second = True

    # Step 3: If "complete" is received, capture new image and process section 2
    if Second:
        run_image_capture()
        run_image_coords()
        pixel_data = load_pixel_coordinates("dice_positions.json")
        section_2_bounds = (1000, 0, 2000, 1000)  # Define section bounds for section 2

        # Filter dice in section 2, even-numbered if "standby" wasn't received initially
        section_2_dice = filter_dice_by_section(pixel_data, section_2_bounds)
        publish_status("Complete")
    if not Standby and not Second:
        section_2_dice = [die for i, die in enumerate(section_2_dice) if i % 2 == 0]

        # Transform section 2 dice to robot coordinates
        section_2_robot_coords = []
        for die in section_2_dice:
            robot_position = transform_pixel_to_robot(die["pixel_center"], homography_matrix)
            section_2_robot_coords.append({
                "robot_position": {"x": robot_position[0], "y": robot_position[1]},
                "rotation": die["rotation"]
            })

        # Stack section 2 dice
        stack_dice(robot, section_2_robot_coords)
        publish_status("Complete")
        robot.write_joint_pose(Home)
        while not Complete:
            time.sleep(.25)

    # Final status update
    
    client.loop_stop()

if __name__ == "__main__":
    main()
