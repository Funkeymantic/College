import json
import sys
sys.path.append("/home/funkey/ME-559/College/myenv/lib/python3.12/site-packages")
import paho.mqtt.client as mqtt
import subprocess
import time
import numpy as np
import cv2 as cv
import os

# Path adjustments if needed
sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

# MQTT Broker details
BROKER_ADDRESS = "172.29.208.31"
# BROKER_ADDRESS = "localhost"
TOPICA = "Beaker/status"
TOPICB = "Bunsen/status"

# MQTT Client setup
client = mqtt.Client()
client.connect(BROKER_ADDRESS, 1883)

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
O = 0

# Publish messages to MQTT
def publish_status(status):
    message = json.dumps({"status": status})
    client.publish(TOPICA, message)
    print(f"Published message: {message}")

# Update paths as needed
def run_image_capture():
    subprocess.run(["/usr/bin/python3", "/home/funkey/ME-559/College/Lab 5/Image_Capture.py"])

def run_image_coords():
    subprocess.run(["/usr/bin/python3", "/home/funkey/ME-559/College/Lab 5/Image_Coords.py"])


# Load the pixel coordinates from Image_Coords JSON output
def load_pixel_coordinates(json_file="dice_positions.json"):
    with open(json_file, "r") as f:
        pixel_data = json.load(f)
    return pixel_data

# Load the transformation matrix
def load_transform_matrix(matrix_file="transform_matrix.json"):
    # os.chdir('./Lab 5')
    with open(matrix_file, "r") as f:
        matrix = json.load(f)
    return np.array(matrix, dtype="float32")

# Transform pixel coordinates to robot coordinates
def transform_pixel_to_robot(pixel_coord, homography_matrix):
    # Convert pixel coordinates to the appropriate shape
    pixel_np = np.array([[[pixel_coord['x'], pixel_coord['y']]]], dtype="float32")
    transformed_coord = cv.perspectiveTransform(pixel_np, homography_matrix)
    return transformed_coord[0][0]

# Move robot to stack dice at specified location, accounting for rotation
def stack_dice(robot, dice_robot_coords, start_z=-30):
    global O, Second
    
    Offset = 0
    if Second == False:
        Offset = 80
    else:
        Offset = -80
    for i, die in enumerate(dice_robot_coords):
        x, y = die['robot_position']['x'], die['robot_position']['y']
        rotation = die["rotation"]
        
        if y <= 1135:
            print(f"Moving Dice: {i} to dice position at ({x}, {y}, {start_z}) with rotation {rotation}")
            try:
                robot.write_cartesian_position([x, y, start_z, -179.9, 0.0, rotation+30.0])
                robot.write_cartesian_position([x, y, -125,  -179.9, 0.0, rotation+30.0])
                
                print("Position reached. Closing gripper.")
                robot.schunk_gripper("close")
                print("Dice picked up.")
                time.sleep(.1)
                robot.write_cartesian_position([x, y, start_z,  -179.9, 0.0, rotation+30.0])
                robot.write_cartesian_position([Stack_Position[0]+Offset, Stack_Position[1]+100, -185+80*9,  -179.9, 0.0, rotation+30.0])
                
                # Move to stacking position with height adjustment for each dice
                target_z = -185 + O * dice_size
                print(f"Moving to stack position at ({Stack_Position[0]}, {Stack_Position[1]}, {target_z})")
                robot.write_cartesian_position([Stack_Position[0], Stack_Position[1], target_z, Stack_Position[3], Stack_Position[4], Stack_Position[5]])
                robot.schunk_gripper("open")
                print(f"Dice placed in stack.")
                time.sleep(.25)
                robot.write_cartesian_position([Stack_Position[0], Stack_Position[1], target_z+80, Stack_Position[3], Stack_Position[4], Stack_Position[5]])
                robot.write_cartesian_position([Stack_Position[0], Stack_Position[1]+100, -185+80*9,  -179.9, 0.0, rotation+30.0])
                O += 1
            except Exception as e:
                print(f"Error during stacking operation: {e}")
            time.sleep(0.25)
        else:
            print(f'Bunsen can grab Dice {i}')


# Callback for MQTT message reception
def on_message(client, userdata, message):
    global Standby, Complete, Ready
    payload = json.loads(message.payload.decode())
    if payload["status"] == "Standby":
        Standby = True
    elif payload["status"] == "Complete":
        Complete = True
    elif payload["status"] == "Ready":
        Ready = True

client.on_message = on_message
client.subscribe(TOPICB)

# Main execution function
def main():
    global Second, Ready, Complete, Standby
    client.loop_start()
    robot.schunk_gripper('open')
    robot.write_joint_pose(Home)

    # Wait until Ready signal is received

    while not Ready:
        time.sleep(0.5)
    print("Ready!")

    # Step 1: Run Image Capture and Coordinates Detection for Section 1
    run_image_capture()
    os.chdir("/home/funkey/ME-559/College/Lab 5")
    img = cv.imread('dice.png')
    mask = np.zeros(img.shape[:2], np.uint8)
    center = (1400, 80)  # Center of the image
    radius = 1800  # Set radius as a quarter of the image's smaller dimension
    cv.circle(mask, center, radius, 255, -1)
    masked_img = cv.bitwise_and(img, img, mask=mask)
    mask = np.zeros(masked_img.shape[:2], np.uint8)
    x1, y1 = 900, 0  # Top-left corner
    x2, y2 = 3000, 1640  # Bottom-right corner
    cv.rectangle(mask, (x1, y1), (x2, y2), 255, -1)

    # Apply the mask to the image using bitwise AND
    masked_image = cv.bitwise_and(masked_img, masked_img, mask=mask)

    # Save the masked image
    cv.imwrite('dice.png', masked_image)
    print("Mask Done")
    run_image_coords()
    print("Image Done!")
    publish_status("Ready")

    time.sleep(5)


    # Load pixel coordinates and transformation matrix
    pixel_data = load_pixel_coordinates("dice_positions.json")
    homography_matrix = load_transform_matrix("transform_matrix.json")
    print("Matrix Done!")

    
    section_1_robot_coords = []
    for die in pixel_data:
            robot_position = transform_pixel_to_robot(die["pixel_center"], homography_matrix)
            section_1_robot_coords.append({
                "robot_position": {"x": robot_position[0], "y": robot_position[1]},
                "rotation": die["rotation"]
            })


    print("Begin Collecting dice from Section 1!")
    stack_dice(robot, section_1_robot_coords)
    
    # Update status if section 1 completes without Standby
    if not Standby:
        print("First section complete without Standby. Publishing 'Standby'.")
        publish_status("Standby")
        Second = True
    
    robot.write_joint_pose(Home)

    # Wait until we get a 'Complete' or proceed directly if Standby is False
    while Standby and not Complete:
        time.sleep(1)
        Second = True

    if Standby and Complete:
        Second = True

    # Step 3: Process Section 2 if "complete" or continue with even dice
    if Second:
        print("Processing Section 2!")
        run_image_capture()
        os.chdir("/home/funkey/ME-559/College/Lab 5")
        img = cv.imread('dice.png')
        mask = np.zeros(img.shape[:2], np.uint8)
        x1, y1 = 900, 0  # Top-left corner
        x2, y2 = 3000, 2000  # Bottom-right corner
        cv.rectangle(mask, (x1, y1), (x2, y2), 255, -1)

        # Apply the mask to the image using bitwise AND
        masked_image = cv.bitwise_and(img, img, mask=mask)

        # Save the masked image
        cv.imwrite('dice.png', masked_image)
        print("Mask Done")
        run_image_coords()
        
        # Process Section 2 Dice
        pixel_data = load_pixel_coordinates("dice_positions.json")
        
        # Prepare robot coordinates for Section 2 dice stacking
        section_2_robot_coords = []
        print(pixel_data)
        for die in pixel_data:
            robot_position = transform_pixel_to_robot(die["pixel_center"], homography_matrix)
            section_2_robot_coords.append({
                "robot_position": {"x": robot_position[0], "y": robot_position[1]},
                "rotation": die["rotation"]
            })
        
        stack_dice(robot, section_2_robot_coords)
    
    # Final status update after both sections
    if not Standby and Second:
        print("Finalizing Section 2 with even dice")
        # Publish final "Complete" after both sections are processed
        robot.write_joint_pose(Home)
        publish_status("Complete")
        Complete = True
    else:
        # If already in Second or Standby, reset to ready for any further commands
        robot.write_joint_pose(Home)
        publish_status("Complete")

    # Stop MQTT loop
    client.loop_stop()

if __name__ == "__main__":
    main()
