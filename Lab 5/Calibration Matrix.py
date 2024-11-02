"""
Enhanced program for handling two robots with distinct grippers and opposite y-direction
movements, capturing image data, identifying dice, and calibrating with homography.
"""

import cv2
import numpy as np
import sys
sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

# Set robot IPs
robot_beaker_ip = '172.29.208.124'
robot_bunsen_ip = '172.29.208.123'

# Define z-heights and dice positions for each robot
z1, z11 = -31.0, -128.0  # Beaker heights
# z2, z21 = -38.0, -146.0  # Bunsen heights
dice_positions_beaker = [
    ([283.75, 555.33, z1], [283.75, 555.33, z11]),
    ([467.5, 555.33, z1], [467.5, 555.33, z11]),
    ([651.25, 555.33, z1], [651.25, 555.33, z11]),
    ([283.75, 774.17, z1], [283.75, 774.17, z11]),
    ([467.5, 774.17, z1], [467.5, 774.17, z11]),
    ([651.25, 774.17, z1], [651.25, 774.17, z11])
]
# dice_positions_bunsen = [
#     ([283.75, -770.83, z2], [283.75, -770.83, z21]),
#     ([467.5, -770.83, z2], [467.5, -770.83, z21]),
#     ([651.25, -770.83, z2], [651.25, -770.83, z21]),
#     ([283.75, -553.66, z2], [283.75, -553.66, z21]),
#     ([467.5, -553.66, z2], [467.5, -553.66, z21]),
#     ([651.25, -553.66, z2], [651.25, -553.66, z21])
# ]

# Adjust coordinates based on robot-specific movements
def transform_coordinates(coords, robot_ip):
    # Adjust y-direction based on the robot
    if robot_ip == robot_bunsen_ip:
        coords[1] = -coords[1]  # Reverse y for Bunsen
    return coords

# Function to get robot coordinates with transformation
def get_robot_coords(robot_ip):
    robot = Robot(robot_ip)
    coords = robot.read_current_cartesian_pose()[:2]
    return transform_coordinates(coords, robot_ip)

# Process image and capture dice coordinates
def get_image_coords(image_path):
    img = cv2.imread(image_path)
    image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    image_hsv_blur = cv2.GaussianBlur(image_hsv, (5, 5), 1)
    lower_yellow = np.array([28, 255, 150])
    upper_yellow = np.array([50, 255, 255])
    mask = cv2.inRange(image_hsv_blur, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # List to hold image coordinates of each detected dice
    dice_image_coords = []
    cv2.namedWindow('Current Dice', cv2.WINDOW_NORMAL)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:
            continue  # Filter out small artifacts

        moment = cv2.moments(cnt)
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])
        dice_image_coords.append([cx, cy])

        # Display image with marked dice
        output = img.copy()
        cv2.drawContours(output, [cnt], -1, (0, 255, 0), 3)
        cv2.imshow("Current Dice", output)
        cv2.waitKey(500)

    cv2.destroyAllWindows()
    return dice_image_coords

# Function to move dice for each robot and record positions
def move_and_record_dice(robot_ip, dice_positions, gripper_type):
    robot = Robot(robot_ip)
    robot_coords = []

    # Define the home position based on the robot
    home_pose = [0, 0, 0, 0, -90, 30] if robot_ip == robot_beaker_ip else [0, 0, 0, 0, -90, -150]
    robot.write_joint_pose(home_pose)

    for dice, dice_high in dice_positions:
        # Open gripper based on type
        if gripper_type == 'schunk':
            robot.schunk_gripper('open')
        # elif gripper_type == 'onRobot':
        #     robot.onRobot_gripper_open(100, 60)
    
        
        robot.write_cartesian_position(dice)
        
        # Close gripper based on type
        if gripper_type == 'schunk':
            robot.schunk_gripper('close')
        # elif gripper_type == 'onRobot':
        #     robot.onRobot_gripper_close(77, 60)
        
        robot.write_cartesian_position(dice_high)

        # Open gripper to release dice
        if gripper_type == 'schunk':
            robot.schunk_gripper('open')
        # elif gripper_type == 'onRobot':
        #     robot.onRobot_gripper_open(100, 60)
        
        # Capture and record transformed robot coordinates
        current_coords = get_robot_coords(robot_ip)
        robot_coords.append(current_coords)
        print(f"Recorded robot coordinates for dice: {current_coords}")

    robot.write_joint_pose(home_pose)
    return robot_coords

# Main function to execute dice placement, capture image, and calculate homography
def main():
    # Move dice and record positions for each robot
    dice_beaker_coords = move_and_record_dice(robot_beaker_ip, dice_positions_beaker, gripper_type='schunk')
    # dice_bunsen_coords = move_and_record_dice(robot_bunsen_ip, dice_positions_bunsen, gripper_type='onRobot')
    
    # Get image coordinates
    image_path = "dice.png"  # Replace with actual image path
    dice_image_coords = get_image_coords(image_path)

    # Prompt user to match dice in image to robot coordinates
    dice_robot_coords = []
    for i, image_coord in enumerate(dice_image_coords, start=1):
        print(f"Image coordinates for Dice {i}: {image_coord}")
        index = int(input(f"Enter the robot coordinates array index for Dice {i}: "))
        
        # Match image dice to robot dice coordinates with correct offset
        dice_robot_coords.append(dice_beaker_coords[index] if index < len(dice_beaker_coords))

    # Calculate homography matrix
    src_points = np.array(dice_image_coords)
    dst_points = np.array(dice_robot_coords)
    transform_matrix, _ = cv2.findHomography(src_points, dst_points, cv2.RANSAC, confidence=0.9)
    print("Calculated Transformation Matrix:")
    print(transform_matrix)

    # Save transformation matrix to file
    with open("transform_mat.txt", mode="w") as file:
        file.write(str(transform_matrix))

if __name__ == '__main__':
    main()
