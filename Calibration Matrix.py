"""
Program to control robots to position dice, capture images,
identify dice, link each dice to robot coordinates, and calibrate
the camera using homography.
"""

import cv2
import numpy as np
import sys
sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

# Set robot IPs
robot_beaker_ip = '172.29.208.124'
robot_bunsen_ip = '172.29.208.123'

# Define the z-heights and dice positions for Beaker and Bunsen
z1, z11 = -31.0, -128.0
z2, z21 = -38.0, -146.0
dice_positions_beaker = [
    ([283.75, 555.33, z1], [283.75, 555.33, z11]),
    ([467.5, 555.33, z1], [467.5, 555.33, z11]),
    ([651.25, 555.33, z1], [651.25, 555.33, z11]),
    ([283.75, 774.17, z1], [283.75, 774.17, z11]),
    ([467.5, 774.17, z1], [467.5, 774.17, z11]),
    ([651.25, 774.17, z1], [651.25, 774.17, z11])
]
dice_positions_bunsen = [
    ([283.75, -770.83, z2], [283.75, -770.83, z21]),
    ([467.5, -770.83, z2], [467.5, -770.83, z21]),
    ([651.25, -770.83, z2], [651.25, -770.83, z21]),
    ([283.75, -553.66, z2], [283.75, -553.66, z21]),
    ([467.5, -553.66, z2], [467.5, -553.66, z21]),
    ([651.25, -553.66, z2], [651.25, -553.66, z21])
]

# Capture robot coordinates function
def get_robot_cords(robot_ip):
    robot = Robot(robot_ip)
    cords = robot.read_current_cartesian_pose()
    return cords[:2]  # Only x, y for homography

# Function to capture and process image for dice locations
def get_image_cords(image_path):
    img = cv2.imread(image_path)
    image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    image_hsv_blur = cv2.GaussianBlur(image_hsv, (5, 5), 1)
    lower_yellow = np.array([28, 255, 150])
    upper_yellow = np.array([50, 255, 255])
    mask = cv2.inRange(image_hsv_blur, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # List to hold image coordinates of each detected dice
    dice_image_cords = []
    cv2.namedWindow('Current Dice', cv2.WINDOW_NORMAL)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 5000:
            continue  # Filter out small artifacts

        moment = cv2.moments(cnt)
        cx = int(moment['m10'] / moment['m00'])
        cy = int(moment['m01'] / moment['m00'])
        dice_image_cords.append([cx, cy])

        # Display image with marked dice
        output = img.copy()
        cv2.drawContours(output, [cnt], -1, (0, 255, 0), 3)
        cv2.imshow("Current Dice", output)
        cv2.waitKey(500)  # Display for a short moment

    cv2.destroyAllWindows()
    return dice_image_cords

# Robot positioning for Beaker and Bunsen
def move_and_record_dice(robot_ip, dice_positions):
    robot = Robot(robot_ip)
    robot_coords = []

    home_pose = [0, 0, 0, 0, -90, 30] if robot_ip == robot_beaker_ip else [0, 0, 0, 0, -90, -150]
    robot.write_joint_pose(home_pose)

    for dice, dice_high in dice_positions:
        robot.schunk_gripper('open')
        robot.write_cartesian_position(dice)
        robot.schunk_gripper('close')
        robot.write_cartesian_position(dice_high)
        robot.schunk_gripper('open')
        
        # Capture and record robot coordinates
        current_coords = get_robot_cords(robot_ip)
        robot_coords.append(current_coords)
        print(f"Recorded robot coordinates for dice: {current_coords}")

    robot.write_joint_pose(home_pose)
    return robot_coords

# Main function to move dice, capture image, and compute homography
def main():
    # Move dice and record positions
    dice_beaker_cords = move_and_record_dice(robot_beaker_ip, dice_positions_beaker)
    dice_bunsen_cords = move_and_record_dice(robot_bunsen_ip, dice_positions_bunsen)
    
    # Get image coordinates by capturing and processing the image
    image_path = "dice.png"  # Update this to the actual image path
    dice_image_cords = get_image_cords(image_path)

    # Prompt user for matching dice to robot coordinates
    dice_robot_cords = []
    for i, image_cord in enumerate(dice_image_cords, start=1):
        print(f"Image coordinates for Dice {i}: {image_cord}")
        index = int(input(f"Enter the robot coordinates array index for Dice {i}: "))
        
        # Append matched robot coordinate
        dice_robot_cords.append(dice_beaker_cords[index] if index < len(dice_beaker_cords) else dice_bunsen_cords[index - len(dice_beaker_cords)])

    # Calculate homography matrix
    src_points = np.array(dice_image_cords)
    dst_points = np.array(dice_robot_cords)
    transform_matrix, _ = cv2.findHomography(src_points, dst_points, cv2.RANSAC, confidence=0.9)
    print("Calculated Transformation Matrix:")
    print(transform_matrix)

    # Save transformation matrix to file
    with open("transform_mat.txt", mode="w") as file:
        file.write(str(transform_matrix))

if __name__ == '__main__':
    main()
