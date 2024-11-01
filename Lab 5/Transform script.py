"""
Calculates the Transformation Matrix from an image to a robots
cordinate system.
"""

import cv2
import numpy as np
import sys
sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

robot_ip = '172.29.208.124' # Beaker
# robot_ip = '172.29.208.123' # Bunson

def get_robot_cords(robot_ip) -> list:
    robot = Robot(robot_ip)
    cords = robot.read_current_cartesian_pose()
    xy_cords = cords[:2]
    return xy_cords

def get_image_cords(image_path, robot_ip, output_file) -> np.ndarray:
    # Load in image
    img = cv2.imread(image_path)
    # Convert to HSV for color mapping
    image_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    # Lightly blur the image
    image_hsv_blur = cv2.GaussianBlur(
        image_hsv,
        (5,5),
        1
    )
    # Mask out anything not in a yellow-ish color range
    lower_yellow = np.array([ 28, 255, 150])
    upper_yellow = np.array([ 50, 255, 255])
    mask = cv2.inRange(image_hsv_blur, lower_yellow, upper_yellow)
    # Get the contours in the image
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )
    dice_image_cords: list = []
    dice_robot_cords: list = []
    # Gather Dice cordinates and generate mapping
    for cnt in contours:
        # Calculate the area
        area = cv2.contourArea(cnt)
        if area < 5000:
            # If area is to small filter it out as its just an
            # image artifact
            continue
        # Create img copy to draw on
        output = img.copy()
        # Calculate the center of the bounding box
        moment = cv2.moments(cnt)
        cx = int((moment['m10']/moment["m00"]))
        cy = int((moment['m01']/moment["m00"]))
        current_image_cord = [cx,cy]
        # Create the rectangle
        rect = cv2.minAreaRect(cnt)
        box = np.intp(cv2.boxPoints(rect))
        # Draw the rectangle
        cv2.drawContours(output, [box], -1, (0, 255, 0), 3)
        # Display the output image
        cv2.imshow("Current Dice", output)
        print(f"Dice Cordinates: {current_image_cord}")
        print("Please Move Robot to current Selected Dice")
        input("Hit Enter when robot is in the correct position")
        # Read the robots Cordinates
        current_robot_cords = get_robot_cords(robot_ip)
        print(f"Robot Cordinates: {current_robot_cords}")
        # Add the cordinates
        dice_image_cords.append(current_image_cord)
        dice_robot_cords.append(current_robot_cords)

    # Calculate the transformation matrix
    src_cords = np.array(dice_image_cords)
    dst_cords = np.array(dice_robot_cords)
    tranform_matrix = cv2.findHomography(
        srcPoints=src_cords,
        dstPoints=dst_cords,
        method=cv2.RANSAC,
        confidence=0.9
    )
    print("Calculated Transformation Matrix:")
    print(tranform_matrix)
    with open(output_file, mode="w") as fd:
        fd.write(str(tranform_matrix))
    return tranform_matrix
# Beaker = 1, Bunsen = 2
Home1 = [0, 0, 0, 0, -90, 30]
Die1 = [468, -5, -185, -179.9, 0.0, 30.0]
Home2 = [0, 0, 0, 0, -90, -150]
Die2 = [462, 0.0, -205, 179.9, 0.0, 0.0]
z1 = -31.0
z11 = -128.0
z2 = -38.0
z21 = -146.0 

dice_beaker_cords: list = []
dice_bunsen_cords: list = []

def main():
    robot = Robot('172.29.208.124')
    
    # List of dice positions with two z-heights
    dice_positions = [
        ([283.75, 555.33, z1], [283.75, 555.33, z11]),
        ([467.5, 555.33, z1], [467.5, 555.33, z11]),
        ([651.25, 555.33, z1], [651.25, 555.33, z11]),
        ([283.75, 774.17, z1], [283.75, 774.17, z11]),
        ([467.5, 774.17, z1], [467.5, 774.17, z11]),
        ([651.25, 774.17, z1], [651.25, 774.17, z11])
    ]

    robot.write_joint_pose(Home1)
    for i, (dice, dice_high) in enumerate(dice_positions, start=1):
        robot.schunk_gripper('open')
        robot.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])
        robot.write_cartesian_position(Die1)
        robot.schunk_gripper('close')
        robot.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])
        robot.write_cartesian_position(dice)
        robot.write_cartesian_position(dice_high)
        robot.schunk_gripper('open')
        current_robot_coordinates = get_robot_cords(robot_ip)
        print(f"Robot coordinates: {current_robot_coordinates}")
        dice_beaker_cords.apend(current_robot_coordinates)
        robot.write_cartesian_position(dice)
        print(f"Dice {i} is done!")
    robot.write_joint_pose(Home1)

    robot = Robot('172.29.208.123')
    
        # List of dice positions with two z-heights
    dice_positions = [
        ([283.75, -770.83, z2], [283.75, -770.83, z21]),
        ([467.5, -770.83, z2], [467.5, -770.83, z21]),
        ([651.25, -770.83, z2], [651.25, -770.83, z21]),
        ([283.75, -553.66, z2], [283.75, -553.66, z11]),
        ([467.5, -553.66, z2], [467.5, -553.66, z21]),
        ([651.25, -553.66, z2], [651.25, -553.66, z21])
    ]

    robot.write_joint_pose(Home2)
    for i, (dice, dice_high) in enumerate(dice_positions, start=1):
        robot.onRobot_gripper_open(100, 60)
        robot.write_cartesian_position([468, -5, z2, -179.9, 0.0, 0.0])
        robot.write_cartesian_position(Die2)
        robot.onRobot_gripper_close(77, 60)
        robot.write_cartesian_position([468, -5, z2, -179.9, 0.0, 0.0])
        robot.write_cartesian_position(dice)
        robot.write_cartesian_position(dice_high)
        robot.onRobot_gripper_open(100, 60)
        current_robot_coordinates = get_robot_cords(robot_ip)
        print(f"Robot coordinates: {current_robot_coordinates}")
        dice_bunsen_cords.apend(current_robot_coordinates)
        robot.write_cartesian_position(dice)
        print(f"Dice {i} is done!")
    robot.write_joint_pose(Home2)

    #trying to understanc this
    cv2.namedWindow('Current Dice', cv2.WINDOW_NORMAL)
    get_image_cords(
        image_path = "dice.png",
        robot_ip = robot_ip,
        output_file= "transform_mat.txt"
    )


if __name__=='__main__':
    main()

    