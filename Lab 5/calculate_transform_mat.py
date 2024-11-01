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
robot_ip = '172.29.208.123' # Bunson

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


def main():
    cv2.namedWindow('Current Dice', cv2.WINDOW_NORMAL)
    get_image_cords(
        image_path = "dice.png",
        robot_ip = robot_ip,
        output_file= "transform_mat.txt"
    )


if __name__=='__main__':
    main()