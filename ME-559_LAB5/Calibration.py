import sys
import time
import json
import numpy as np
import cv2 as cv
import subprocess
import os

# Ensure access to the robot controller path
sys.path.append('../fanuc_ethernet_ip_drivers/src')
from robot_controller import robot as Robot

# Set up the robot connection and parameters
robot_ip = '172.29.208.124'  # Beaker
robot = Robot(robot_ip)
Home = [0, 0, 0, 0, -90, 30]
Die = [468, -5, -185, -179.9, 0.0, 30.0]
z1 = -25.0
z11 = -120.0

# Predefined robot coordinates to align with pixel coordinates
dice_beaker_cords = []

# Define dice positions in robot coordinates
dice_positions = [
    ([651.25, 905.5, z1], [651.25, 905.5, z11]), # 1
    ([467.5, 905.5, z1], [467.5, 905.5, z11]), # 2
    ([283.75, 905.5, z1], [283.75, 905.5, z11]), # 3
    ([651.25, 686.67, z1], [651.25, 686.67, z11]), # 4
    ([467.5, 686.67, z1], [467.5, 686.67, z11]), # 5
    ([283.75, 686.67, z1], [283.75, 686.67, z11]), # 6
    ([651.25, 467.83, z1], [651.25, 467.83, z11]), # 7
    ([467.5, 467.83, z1], [467.5, 467.83, z11]), # 8
    ([283.75, 467.83, z1], [283.75, 467.83, z11]) # 9
]

# Gather robot coordinates by moving to each dice position
def gather_robot_coordinates():
    for i, (dice, dice_high) in enumerate(dice_positions, start=1):
        # robot.write_cartesian_position(dice)
        # time.sleep(0.5)
        current_robot_coordinates = robot.read_current_cartesian_pose()[:2]
        dice_beaker_cords.append(current_robot_coordinates)
        print(f"Recorded robot coordinates for Dice {i}: {current_robot_coordinates}")
    # robot.write_joint_pose(Home)

# Run the Image Capture code
def run_image_capture():
    subprocess.run([sys.executable, "/home/funkey/ME-559/College/Lab 5/Image_Capture.py"])
    

# Run the Image Coordinates detection code
def run_image_coords():
    subprocess.run([sys.executable, "/home/funkey/ME-559/College/Lab 5/Image_Coords.py"])

# Load pixel coordinates from the JSON file created by Image_Coords.py
def load_pixel_coordinates(json_file="./Lab 5/dice_positions.json"):
    with open(json_file, "r") as f:
        pixel_data = json.load(f)
        calibration_points = []
        for Dice in pixel_data:
            x = Dice['pixel_center']['x']
            y = Dice['pixel_center']['y']
            calibration_points.append([x, y, 1])
    return calibration_points

# Calibrate using homography to relate pixel and robot coordinates
def calibrate_transform(pixel_coords, robot_coords, output_file="transform_matrix.json"):
    pixel_coords_np = np.array(pixel_coords, dtype="float32")
    robot_coords_np = np.array(robot_coords, dtype="float32")
    homography_matrix, _ = cv.findHomography(pixel_coords_np, robot_coords_np, cv.RANSAC)

    os.chdir('./Lab 5')
    # Save the homography matrix to a JSON file
    with open(output_file, "w") as f:
        json.dump(homography_matrix.tolist(), f, indent=4)
    print(f"Transformation matrix saved to {output_file}")
    return homography_matrix

# Function to transform pixel coordinates to robot coordinates using the homography matrix
def transform_pixel_to_robot(pixel_coord, homography_matrix):
    pixel_np = np.array([[pixel_coord]], dtype="float32")
    transformed_coord = cv.perspectiveTransform(pixel_np, homography_matrix)
    return transformed_coord[0][0]



def main():
    robot.schunk_gripper('open')
    robot.write_cartesian_position([700, -5, -185+81*8, -179.9, 0.0, 30.0+90])
    input("Once the dice is inposition remove top dice.")
    robot.write_cartesian_position([700, -5, -185+81*8, -179.9, 0.0, 30.0])
    input("Is dice in position x and y?")
    # robot.write_joint_pose(Home)
    for i, (dice, dice_high) in enumerate(dice_positions, start=1):
        robot.schunk_gripper('open')
        robot.write_cartesian_position([700, -5, -185+80*(10-i), -179.9, 0.0, 30.0])
        robot.write_cartesian_position([700, -5, -185+80*(10-i), -179.9, 0.0, 30.0+90])
        robot.write_cartesian_position([700, -5, -185+80*(9-i), -179.9, 0.0, 30.0+90])
        robot.schunk_gripper('close')
        time.sleep(0.25)
        robot.write_cartesian_position([700, -5, -185+80*(10-i), -179.9, 0.0, 30.0])
        robot.write_cartesian_position([468, -5, -185+80*(10-i), -179.9, 0.0, 30.0])
        robot.write_cartesian_position([468, -5, -185, -179.9, 0.0, 30.0])
        robot.schunk_gripper('open')
        time.sleep(0.25)
        # robot.write_cartesian_position([468+40, -5, -185, -179.9, 0.0, 30.0])
        # robot.schunk_gripper('open')
        # time.sleep(0.25)
        robot.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])

        robot.write_cartesian_position([468+40, -5, -185, -179.9, 0.0, 30.0])
        robot.schunk_gripper('close')
        time.sleep(0.25)
        robot.schunk_gripper('open')
        time.sleep(0.25)
        robot.write_cartesian_position([468-40, -5, -185, -179.9, 0.0, 30.0])
        robot.schunk_gripper('close')
        time.sleep(0.25)
        robot.schunk_gripper('open')
        time.sleep(0.25)
        robot.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0+90.0])

        robot.write_cartesian_position([468, -5-40, -185, -179.9, 0.0, 30.0+90])
        robot.schunk_gripper('close')
        time.sleep(0.25)
        robot.schunk_gripper('open')
        time.sleep(0.25)
        robot.write_cartesian_position([468, -5+40, -185, -179.9, 0.0, 30.0+90])
        robot.schunk_gripper('close')
        time.sleep(0.25)
        robot.schunk_gripper('open')
        time.sleep(0.25)
        robot.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])
        
        robot.write_cartesian_position(Die)
        robot.schunk_gripper('close')
        time.sleep(.25)
        print("ahhhh")

        robot.write_cartesian_position([468, -5, z1, -179.9, 0.0, 30.0])
        robot.write_cartesian_position(dice)
        print("oooo")
        robot.write_cartesian_position(dice_high)
        robot.schunk_gripper('open')
        time.sleep(.25)
        robot.write_cartesian_position([dice_high[0], dice_high[1], dice_high[2]+100])
        # Gather robot coordinates
        current_robot_coordinates = [dice_high[0], dice_high[1], 1]
        dice_beaker_cords.append(current_robot_coordinates)
        print(f"Recorded robot coordinates for Dice {i}: {current_robot_coordinates}")
        # gather_robot_coordinates()
        # print(f"Robot coordinates: {gather_robot_coordinates}")
        # dice_beaker_cords.append(gather_robot_coordinates)
        # robot.write_cartesian_position(dice)
        print(f"Dice {i} is done!")
    robot.write_joint_pose(Home)

    # Load pixel coordinates from JSON file
    sys.path.append("/home/funkey/ME-559/College/myenv/lib/python3.12/site-packages")
    run_image_capture()
    run_image_coords()
    pixel_coords = load_pixel_coordinates()
    print(f"Image: {pixel_coords}")
    print(f"Beaker: {dice_beaker_cords}")

    New_Beaker_Coords = {}

    for i, coord1 in enumerate(pixel_coords):
        print(f'\nCoordinate {i + 1} in Pixel Array: {coord1}')
        print("Available coordinates in Beaker Array:")

        # Print all coordinates in the Pixel Array with an index
        for j, coord3 in enumerate(pixel_coords):
            print(f" {j + 1}: {coord3}")

        # Print all coordinates in the Beaker Array with an index
        for j, coord2 in enumerate(dice_beaker_cords):
            print(f" {j + 1}: {coord2}")

        # Ask the user for input
        choice = int(input(f"Select the index of the matching coordinates in the Pixel Array for {coord1}: ")) - 1

        # Store the match in the dictionary using a tuple as the key
        if 0 <= choice < len(dice_beaker_cords):
            New_Beaker_Coords[tuple(coord1)] = tuple(dice_beaker_cords[choice])
            print(f"Matched {coord1} with {dice_beaker_cords[choice]}")
        else:
            print("Invalid choice. Please try again.")

    # Print final Beaker Coords
    print("\nFinal matches:")
    for key, value in New_Beaker_Coords.items():
        print(f"{key} -> {value}")



    # Perform homography calibration
    homography_matrix = calibrate_transform(pixel_coords, New_Beaker_Coords)
    print(homography_matrix)

    # # Example usage of the transformation matrix
    # example_pixel_coord = pixel_coords[0]  # Example from pixel data
    # robot_coord = transform_pixel_to_robot(example_pixel_coord, homography_matrix)
    # print(f"Example pixel coordinate {example_pixel_coord} corresponds to robot coordinate {robot_coord}")

if __name__ == "__main__":
    main()