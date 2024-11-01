'''
Calculates the Transformation Matrix of the image to the robots coordinates
'''

import cv2 as cv
import numpy as np
import sys

sys.path.append('.\Fanuc_ethernet_ip_drivers\src')
from robot_controller import robot as Robot

robot_ip = '172.29.208.124' # Beaker
robot_ip = '172.29.208.123' # Bunson

def get_robot_cords(robot_ip) -> list:
    robot = Robot(robot_ip)
    cords = robot.read_current_cartesian_pose()
    xy_cords = cords[:2]
    return xy_cords

