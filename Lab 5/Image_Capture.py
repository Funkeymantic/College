import cv2 as cv
import numpy as np
import gxipy as gx
import os
from PIL import Image


'''
Capture Image
'''

os.chdir('/home/funkey/ME-559/College')
current_dir = os.getcwd()
print(f"Current Directory is: {current_dir}")

def open_camera(device_manager):
    # Scan thenetwork for camera device
    num_dev, dev_info_list = device_manager.update_all_device_list()

    # Check if a device was found
    if num_dev == 0:
        print('No devices were found on the network.')
        print('Make sure you are on VandalRobot.')
        return None
    # The Camera was found on the network
    #Extract the ip address
    camer_ip = dev_info_list[0].get('ip')

    #open the device
    try:
        camera = device_manager.open_device_by_ip(camer_ip)
    except:
        print('Cannot open Camera.')
        return None
    
    #set a continuouus acquisition
    camera.TriggerMode.set(gx.GxSwitchEntry.OFF)

    #set exposure time & gain
    camera.ExposureTime.set(10000.0)
    camera.Gain.set(10.0)

    return camera

def capture_image(camera, filename):
    # Get the current Image
    print("Attempting to capture image...")
    raw_image = camera.data_stream[0].get_image(timeout=10000)
    if raw_image is None:
        print("Timeout: Failed to get image.")
        exit(1)
    print("Image captured successsfully")

    # convert image to RGB
    rgb_image = raw_image.convert('RGB')
    if rgb_image is None:
        print('Failed to convert image to an RGB image.')
        exit(1)
    
    # Convert numpy array to save image
    numpy_image = rgb_image.get_numpy_array()
    if numpy_image is None:
        print('Failed to convert RGB image to numpy array.')
        exit(1)
    
    # Rotate Image
    Rotated = cv.rotate(numpy_image, cv.ROTATE_90_CLOCKWISE)
    Flipped = cv.flip(Rotated, 1)

    #save Image
    image = Image.fromarray(Flipped, 'RGB')
    image.save(filename)

# Find compoatible device on the network
device_manager = gx.DeviceManager()
camera = open_camera(device_manager)

# Unable to connect to camera
if camera is None:
    exit(1)

#Start image acququisition
camera.stream_on()

# Capture and save image
os.chdir('./Lab 5')
filename = 'dice.png'
capture_image(camera, filename)

#open saved image with OpenCV
image = cv.imread(filename)
cv.waitKey(0)

# End image acquisition
camera.stream_off()
camera.close_device()

cv.destroyAllWindows()