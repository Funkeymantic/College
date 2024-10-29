import cv2 as cv
import numpy as np
import os
import gxipy as gx
from PIL import Image

'''
Capture image
'''

def open_camera(device_manager):
    # Scan the network to find our device
    num_dev, dev_info_list = device_manager.update_device_list()

    # Check if a device was found
    if num_dev == 0:
        print('No devices were found on the network.')
        print('Make sure you are on AirVandalRobot.')
        return None

    # The camera was found on the network
    # Extract the ip address
    camera_ip = dev_info_list[0].get('ip')

    # Open the device
    try:
        camera = device_manager.open_device_by_ip(camera_ip)
    except:
        print('Cannot open camera.')
        return None

    # Set a continuous acquisition
    camera.TriggerMode.set(gx.GxSwitchEntry.OFF)

    # Set exposure time & gain
    camera.ExposureTime.set(10000.0)
    camera.Gain.set(10.0)

    return camera

def capture_image(camera, filename):
    # Image improvement parameters
    if camera.GammaParam.is_readable():
        gamma_value = camera.GammaParam.get()
        gamma_lut = gx.Utility.get_gamma_lut(gamma_value)
    else:
        gamma_lut = None

    if camera.ContrastParam.is_readable():
        contrast_value = camera.ContrastParam.get()
        contrast_lut = gx.Utility.get_contrast_lut(contrast_value)
    else:
        contrast_lut = None
        
    if camera.ColorCorrectionParam.is_readable():
        color_correction = camera.ColorCorrectionParam.get()
    else:
        color_correction = 0

    # Get the current image
    raw_image = camera.data_stream[0].get_image(timeout=2000)
    if raw_image is None:
        print('Timeout: Failed to get image.')
        exit(1)

    # Convert image to RGB 
    rgb_image = raw_image.convert('RGB')
    if rgb_image is None:
        print('Failed to convert image to an RGB image.')
        exit(1)

    # Apply image improvements
    # rgb_image.image_improvement(color_correction, contrast_lut, gamma_lut)

    # Convert numpy array to save image
    numpy_image = rgb_image.get_numpy_array()
    if numpy_image is None:
        print('Failed to convert RGB image to numpy array.')
        exit(1)

    # Save image
    image = Image.fromarray(numpy_image, 'RGB')
    image.save(filename)

# Find compatible devices on the network
device_manager = gx.DeviceManager()
camera = open_camera(device_manager)

# Unable to connect to camera
if camera is None:
    exit(1)

# Start image acquisition
camera.stream_on()

# Capture and save image
filename = 'dice.png'
capture_image(camera, filename)

'''
Modify code start
'''

os.chdir('./Lab 5')

# Load the image
image = cv.imread('dice.png')

# Crop Original Image
cropped_image = image[1100:3000,75:3750]

# Convert to HSV
hsv = cv.cvtColor(cropped_image, cv.COLOR_BGR2HSV)

# Define yellow color range and mask it
lower_yellow = np.array([20, 100, 190])
upper_yellow = np.array([30, 255, 255])
yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)

# Find contours of the yellow regions (the dice)
contours, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)

# Filter based on the size of the contour (to avoid noise)
min_dice_area = 10000 # Adjust this based on the dice size in your image
max_dice_area = 1000000 # Adjusted maximum area for a dice

# Draw bounding boxes around the dice and detect dots inside each box
for contour in contours:
    area = cv.contourArea(contour)

    # Apply contour area filtering
    if min_dice_area < area < max_dice_area:
        # Use minAreaRect to get the rotated bounding box
        rect = cv.minAreaRect(contour)
        box = cv.boxPoints(rect) # Get box corners
        box = np.int32(box) # Convert to integer
        cv.drawContours(image, [box], 0, (0, 255, 0), 2) # Draw the green bounding box

        # Create a region of interest (ROI) to detect dots inside the dice
        roi_mask = yellow_mask[min(box[:, 1]):max(box[:, 1]), min(box[:,0]):max(box[:, 0])]
        roi_image = image[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:,0])]
        
        # Convert the ROI to grayscale
        gray_roi = cv.cvtColor(roi_image, cv.COLOR_BGR2GRAY)
        blurred_roi = cv.GaussianBlur(gray_roi, (5, 5), 2) # Adjusted blur to (5,5)
        
        # Try adaptive thresholding for better dot detection
        thresh = cv.adaptiveThreshold(blurred_roi, 255,cv.ADAPTIVE_THRESH_GAUSSIAN_C,cv.THRESH_BINARY_INV, 11, 3)
        
        # Detect circles (dots) in the gray ROI with adjusted parameters
        circles = cv.HoughCircles(blurred_roi, cv.HOUGH_GRADIENT, dp=1.2,minDist=15,param1=50, param2=20, minRadius=39,maxRadius=50) # Adjusted HoughCircles params
        
        # Initialize pip count
        pip_count = 0

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            pip_count = len(circles) # Count the number of detected circles (pips)
            
            for (cx, cy, r) in circles:
                # Draw the blue circle in the original image, adjusting for ROI offset
                cv.circle(image, (min(box[:, 0]) + cx, min(box[:, 1]) + cy), r,(255, 0, 0), 2)
        # Display the pip count for each die
        print(f"Pip count for dice: {pip_count}")
# Save the image with rotated bounding boxes and dots
cv.imwrite('Dice_Filter.png', cropped_image)

# Display the result (optional)
# cv.namedWindow('Dice Detection', cv.WINDOW_NORMAL) # Create a resizable window
# cv.resizeWindow('Dice Detection', 1512, 2016) # Set the window size (width,height)
# cv.imshow('Dice Detection', image) # Display the image in the resized window
cv.waitKey(0)
cv.destroyAllWindows()