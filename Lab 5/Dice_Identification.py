# Create a timer when operation starts and ends
import cv2 as cv
import numpy as np
import gxipy as gx
from PIL import Image
import os
from fpdf import FPDF
import json

'''
Capture Image
'''

os.chdir('/home/funkey/ME-559/College')
current_dir = os.getcwd()
print(current_dir)


def open_camera(device_manager):
    # Scan the network to find our device
    num_dev, dev_info_list = device_manager.update_all_device_list()

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
    print("Attempting to capture image...")
    raw_image = camera.data_stream[0].get_image(timeout=5000)
    if raw_image is None:
        print("Timeout: Failed to get image.")
        exit(1)
    print("Image captured successfully.")


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
os.chdir('/home/funkey/ME-559/College/Lab 5')
filename = 'dice.png'
capture_image(camera, filename)

# Open saved image with OpenCV
image = cv.imread(filename)
# cv.namedWindow('Captured Dice Image', cv.WINDOW_NORMAL)
# cv.imshow('Captured Dice Image', image)
cv.waitKey(0)

# End image acquisition and close device
camera.stream_off()
camera.close_device()

'''
Modify Image
'''

# Calibration points for robot and pixel coordinates (setup phase)
robot_coords = [
    [165.897, 400.657], [176.419, 694.628], [168.355, 1052.786],
    [486.306, 399.412], [473.538, 703.095], [473.25, 1047.543],
    [754.817, 397.818], [761.057, 724.025], [761.057, 935.786]
]
pixel_coords = [
    (94, 232), (787, 257), (1654, 232), 
    (96, 1002), (807, 965), (1631, 965), 
    (97, 1623), (859, 1633), (1383, 1647)
]

# Compute transformation matrix
pixel_coords_np = np.array(pixel_coords, dtype="float32")
robot_coords_np = np.array(robot_coords, dtype="float32")
transformation_matrix, _ = cv.findHomography(pixel_coords_np, robot_coords_np)

def transform_pixel_to_robot(pixel_position):
    px, py = pixel_position
    transformed_point = cv.perspectiveTransform(np.array([[[px, py]]], dtype="float32"), transformation_matrix)
    return transformed_point[0][0]

# Load and process the image
image = cv.imread('dice.png')

# Define the four points on the original image you want to warp from
# (top-left, top-right, bottom-right, bottom-left)
src_points = np.float32([
    [0, 1090],  # Top-left corner
    [3950, 1100],  # Top-right corner
    [3950, 3050],  # Bottom-right corner
    [50, 2850]   # Bottom-left corner
])

# Define the points in the output image you want to warp to
# For example, this could be a rectangle of width and height matching your intended output
width, height = 3800, 1750  # Adjust these values as needed
dst_points = np.float32([
    [0, 0],           # Top-left corner
    [width, 0],       # Top-right corner
    [width, height],  # Bottom-right corner
    [0, height]       # Bottom-left corner
])

# Calculate the perspective transformation matrix
matrix = cv.getPerspectiveTransform(src_points, dst_points)

# Perform the perspective warp
warped_image = cv.warpPerspective(image, matrix, (width, height))

cropped_image = image[1100:3000, 0:4000]
hsv = cv.cvtColor(warped_image, cv.COLOR_BGR2HSV)

# Define yellow color range
lower_yellow = np.array([28, 255, 150])
upper_yellow = np.array([40, 255, 255])
yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)

# Detect dice contours
contours, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
dice_data = []
cv.imwrite('Check.png', yellow_mask)

for contour in contours:
    area = cv.contourArea(contour)
    if 10000 < area < 50000:
        rect = cv.minAreaRect(contour)
        box = cv.boxPoints(rect)
        box = np.int32(box)
        cv.drawContours(warped_image, [box], 0, (0, 255, 0), 2)

        # Center and rotation of the dice
        center_x, center_y = np.mean(box, axis=0).astype(int)
        angle = rect[2]

        # Detect pips using Hough Circles
        roi_image = warped_image[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:, 0])]
        gray_roi = cv.cvtColor(roi_image, cv.COLOR_BGR2GRAY)
        blurred_roi = cv.GaussianBlur(gray_roi, (9, 9), 1)
        
        # Adjusted adaptive thresholding to improve pip detection
        thresh = cv.adaptiveThreshold(blurred_roi, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
        
        # Updated HoughCircles parameters for better pip detection
        circles = cv.HoughCircles(
            blurred_roi, 
            cv.HOUGH_GRADIENT, 
            dp=1.2, 
            minDist=25,           # Increased to reduce close false positives
            param1=50,            # Edge detection threshold
            param2=33,            # Accumulator threshold for circle detection
            minRadius=9,         # Adjusted for pip size
            maxRadius=25
        )

        # Count pips, filtered for circle properties
        num_pips = len(circles[0]) if circles is not None else 0
        if circles is not None:
            for (cx, cy, r) in np.round(circles[0, :]).astype("int"):
                if 10 <= r <= 25:  # Ensure radius is within expected range
                    cv.circle(warped_image, (min(box[:, 0]) + cx, min(box[:, 1]) + cy), r, (0, 255, 0), 2)

        # Convert dice center from pixels to robot coordinates
        robot_position = transform_pixel_to_robot((center_x, center_y))

        dice_data.append({
            "pixel_center": (center_x, center_y),
            "robot_position": robot_position,
            "pips": num_pips,
            "rotation": angle
        })
    if area > 50000:
        rect = cv.minAreaRect(contour)
        box = cv.boxPoints(rect)
        box = np.int32(box)
        cv.drawContours(warped_image, [box], 0, (0, 255, 0), 2)

        # Center and rotation of the dice
        center_x, center_y = np.mean(box, axis=0).astype(int)
        angle = rect[2]

        # Detect pips using Hough Circles
        roi_image = warped_image[min(box[:, 1]):max(box[:, 1]), min(box[:, 0]):max(box[:, 0])]
        gray_roi = cv.cvtColor(roi_image, cv.COLOR_BGR2GRAY)
        blurred_roi = cv.GaussianBlur(gray_roi, (9, 9), 1)
        
        # Adjusted adaptive thresholding to improve pip detection
        thresh = cv.adaptiveThreshold(blurred_roi, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 11, 2)
        
        cnt = contours[0]

        hull = cv.convexHull(cnt, returnPoints = False)
        # Assume `contour` and `hull` are defined contours and convex hulls
        defects = cv.convexityDefects(contour, hull)

        # Check if defects is None before proceeding
        if defects is not None:
            for i in range(defects.shape[0]):
                # Process each defect
                start_index, end_index, farthest_point_index, distance = defects[i, 0]
                start_point = tuple(contour[start_index][0])
                end_point = tuple(contour[end_index][0])
                farthest_point = tuple(contour[farthest_point_index][0])
            cv.line(cropped_image,start_point,end_point,[0,255,0],2)
            cv.circle(cropped_image,farthest_point,5,[0,0,255],-1)

# Sort dice data by number of pips
dice_data.sort(key=lambda x: x["pips"])
dice = 1

# Define the text offset
text_offset_x = -90  # Adjust as needed
text_offset_y = 40  # Adjust as needed

# Display results with offset text
for die in dice_data:
    print(f"Die with {die['pips']} pips at pixel position {die['pixel_center']}, "
          f"robot position {die['robot_position']} with rotation {die['rotation']} degrees.")
    cv.circle(warped_image, die["pixel_center"], 5, (0, 255, 0), -1)
    # Apply offset to text position
    text_position = (die["pixel_center"][0] + text_offset_x, die["pixel_center"][1] + text_offset_y)
    cv.putText(warped_image, f"Pips: {die['pips']}", text_position, cv.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 255), 2)
    cv.putText(warped_image, f"Pos: {dice}", (text_position[0], text_position[1] + 40), cv.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 2)
    dice += 1

# Define the file name to save the JSON output
output_file = "dice_positions.json"

# Prepare the data list for JSON format, ensuring int conversion
data_list = []
for i, die in enumerate(dice_data, start=1):
    data_entry = {
        "Dice": i,
        "pips": int(die['pips']),
        "pixel_center": {"x": int(die['pixel_center'][0]), "y": int(die['pixel_center'][1])},
        "robot_position": {"x": float(die['robot_position'][0]), "y": float(die['robot_position'][1])},
        "rotation": float(die['rotation'])
    }
    data_list.append(data_entry)

# Save the list to a JSON file
with open(output_file, "w") as f:
    json.dump(data_list, f, indent=4)

print(f"Dice data has been saved to {output_file}")

# Save the annotated image
cv.imwrite('Dice_Filter.png', warped_image)
cv.imwrite('Check.png', yellow_mask)
cv.imwrite('Thresholded_ROI.png', thresh)

cv.waitKey(0)
cv.destroyAllWindows()