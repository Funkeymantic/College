import cv2 as cv
import numpy as np
import json
import os

# Define the base directory and paths
BASE_DIR = "/home/funkey/ME-559/College/Lab 5"  # Adjust if needed
image_path = os.path.join(BASE_DIR, 'dice.png')
output_json_path = "/home/funkey/ME-559/College/Lab 5/dice_positions.json"


# Load and process the image
image = cv.imread(image_path)

'''
Define the four points on the original image you want to warp from
(top-left, top-right, bottom-right, bottom-left)
'''

src_points = np.float32([
    [1075, 0],       # Top-left corner
    [2830, 40],    # Top-right coner
    [3000, 3835],    # Bottom-right corner
    [1100, 3920]       # Bottom-left corner
])

'''
Define the points in the output image you to warp to
'''
width, height = 800*2, 1670*2 # Adjust these values as needed
dst_points = np.float32([
    [0,0],           # Top-left corner
    [width, 0],      # Top-right corner
    [width, height], # Bottom-right corner
    [0, height]      # Bottom-left corner
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
detected_dice_centers = []
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
            param1=60,            # Edge detection threshold
            param2=30,            # Accumulator threshold for circle detection
            minRadius=9,          # Adjusted for pip size
            maxRadius=25
        )

        # Count pips, filtered for circle properties
        num_pips = len(circles[0]) if circles is not None else 0
        if circles is not None:
            for (cx, cy, r) in np.round(circles[0, :]).astype("int"):
                if 10 <= r <= 25:  # Ensure radius is within expected range
                    cv.circle(warped_image, (min(box[:, 0]) + cx, min(box[:, 1]) + cy), r, (0, 255, 0), 2)

        # Convert dice center from pixels to robot coordinates
        # robot_position = transform_pixel_to_robot((center_x, center_y))

        dice_data.append({
            "pixel_center": (center_x, center_y),
            # "robot_position": robot_position,
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
          f"rotation {die['rotation']} degrees.")
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
        "pixel_center": {"x": int(die['pixel_center'][1]), "y": int(die['pixel_center'][0])},
        # "robot_position": {"x": float(die['robot_position'][0]), "y": float(die['robot_position'][1])},
        "rotation": float(die['rotation'])
    }
    data_list.append(data_entry)

# Save the list to a JSON file
with open(output_file, "w") as f:
    json.dump(data_list, f, indent=4)

print(f"Dice data has been saved to {output_file}")

# Save dice data to JSON file
with open(output_json_path, "w") as f:
    json.dump(data_list, f, indent=4)

print(f"Dice data has been saved to {output_json_path}")

# Save images in the BASE_DIR
cv.imwrite(os.path.join(BASE_DIR, 'Dice_Filter.png'), warped_image)
cv.imwrite(os.path.join(BASE_DIR, 'Check.png'), yellow_mask)
cv.imwrite(os.path.join(BASE_DIR, 'Thresholded_ROI.png'), thresh)


cv.waitKey(0)
cv.destroyAllWindows()