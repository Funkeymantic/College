import cv2 as cv
import numpy as np
import os

os.chdir('/home/funkey/ME-559/College/Lab 5')

# Load the image
image = cv.imread('dice.png')

# Convert to HSV
hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

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
cv.imwrite('Dice_Filter.png', image)

# Display the result (optional)
cv.namedWindow('Dice Detection', cv.WINDOW_NORMAL) # Create a resizable window
cv.resizeWindow('Dice Detection', 1512, 2016) # Set the window size (width,height)
cv.imshow('Dice Detection', image) # Display the image in the resized window
cv.waitKey(0)
cv.destroyAllWindows()