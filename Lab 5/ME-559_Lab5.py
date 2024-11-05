from fpdf import FPDF
import timer
import subprocess
import cv2 as cv
import json

import subprocess
import json
from robot_controller import Robot

# Execute Dice_Identification.py
subprocess.run(["python", "Dice_Identification.py"], check=True)

# Load JSON data from dice_positions.json
with open("dice_positions.json", "r") as f:
    dice_data = json.load(f)

# Connect to Beaker Robot
beaker_ip = "172.29.208.124"  # Replace with Beaker's actual IP address
beaker = Robot(beaker_ip)
beaker.write_joint_pose([0, 0, 0, 0, -90, 30])  # Move Beaker to Home1 position

# Define stacking position parameters
stack_x, stack_y, stack_z = 468, -5, -185  # Base position for stacking
dice_size = 80  # Dice are 8 cm (80 mm) each
stack_height_increment = dice_size  # Increase in z-axis per dice

# Stack each dice based on JSON coordinates
for i, die in enumerate(dice_data, start=1):
    # Get robot coordinates from JSON data
    robot_x, robot_y = die["robot_position"]["x"], die["robot_position"]["y"]

    # Move Beaker to dice position and grab it
    beaker.write_cartesian_position([robot_x, robot_y, stack_z, -179.9, 0.0, 30.0])
    beaker.schunk_gripper("close")
    
    # Move to stacking position, incrementing height for each dice
    beaker.write_cartesian_position([stack_x, stack_y, stack_z + i * stack_height_increment, -179.9, 0.0, 30.0])
    beaker.schunk_gripper("open")

print("All dice stacked successfully.")




# Create  PDF compiled of all results
def create_PDF(dice_data, images, title_img):
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)

    #Title Page
    pdf.add_page()
    pdf.set_font("Trebuchet MS", "B", 16)
    pdf.cell(200, 10, "Dice Detection Report", ln=True, align="C")
    pdf.ln(10)

    # Image on Title Page
    for img in title_img:
        pdf.image(img,x=10, w=180)
        pdf.ln(10)

    # Dice Data Table
    pdf.set_font("Arial", "B", 12)
    pdf.cell(200, 10, "Dice Data Summary", ln=True, align="L")
    pdf.set_font("Arial", "", 10)
    pdf.ln(5)
    
    # Table headers
    pdf.cell(20, 10, "Dice#", 1)
    pdf.cell(20, 10, "Pips", 1)
    pdf.cell(50, 10, "Pixel Position", 1)
    pdf.cell(50, 10, "Robot Position", 1)
    pdf.cell(20, 10, "Rotation", 1)
    pdf.ln()

    # Dice data rows
    for die in dice_data:
        pdf.cell(20, 10, str(die["Dice"]), 1)
        pdf.cell(20, 10, str(die["pips"]), 1)
        pixel_pos = f"({die['pixel_center']['x']}, {die['pixel_center']['y']})"
        robot_pos = f"({die['robot_position']['x']:.2f}, {die['robot_position']['y']:.2f})"
        pdf.cell(50, 10, pixel_pos, 1)
        pdf.cell(50, 10, robot_pos, 1)
        pdf.cell(20, 10, f"{die['rotation']:.2f}", 1)
        pdf.ln()
    
    # Annotated Images Section
    pdf.add_page()
    pdf.set_font("Arial", "B", 12)
    pdf.cell(200, 10, "Annotated Images", ln=True, align="L")
    pdf.ln(10)

    # Insert images
    for img in images:
        pdf.image(img, x=10, w=180)
        pdf.ln(10)

    # Save the PDF
    pdf_output_path = "Dice_Report.pdf"
    pdf.output(pdf_output_path)
    print(f"PDF report created: {pdf_output_path}")

# List of images to include in the PDF
images = ['Dice_Filter.png', 'Check.png', 'Thresholded_ROI.png']
title_image = ['dice.png']

# Generate the PDF report
create_PDF(dice_data, images, title_image)