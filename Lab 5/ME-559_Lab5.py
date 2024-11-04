from fpdf import FPDF
import timer
import subprocess
import cv2 as cv
import json

def Capture_image(type, file):
    # Run operation_script.py and capture any output or errors
    try:
        result = subprocess.run(
            [type, file],  # Adjust "python" if needed
            capture_output=True,
            text=True,
            check=True
        )
        print("Subprocess output:", result.stdout)
    except subprocess.CalledProcessError as e:
        print("An error occurred:", e.stderr)

    # Check for output files
    # Load and display the generated image
    try:
        image = cv.imread("output_image.png")
        if image is not None:
            cv.imshow("Generated Image", image)
            cv.waitKey(0)
            cv.destroyAllWindows()
        else:
            print("No image file found.")

        # Load and print the generated JSON data
        with open("output_data.json", "r") as json_file:
            data = json.load(json_file)
            print("Generated JSON data:", data)
    except FileNotFoundError as e:
        print("Output file not found:", e)





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