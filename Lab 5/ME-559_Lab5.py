from fpdf import FPDF

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