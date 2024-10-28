import gxipy as gx
from PIL import Image
import cv2 as cv


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
    rgb_image.image_improvement(color_correction, contrast_lut, gamma_lut)

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

# Open saved image with OpenCV
image = cv.imread(filename)
cv.namedWindow('Captured Dice Image', cv.WINDOW_NORMAL)
cv.imshow('Captured Dice Image', image)
cv.waitKey(0)

# End image acquisition and close device
camera.stream_off()
camera.close_device()