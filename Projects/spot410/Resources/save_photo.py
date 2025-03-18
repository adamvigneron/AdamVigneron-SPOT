import cv2
import pyzed.sl as sl
import os
import signal
import sys
import time

# Configuration
SAVE_DIR = '/home/spot-spare-red/zed-photos/'

# Ensure the directory exists
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# Initialize ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 15

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Error opening ZED camera")
    zed.close()
    exit()

def cleanup_and_exit(signal_received=None, frame=None):
    """Cleanup resources and exit gracefully."""
    print("Releasing resources and exiting...")
    zed.close()
    print("Resources released. Exiting.")
    sys.exit(0)

# Register signal handler for graceful termination
signal.signal(signal.SIGINT, cleanup_and_exit)  # Handle Ctrl+C
signal.signal(signal.SIGTERM, cleanup_and_exit)  # Handle termination from shell

try:
    # Grab a frame from the ZED camera
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image
        left_image = sl.Mat()
        zed.retrieve_image(left_image, sl.VIEW.LEFT)

        # Retrieve right image
        right_image = sl.Mat()
        zed.retrieve_image(right_image, sl.VIEW.RIGHT)

        # Convert to NumPy arrays for saving
        left_image_np = left_image.get_data()
        right_image_np = right_image.get_data()

        # Create filenames with timestamps
        left_filename = os.path.join(SAVE_DIR, "left_frame.jpg")
        right_filename = os.path.join(SAVE_DIR, "right_frame.jpg")

        # Save the images as JPEG
        cv2.imwrite(left_filename, left_image_np)
        cv2.imwrite(right_filename, right_image_np)

    else:
        print("Error capturing frame")

except KeyboardInterrupt:
    print("Capture stopped by user")
    cleanup_and_exit()

finally:
    cleanup_and_exit()
