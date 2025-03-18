import cv2
import socket
import pyzed.sl as sl
import struct
import signal
import sys

# Configuration
SERVER_IP = '192.168.1.104'  # Replace with receiver's IP
SERVER_PORT = 5000           # Port for TCP streaming

# Initialize ZED camera
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 15

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Error opening ZED camera")
    zed.close()
    exit()

# Setup TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((SERVER_IP, SERVER_PORT))

def cleanup_and_exit(signal_received=None, frame=None):
    """Cleanup resources and exit gracefully."""
    print("Stopping stream and releasing resources...")
    zed.close()
    sock.close()
    print("Resources released. Exiting.")
    sys.exit(0)

# Register signal handler for graceful termination
signal.signal(signal.SIGINT, cleanup_and_exit)  # Handle Ctrl+C
signal.signal(signal.SIGTERM, cleanup_and_exit)  # Handle termination from shell

try:
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image
            image = sl.Mat()
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Convert to NumPy array and encode as JPEG
            image_np = image.get_data()
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]  # Adjust quality for bandwidth
            result, buffer = cv2.imencode('.jpg', image_np, encode_param)
            
            if result:
                data = buffer.tobytes()
                data_len = len(data)

                # Send the length of the data first
                sock.sendall(struct.pack('!I', data_len))
                # Send the JPEG data
                sock.sendall(data)
            else:
                print("Failed to encode image")
        else:
            print("Error capturing frame")
        
except KeyboardInterrupt:
    print("Streaming stopped by user")
    cleanup_and_exit()
finally:
    cleanup_and_exit()
