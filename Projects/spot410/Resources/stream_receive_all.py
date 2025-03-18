import socket
import cv2
import numpy as np
import struct
import threading
import psutil  # For checking and killing processes
import os
import signal
from flask import Flask, Response
import time

# Configuration
HOST = '0.0.0.0'   # Listen on all network interfaces
PORT = 5000         # Port for TCP streaming

# Globals
frame_lock = threading.Lock()
latest_frame = None

# Flask app for MJPEG streaming
app = Flask(__name__)

def terminate_process_using_port(port):
    """Check if a port is in use and terminate the process using it."""
    for proc in psutil.process_iter(['pid', 'name']):
        try:
            for conn in proc.connections(kind='inet'):
                if conn.laddr.port == port:
                    print(f"Terminating process {proc.info['name']} (PID {proc.info['pid']}) using port {port}.")
                    os.kill(proc.info['pid'], signal.SIGTERM)
                    return True
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass
    return False

@app.route('/video_feed')
def video_feed():
    def generate():
        global latest_frame
        while True:
            with frame_lock:
                if latest_frame is not None:
                    _, jpeg = cv2.imencode('.jpg', latest_frame)
                    if jpeg is not None:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            threading.Event().wait(0.03)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def start_server():
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

def handle_client(client_socket):
    global latest_frame
    try:
        while True:
            # Receive the length of the incoming data
            data_len_bytes = client_socket.recv(4)
            if not data_len_bytes:
                break
            data_len = struct.unpack('!I', data_len_bytes)[0]

            # Receive the actual data
            data = b''
            while len(data) < data_len:
                packet = client_socket.recv(data_len - len(data))
                if not packet:
                    break
                data += packet

            # Decode the JPEG data into an image
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)

            if image is not None:
                with frame_lock:
                    latest_frame = image
            else:
                print("Failed to decode image")

    except Exception as e:
        print(f"Error receiving data: {e}")
    finally:
        client_socket.close()

if __name__ == '__main__':
    # Check and terminate any process using the target port
    if terminate_process_using_port(PORT):
        print(f"Port {PORT} was in use. The process was terminated.")

    # Setup TCP server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of the port
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print(f"Listening for TCP connections on {HOST}:{PORT}")

    # Start Flask server for MJPEG streaming
    server_thread = threading.Thread(target=start_server, daemon=True)
    server_thread.start()

    # Accept TCP connections and handle clients
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Accepted connection from {addr}")
        client_thread = threading.Thread(target=handle_client, args=(client_socket,), daemon=True)
        client_thread.start()
