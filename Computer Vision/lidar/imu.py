import pyrealsense2 as rs
import socket
import struct

sock_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
imu_server_address  = ('192.168.1.110',10097)

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)

pipeline.start(config)

loopCount = 0

while True:

    frames = pipeline.wait_for_frames()

    accel_frame = frames.first_or_default(rs.stream.accel)
    if accel_frame:
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        xAccel = accel_data.x
        yAccel = accel_data.y
        zAccel = accel_data.z
    else:
        xAccel = 0
        yAccel = 0
        zAccel = 0

    gyro_frame= frames.first_or_default(rs.stream.gyro)
    if gyro_frame:
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
        xGyro = gyro_data.x
        yGyro = gyro_data.y
        zGyro = gyro_data.z
    else:
        xGyro = 0
        yGyro = 0
        zGyro = 0

    imu_udp  = bytearray(struct.pack("ffffff", xAccel, yAccel, zAccel, xGyro, yGyro, zGyro))

    sock_send.sendto(imu_udp, imu_server_address)

    if loopCount < 100:
        loopCount += 1
    else:
        loopCount = 0        
        print(f"xAccel={xAccel}, zAccel={zAccel}, yGyro={yGyro}.")


