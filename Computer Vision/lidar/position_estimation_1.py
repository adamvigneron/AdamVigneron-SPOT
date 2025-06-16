# -*- coding: utf-8 -*-
"""
Created on Sun Mar 23 23:14:55 2025

@authors (best to worst): brianna, raph, thomas, daanp
"""


# -*- coding: utf-8 -*-
"""
Created on Mon Nov 25 10:49:36 2024


https://www.open3d.org/docs/latest/tutorial/pipelines/icp_registration.html
"""

# Import Statements
import pyrealsense2 as rs
import numpy as np
import time
import math
import threading
import sys
import open3d as o3d # 0.16.0
import cv2
import pickle
import os
import socket
import struct
import copy

#import tensorflow as tf

#from tensorflow.keras.models import load_model

#import matplotlib.pyplot as plt

#from tensorflow.keras.preprocessing.image import ImageDataGenerator

######################################### 1: User-defined Variables #######################################################
Select_Mode = "Live" # "Read" "Live"
platform = "Orin" # "Orin" "Raph" "Xavier"

send_package_udp =  True # [bool]

switch_distance = 1 # [m] If below, switch to short mode

visualizer_enabled = False # [bool]
remove_tank = True # from model

# Read recording parameters
sequence = 20
first_frame = 38
last_frame = 38

frames = np.linspace(first_frame, last_frame, last_frame-first_frame+1, endpoint = True, dtype=int)
######################################### 3: Functions #######################################################
def read_specific_frame(seg_num, frame_num):
    try:
        savedrecording = np.load(read_video_path+str(seg_num)+".npy", allow_pickle=True)
        my_frame_data = savedrecording[frame_num] # ["Depth Verts", "Time Stamp"]
        print(f"Video segment {seg_num}, frame {frame_num} successfully loaded")
    except Exception as e:
        print(e)
    return my_frame_data

def draw_result(source_pcd, target_pcd, transformation):
    source_pcd_temp = copy.deepcopy(source_pcd)
    target_pcd_temp = copy.deepcopy(target_pcd)
    source_pcd_temp.transform(transformation)
    source_pcd_temp.paint_uniform_color([0, 0, 1]) # Blue
    target_pcd_temp.paint_uniform_color([1, 0, 0]) # Red
    # X-axis: Red
    # Y-axis: Green
    # Z-axis: Blue
    o3d.visualization.draw_geometries([source_pcd_temp,target_pcd_temp, axis])

def dbscan_wall_remove(pcd):
    local_visualizer_enabled = False
    points = np.asarray(pcd.points)
    to_mask_labels = [-1]
    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    
    for label in np.linspace(0, max_label, max_label+1, endpoint = True, dtype=int):
        filtered_points_label = np.asarray(pcd.points)[labels == label]
        filtered_pcd_label = o3d.geometry.PointCloud()
        filtered_pcd_label.points = o3d.utility.Vector3dVector(filtered_points_label)
        max_bound_box = filtered_pcd_label.get_oriented_bounding_box(robust = True)
        box_extent = np.array(max_bound_box.extent)
        max_distance = np.sqrt(box_extent[0]**2+box_extent[1]**2+box_extent[2]**2)
        
        if visualizer_enabled and local_visualizer_enabled:
            print(max_distance)
            filtered_pcd_label.paint_uniform_color([0, 0, 1]) # Blue
            o3d.visualization.draw_geometries([filtered_pcd_label, axis])
        if max_distance > 2.5:
            to_mask_labels.append(label)
    mask = ~np.isin(labels, to_mask_labels)
    filtered_pcd_point = points[mask]
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_pcd_point)
    return filtered_pcd

def model_initialization(point_num, transform_model):
    global remove_tank
    origin = np.array([0, 0, 0])
    model = o3d.io.read_triangle_mesh(model_path)
    model_pcd = model.sample_points_poisson_disk(number_of_points=point_num)
    model_pcd = model_pcd.transform(transform_model)
    model_pcd.scale(0.001, origin)

    points = np.asarray(model_pcd.points)

    # Remove inside
    half_width = 0.15
    xmin = -half_width +0.005
    xmax = half_width -0.005
    ymin = -half_width +0.005
    ymax = half_width -0.005
    zmin = -1
    zmax = 2*half_width +0.02

    mask = np.logical_or(
    np.logical_or(
        (points[:, 0] < xmin) | (points[:, 0] > xmax),  # Check x-range
        (points[:, 1] < ymin) | (points[:, 1] > ymax)   # Check y-range
        ),
        (points[:, 2] < zmin) | (points[:, 2] > zmax)  # Check z-range
        )
    filtered_points = points[mask]
    
    # Remove Tank
    if remove_tank:
        half_width = 0.15
        xmin = -half_width
        xmax = half_width
        ymin = -half_width
        ymax = 0.02
        zmin = 2*half_width +0.02
        zmax = 3*half_width

        mask = np.logical_or(
        np.logical_or(
            (filtered_points[:, 0] < xmin) | (filtered_points[:, 0] > xmax),  # Check x-range
            (filtered_points[:, 1] < ymin) | (filtered_points[:, 1] > ymax)   # Check y-range
            ),
            (filtered_points[:, 2] < zmin) | (filtered_points[:, 2] > zmax)  # Check z-range
            )
        filtered_points = filtered_points[mask]

    filtered_model_pcd = o3d.geometry.PointCloud()
    filtered_model_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    return filtered_model_pcd

def wait_for_trigger(port=23456,buffer_size =1024):
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', port))  # '' binds to all interfaces, same as '0.0.0.0'

    print(f"Waiting for UDP trigger on port {port}...")

    data, _ = sock.recvfrom(buffer_size)


    message = struct.unpack('d', data)
    mes_int = int(message[0])
    #print(f"Received message: {message}")
    return mes_int == 1

def transform_builder(x, y, theta):
    # meters and rad
    my_transform = np.array([[np.cos(theta),-np.sin(theta),0,x],
                             [np.sin(theta),np.cos(theta),0,y],
                             [0,0,1,0],
                             [0,0,0,1]])
    return my_transform

def read_recorded_data_txt(sequence, frameNUM):
    end_recording = False
    point_cloud = o3d.geometry.PointCloud()
    my_frame_data = None
    try:
        point_cloud_txt = np.loadtxt(f"{read_data_path_txt}PC_{sequence}/point_cloud_{frameNUM:04d}.txt", delimiter=" ")
        point_cloud.points = o3d.utility.Vector3dVector(point_cloud_txt[:, :3])
        verts = np.array(point_cloud.points)
        my_frame_data = [verts, 0]
        print(f"Frame {frameNUM} successfully loaded")
    except:
        end_recording = True

    return my_frame_data, not(end_recording)

def read_recorded_ground_truth_txt(sequence, frameNUM):
    # x(m), y(m), theta(rad), other
    ground_truth_raw = np.loadtxt(f"{read_data_path_txt}Phase_{sequence}/phasespace_data_{frameNUM:04d}.txt", delimiter=" ")
    ground_truth_raw = ground_truth_raw[:3]
    return ground_truth_raw

def read_live_data():
    # Wait for a coherent pair of frames: depth and color       
    frames = pipeline.wait_for_frames()
    
    frame_timestamp = frames.get_timestamp()
    depth_frame = frames.get_depth_frame()
    #depth_frame = decimate.process(depth_frame) # The Decimate reduces resolution by 2 folds

    points = pc.calculate(depth_frame)

    # Pointcloud data to array
    v = points.get_vertices()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    return [verts, frame_timestamp]

def registration_ICP_point2plane(model_pcd, target_pcd, distance_threshold_ICP, transform):
    transform_ICP = o3d.pipelines.registration.registration_icp(
            model_pcd, target_pcd, distance_threshold_ICP, transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return transform_ICP

def registration_RANSAC(model_down, target_down, model_fpfh, target_fpfh, distance_threshold, confidence):
    global visualizer_enabled

    transform_RANSAC = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        model_down, target_down, model_fpfh, target_fpfh, True,
        distance_threshold, 
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, confidence))
 
    # if visualizer_enabled:
        # draw_result(model_down, target_down, transform_RANSAC.transformation)
    return transform_RANSAC

def estimate_pose(transformation):
    # Rz(gamma) Ry(beta) Rx(alpha) [degrees]
    gamma = np.arctan2(transformation[1,0], transformation[0,0])
    beta = np.arctan2(-transformation[2,0], np.sqrt(transformation[2,1]**2+transformation[2,2]**2))
    alpha = np.arctan2(transformation[2,1], transformation[2,2])
    
    # In Deg
    beta = round(np.rad2deg(beta),3) # Ry
    alpha = round(np.rad2deg(alpha),3) # Rx
    gamma = round(np.rad2deg(gamma),3) # Rz

    # Translations [m]
    x = round(transformation[0, 3],4)
    y = round(transformation[1, 3],4)
    z = round(transformation[2, 3],4)
    return [x, y, z, alpha, beta, gamma]

def pcd_downsample_fpfh(pcd, voxel_size, radius_normal, radius_feature):
    model_down = pcd.voxel_down_sample(voxel_size) # Downsample and estimate normals for point cloud
    model_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    # Compute FPFH features
    model_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        model_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return model_down, model_fpfh

def data_reduction_pcd(pcd):
    pcd_reduced = o3d.geometry.PointCloud()
    verts = np.array(pcd.points)

    # Remove points that are too far (X-axis)
    indicesXfar = np.where(verts[:,0] > 4)[0] # find where X > 4 (m)
    verts = np.delete(verts, indicesXfar, axis=0)

    # Remove points that are too close (X-axis)
    indicesXclose = np.where(verts[:,0] <= 0)[0] # find where X <= 0 (m)
    verts = np.delete(verts, indicesXclose, axis=0)

    # Remove points that are too high (Z-axis). Is the target reflection
    # indicesZhigh = np.where(verts[:,2] > 0.25)[0] # find where Z > threshold (m)
    # verts = np.delete(verts, indicesZhigh, axis=0)

    # Remove points that are too low (Z-axis).
    indicesZlow = np.where(verts[:,2] < -0.115)[0] # find where Z < threshold (m)
    verts = np.delete(verts, indicesZlow, axis=0)  
    
    pcd_reduced.points = o3d.utility.Vector3dVector(verts)
    pcd_reduced = pcd_reduced.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.8)[0]
    return pcd_reduced

# def convert_to_depth_im(target_pcd):
#     global transform_camera
#     global depth_folder

#     # convert back to original lidar ref frame for PC to depth image
#     transform_camera_inverse = np.linalg.inv(transform_camera)
#     target_pcd.transform(transform_camera_inverse)

#     # net was trained on these fx,fy
#     fx = 455
#     fy = 455
#     cx = 320  
#     cy = 240 
#     width, height = 640, 480  
    
#     filtered_points  = np.asarray(target_pcd.points) # convert to np

#     depth_image = np.zeros((height, width), dtype=np.float32) # make empty im frame
 
#     for X, Y, Z in filtered_points:
#         if Z > 0: 
#             u = int((X * fx) / Z + cx)
#             v = int((Y * fy) / Z + cy)
#             if 0 <= u < width and 0 <= v < height: 
#                 depth_image[v, u] = Z  
    
#     #remove image file saving once sure NN actually works
#     depth_path = os.path.join(depth_folder,"depth_im.png")
#     plt.imsave(depth_path,depth_image,cmap='gray')
#     print("Image Proccessed")

#     return depth_image

# def neural_net(model):
#     IMG_SIZE = (320, 240)  # desired image size (original: 640,480)
#     global temporary_dir
#     test_datagen = ImageDataGenerator(rescale=1./255)
#     test_dataset = test_datagen.flow_from_directory(temporary_dir,
#                                                     target_size=IMG_SIZE,
#                                                     class_mode="categorical",
#                                                     color_mode="grayscale",
#                                                     shuffle=False)

#     predictions = model.predict(test_dataset)
#     predicted_classes = tf.argmax(predictions, axis=1).numpy()
#     confidence_scores = tf.reduce_max(predictions, axis=1).numpy()

#     class_names = list(test_dataset.class_indices.keys())

#     predicted_class_values = class_names[predicted_classes[0]] 
#     confidence = confidence_scores[0] 

#     return predicted_class_values , confidence

"""Initialization"""

######################################### 1: Initialization ######################################################
# Camera frame to SPOT frame transform
Identity = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
RotXNeg90 = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]])
RotY90 = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])
RotZ90 = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
RotZ180 = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
RotZNeg90 = np.array([[0,1,0,0],[-1,0,0,0],[0,0,1,0],[0,0,0,1]])
RotZNeg180 = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
RotX90 = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]])
transform_camera = np.dot(RotY90, RotZNeg90)
# transform_camera[0,3] = 0.15
# transform_camera[1,3] = 0.087

origin = np.array([0, 0, 0])
axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=origin)

# RANSAC and ICP parameters
# Promising test result: values from 712

model_sampling = 10000 # number of points
voxel_size_RANSAC = 0.01  # (m) Change depending on point cloud size (0.05)
radius_normal_RANSAC = voxel_size_RANSAC * 2
radius_feature_RANSAC = voxel_size_RANSAC * 3 # (10)
distance_threshold_RANSAC = voxel_size_RANSAC * 8 # 4
ransac_confidence = 0.9999 # (0.999)

# Duplicated from RANSAC
voxel_size_ICP = 0.01
distance_threshold_ICP = voxel_size_ICP * 8 # 4

# Set script states
read_recording = True if Select_Mode == "Read" else (False)

# UDP Setup
if send_package_udp:
    sock_send = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    server_address_send = ('192.168.1.110',10099)
    time_server_address = ('192.168.1.110',10098)

# Paths
if platform == "Orin":
    read_video_path = "/home/spot-vision/Documents/o3d_icp/recording/myrecording" # Path String with file name, without index nor extension
    record_video_path = "/home/spot-vision/Documents/o3d_icp/recording/myrecording" # Path String with file name, without index nor extension
    pose_estimation_path = "/home/spot-vision/Documents/o3d_icp/pose_estimation/pose_estimation.csv" # Path String with file name with extension
    model_path = "/home/spot-vision/Documents/LIDAR_Example/box_centered.stl"
    model_pc_path = "/home/spot-vision/Documents/o3d_icp/Target_v39_pc.npy"

    result_pc_path = "/home/spot-vision/Documents/o3d_icp/pose_estimation/"
  #  nn_model_path = "/home/spot-vision/Documents/o3d_icp/model_capstone_mar27_t3.h5"
  #  temporary_dir = "/home/spot-vision/Documents/o3d_icp/NN_temp_dir" #empty folder to temporarily hold depth ims for NN proccessing
    
    # if not os.path.exists(temporary_dir):
    #     os.makedirs(temporary_dir)
    # depth_folder = os.path.join(temporary_dir, "Back")
    # depth_folderB = os.path.join(temporary_dir, "Front")
    # depth_folderC = os.path.join(temporary_dir, "Side")

    # if not os.path.exists(depth_folder):
    #     os.makedirs(depth_folder)
    # if not os.path.exists(depth_folderB):
    #     os.makedirs(depth_folderB)
    # if not os.path.exists(depth_folderC):
    #     os.makedirs(depth_folderC)

elif platform == "Raph":
    read_video_path = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/recording/myrecording" # Path String with file name, without index nor extension
    record_video_path = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/recording/myrecording" # Path String with file name, without index nor extension
    pose_estimation_path = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/pose_estimation/pose_estimation.csv" # Path String with file name with extension
    model_path = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/Target_v39.stl"
    model_pc_path = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/Target_v39_pc.npy"

    result_pc_path = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/Data/test/"
    read_data_path_txt = "E:/Ecole/Carleton/Fall2024/MAAE4907Q/Scripts/o3d_icp/Data/"


elif platform == "Xavier":
    print("Not set for Xavier")
    exit(0)

# Model Initialization
try:
    print("Initializing Model: Start")
    model_pcd = model_initialization(model_sampling, RotZ180)
    model_down, model_fpfh = pcd_downsample_fpfh(model_pcd, voxel_size_RANSAC, radius_normal_RANSAC, radius_feature_RANSAC) # Downsample and estimate normals for point cloud
    model_down.paint_uniform_color([1, 0, 0]) # Red
    print("Initializing Model: End")

    # print("Loading Neural Net...")
    # model = load_model(nn_model_path,compile=False)
    # model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    # print("Net Loaded.")

except KeyboardInterrupt:
    print("\nCtrl+C pressed, exiting loop...")
    exit(0)
except Exception as e:
    print(e)
    print("Could not find Target Model.")
    exit(0)

# Visualized Model
#if visualizer_enabled:
#    o3d.visualization.draw_geometries([model_down, axis])

# Configure depth stream
if not read_recording:    
    pipeline = rs.pipeline()
    config = rs.config()
    

    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    try:
        pipeline_profile = config.resolve(pipeline_wrapper)
    except:
        print("Connect L515 Camera")
        exit(0)
    device = pipeline_profile.get_device()

    # Realsense Objects Initialization
    pc = rs.pointcloud()
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    sensor = profile.get_device().first_depth_sensor()
    # sensor = pipeline.get_active_profile().get_device().first_depth_sensor()

    sensor.set_option(rs.option.visual_preset,4)
    current_preset = sensor.get_option(rs.option.visual_preset)

# Script Variables - General
ground_truth_x = 0
ground_truth_y = 0
ground_truth_theta = 0

lidar_mode = "Long"

target_pcd = o3d.geometry.PointCloud()
recording_header = ["Depth Verts", "Time Stamp"]
recording = [recording_header]
transform = None
result_array_header = ["script_start_timestamp", "frame", "registration_type", "prediction", "registration_start_timestamp", "registration_duration",
                          "x", "y", "z", "Rx", "Ry", "Rz",
                          "fitness", "rmse", "skipped_count",
                          "ground_truth_x", "ground_truth_y", "ground_truth_theta"]
result_array = [result_array_header]

frame_iterator = 0
icp_skip_count = 0
skipped_count = 0
max_skip = 20

frame = 0


######################################### 3: Main Script #######################################################
print(f"Selected Mode: {Select_Mode}, Platform: {platform}, UDP Enabled: {send_package_udp}")

print("Waiting for trigger to start RANSAC... (press Ctrl+C to abort)")

if True:
    print("Trigger received! Running conditional section...")

    script_start_timestamp = time.strftime('%H:%M:%S')
    start_time = time.perf_counter()

    try:
        transform = None
        reset_counter = 1
        run_loop = True

        while_iteration = 0
        while_start = time.perf_counter()
        
        while run_loop:       
            prediction = None
            registration_type = None
            pose = [0, 0, 0, 0, 0, 0]
            ground_truth_x, ground_truth_y, ground_truth_theta = 0, 0, 0
            fitness = 0
            rmse = 0       
            registration_start_timestamp = 0
            registration_duration = 0

            # Live Camera
            if not read_recording:
                frame_data = read_live_data() # [verts, frame_timestamp]
            
            # Reading a Recorded Video
            elif read_recording:
                try:
                    frame = frames[frame_iterator]
                    frame_iterator += 1
                except:
                    run_loop = False
                    print("End of frame list reached. Exiting script.")
                    continue
                
                frame_data, run_loop = read_recorded_data_txt(sequence, frame) # read_recorded_data sets run_loop used to flag the end of the recording
                if not(run_loop):
                    print(f"Frame not found. Exiting script.")
                    continue
                ground_truth_x, ground_truth_y, ground_truth_theta = read_recorded_ground_truth_txt(sequence, frame)
                ground_truth_theta = np.rad2deg(ground_truth_theta)

            verts = frame_data[0]
            target_pcd.points = o3d.utility.Vector3dVector(verts)
            target_pcd.transform(transform_camera)
            target_pcd = data_reduction_pcd(target_pcd)
            
            ### GLOBAL REGISTRATION ###
            if transform is None:
                # ### DBSCAN Wall removal
                # target_pcd.paint_uniform_color([0, 0, 1]) # Blue
                # if visualizer_enabled and False:
                #     print("Vizualizing pre-dbscan removal")
                #     o3d.visualization.draw_geometries([target_pcd, axis])
                # target_pcd = dbscan_wall_remove(target_pcd) 
                # if visualizer_enabled and False:
                #     print("Vizualizing post-dbscan removal")
                #     o3d.visualization.draw_geometries([target_pcd, axis])
                try:
                    target_down, target_fpfh = pcd_downsample_fpfh(target_pcd, voxel_size_RANSAC, radius_normal_RANSAC, radius_feature_RANSAC)
                except KeyboardInterrupt:
                    print("\nCtrl+C pressed, exiting loop...")
                    break          
                except Exception as e:
                    print(e)
                    print(f"{time.strftime('%H:%M:%S')} Point Cloud Empty: Can't calculate FPFH. Next Frame")
                    continue


                registration_start_timestamp = time.strftime('%H:%M:%S')
                registration_type = "Global"
                global_start = time.time()
                
                # Neural Net Stuff
                #depth_image = convert_to_depth_im(target_pcd)
                #prediction, confidence = neural_net(model)
                #print("Prediction: ", prediction, "| Confidence: ", confidence)
             
                # prediction, confidence = "Front", 0.99 # TO REPLACE WITH BRIANNA's NN
                # if prediction == "Front" and confidence > 0.5: # valid_Rz = [0, 180]
                #     brute_angles_deg = [15, 45, 75, 105, 135, 165]
                #     print(f"NN Prediction: {prediction}.")
                    
                # elif prediction == "Back" and confidence > 0.5:# valid_Rz = [-180, 0]
                #     brute_angles_deg = [-165, -135, -105, -75, -45, -15]
                #     print(f"NN Prediction: {prediction}.")
                    
                # else:
                #     # Catch invalid frame (non ideal).
                #     print(f"NN Prediction: {prediction}. Skipping frame")
                #     continue
                
                brute_angles_deg = [15, 45, 75, 105, 135, 165]

                # RANSAC
                try: 
                    transform_temp = registration_RANSAC(model_down, target_down, model_fpfh, target_fpfh, distance_threshold_RANSAC, ransac_confidence)        
                    RANSAC_pose = estimate_pose(transform_temp.transformation) # X, Y, Z, Rx, Ry, Rz (millimeters and degrees, relative Chaser-Target)
                    if visualizer_enabled:
                        draw_result(model_down, target_down, transform_temp.transformation)
                    if RANSAC_pose[2] > 0.3:
                        print(f"{time.strftime('%H:%M:%S')} RANSAC Position Estimation out of bound (z > 0.3m). Next Frame")
                        continue
                    # Add condition for max Y? Wouldnt work for current dataset. Might work for actual experiment

                # ICP for Global, brute force
                    target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                    best_brute_ICP_fitness = 0
                    best_brute_ICP_transform = None
                    for angle_deg in brute_angles_deg:
                        transform_temp = transform_builder(RANSAC_pose[0], RANSAC_pose[1], np.deg2rad(angle_deg))
                        transform_temp = registration_ICP_point2plane(model_down, target_down, distance_threshold_ICP, transform_temp)
                        
                        if transform_temp.fitness > best_brute_ICP_fitness:
                            best_brute_ICP_fitness = transform_temp.fitness
                            best_brute_ICP_transform = transform_temp

                    best_brute_ICP_pose = estimate_pose(best_brute_ICP_transform.transformation)
                    if -10 < best_brute_ICP_pose[3] < 10 and -10 < best_brute_ICP_pose[4] < 10:
                        # The Global registration output is valid and recorded
                        transform = best_brute_ICP_transform
                        transform_temp = transform # For visualizaer
                        pose = estimate_pose(transform.transformation)
                        fitness = transform.fitness
                        registration_duration = time.time() - global_start
                    else:
                        print(f"{time.strftime('%H:%M:%S')} No valid Global Registration Transform. Next Frame")
                        continue

                except KeyboardInterrupt:
                    print("\nCtrl+C pressed, exiting loop...")
                    break          
                except Exception as e:
                    print(e)
                    transform = None
                    print(f"{time.strftime('%H:%M:%S')} Global Registration Error. Next Frame")
                    continue

                #else:
                #    print("No valid trigger received. Skipping section.")
                    
            ### LOCAL REGISTRATION ###
            else:    
                registration_type = "Local"
                try:
                    target_down, target_fpfh = pcd_downsample_fpfh(target_pcd, voxel_size_RANSAC, radius_normal_RANSAC, radius_feature_RANSAC)
                    registration_start_timestamp = time.strftime('%H:%M:%S')
                    local_start = time.time()
                    target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
                    transform_temp = registration_ICP_point2plane(model_down, target_down, distance_threshold_ICP, transform.transformation)
                    print(f"{time.strftime('%H:%M:%S')} ICP Success")
                    pose = estimate_pose(transform_temp.transformation) # X, Y, Z, Rx, Ry, Rz (meters and degrees, relative Chaser-Target)
                    #if -35 < pose[3] < 35 and -35 < pose[4] < 35:
                    #if lidar_mode == "Short" or -35 < pose[3] < 35 and -35 < pose[4] < 35:
                        # The Local registration output is valid and recorded
                    skipped_count = icp_skip_count # To record count frame skipped
                    icp_skip_count = 0
                    transform = transform_temp
                    registration_duration = time.time() - local_start
                    # else:
                    #     icp_skip_count += 1
                    #     skipped_count = icp_skip_count
                    #     print(f"{time.strftime('%H:%M:%S')} Invalid Local Output. Skipping Frame. Skip count: {icp_skip_count}")
                    #     if visualizer_enabled:
                    #         draw_result(model_down, target_down, transform_temp.transformation)
                    #     if icp_skip_count > max_skip:
                    #         transform = None
                    #         print(f"{time.strftime('%H:%M:%S')} Skip count exceed {max_skip}. Reverting to Global Registration")
                    #     continue
                
                except Exception as e:
                    print(e)
                    icp_skip_count += 1
                    print(f"{time.strftime('%H:%M:%S')} ICP Fail")
                    continue

                while_iteration +=1
            
            # Should never reach this if no valid Global or Local registration result
            # Results
            fitness = transform.fitness
            rmse = transform.inlier_rmse
            transform_result_x = pose[0]+0.145
            transform_result_y = pose[1]+0.11
            transform_result_theta = pose[5]

            end_time = time.perf_counter()
            computation_time = end_time - start_time
            elapsed_time = while_start - end_time
            while_start = end_time
            
            #print(f"Pose: X = {transform_result_x}, Y = {transform_result_y}, Theta = {transform_result_theta}")
            print(f"Compute Time: t = {computation_time} ,Iteration Time: it = {elapsed_time} ,Pose: X = {transform_result_x}, Y = {transform_result_y}, Theta = {transform_result_theta}")    
            
            if send_package_udp:
                udp_data = bytearray(struct.pack("fffff",transform_result_x,transform_result_y,np.deg2rad(transform_result_theta),rmse, reset_counter))
                time_udp = bytearray(struct.pack("fffffffff", computation_time, elapsed_time, 1, skipped_count, 1, registration_duration, rmse, fitness,while_iteration))
                try:
                    sock_send.sendto(udp_data,server_address_send)
                    #print(f"{time.strftime('%H:%M:%S')} UPD package sent: x={transform_result_x}, y={transform_result_y}, theta={np.deg2rad(transform_result_theta)}.")
                    
                    sock_send.sendto(time_udp,time_server_address)
                    print(f"{time.strftime('%H:%M:%S')},Compute Time: t = {computation_time} , UDP package sent: x={transform_result_x}, y={transform_result_y}, theta={np.deg2rad(transform_result_theta)}.")
                except Exception as e:
                    print(e)
                    print(f"{time.strftime('%H:%M:%S')} Failed to send UDP package")

            result_array.append([script_start_timestamp, frame, registration_type, prediction, registration_start_timestamp, registration_duration,
                                    transform_result_x, transform_result_y, pose[2], pose[3], pose[4], pose[5],
                                    fitness, rmse, skipped_count,
                                    ground_truth_x, ground_truth_y, ground_truth_theta])

            if visualizer_enabled:
                draw_result(model_down, target_down, transform_temp.transformation)
            
            reset_counter +=1

            # Check if need sensor mode switch
            if not read_recording and lidar_mode == "Long" and transform_result_x < switch_distance:
                print('######################################################################################################################################################################################################################################################################################################')
                lidar_mode = "Short"
                sensor.set_option(rs.option.visual_preset,3)
                
                #sensor.set_option(rs.option.noise_filtering, 6)
                #sensor.set_option(rs.option.confidence_threshold, 3)
                #sensor.set_option(rs.option.receiver_gain,8)

                current_preset = sensor.get_option(rs.option.visual_preset)


    except Exception as e:
        print(e)

    finally:
        np.savetxt(pose_estimation_path, result_array, fmt="%s", delimiter=',')
        if not read_recording:
            pipeline.stop()
            profile = pipeline = None

        if send_package_udp:
            sock_send.close()

else:
    print("No valid trigger received. Skipping section.")
