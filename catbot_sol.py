import os

import cv2  # OpenCV for traditional computer vision tasks
import numpy as np  # NumPy for numerical operations
import matplotlib.pyplot as plt  # For visualizing images
import open3d as o3d  # For working with 3D images and point clouds

# Path to the folder containing .pcd files
folder_path = "/home/mayuresh/Desktop/Image_Processing/Tire_Inspection/Tire_3D_Images"

# List to store the loaded point clouds
point_clouds = {}

# Read all .pcd files in the folder
for file_name in os.listdir(folder_path):
    if file_name.endswith(".pcd"):
        file_path = os.path.join(folder_path, file_name)
        point_cloud = o3d.io.read_point_cloud(file_path)
        point_clouds[file_name] = point_cloud
        pass
    pass


# Now `point_clouds` contains all the loaded point clouds with file names as keys



# Downsample all the point clouds to a voxel size of 1.5
voxel_size = 1.5
downsampled_point_clouds = {}

for file_name, point_cloud in point_clouds.items():
    downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
    downsampled_point_clouds[file_name] = downsampled_point_cloud
    pass


