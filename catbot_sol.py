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


# Create a bounding box around each downsampled point cloud and calculate its dimensions
bounding_boxes = {}

for file_name, point_cloud in downsampled_point_clouds.items():
# Compute the axis-aligned bounding box
    bounding_box = point_cloud.get_axis_aligned_bounding_box()
    bounding_boxes[file_name] = bounding_box

    # Get the dimensions of the bounding box
    min_bound = bounding_box.get_min_bound()
    max_bound = bounding_box.get_max_bound()
    dimensions = max_bound - min_bound

    # Print the dimensions
    print(f"File: {file_name}")
    print(f"Bounding Box Dimensions (x, y, z): {dimensions}")
    print("-" * 50)
    pass


# Stretch all point clouds to have x size = 150 and y size = 500
stretched_point_clouds = {}

for file_name, point_cloud in downsampled_point_clouds.items():
    # Get the bounding box dimensions
    bounding_box = bounding_boxes[file_name]
    min_bound = bounding_box.get_min_bound()
    max_bound = bounding_box.get_max_bound()
    dimensions = max_bound - min_bound

    # Calculate scaling factors for x and y axes
    scale_x = 150 / dimensions[0]
    scale_y = 500 / dimensions[1]
    scale_z = 1  # Keep z-axis scaling unchanged

    # Create a scaling matrix
    scaling_matrix = np.diag([scale_x, scale_y, scale_z])

    # Apply the scaling transformation
    points = np.asarray(point_cloud.points)
    scaled_points = np.dot(points, scaling_matrix)
    point_cloud.points = o3d.utility.Vector3dVector(scaled_points)

    # Store the stretched point cloud
    stretched_point_clouds[file_name] = point_cloud

    # Compute the new bounding box
    new_bounding_box = point_cloud.get_axis_aligned_bounding_box()
    new_min_bound = new_bounding_box.get_min_bound()
    new_max_bound = new_bounding_box.get_max_bound()
    new_dimensions = new_max_bound - new_min_bound

    # Print confirmation and new bounding box dimensions
    print(f"Stretched point cloud for file: {file_name}")
    print(f"New Bounding Box Dimensions (x, y, z): {new_dimensions}")
    print("-" * 50)