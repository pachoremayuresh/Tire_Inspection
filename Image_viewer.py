
# Importing the required libraries

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt



# Reading the Images
Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_1.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_2.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_3.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_4.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_5.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_6.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_7.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_8.pcd")
# Tire = o3d.io.read_point_cloud("Tire_3D_Images/Ideal_Image.pcd")

print(Tire)

# Find the point with minimum x and minimum y values

# Perform voxel down sampling
voxel_size = 1.5  # Set the voxel size
downsampled_tire = Tire.voxel_down_sample(voxel_size)

# Print the downsampled point cloud
print("Downsampled point cloud:")
print(downsampled_tire)

# Convert the point cloud to a numpy array
points = np.asarray(downsampled_tire.points)

# Find the minimum y value
y_min = np.min(points[:, 1])

# Define the range for y values
y_range_min = y_min
y_range_max = y_min + 1.5

# Filter points within the specified y range
mask = (points[:, 1] >= y_range_min) & (points[:, 1] <= y_range_max)

# Create colors for the points
colors = np.zeros_like(points)  # Initialize all colors to black
colors[:, 1] = 1.0  # Set all points to green
colors[mask] = [0, 0, 0]  # Set points in the range to black

# Exclude the black points from the downsampled_tire
remaining_points = points[~mask]

# Find the next minimum y value in the remaining points
if len(remaining_points) > 0:
    y_min_next = np.min(remaining_points[:, 1])

    # Define the range for the next y values
    y_range_min_next = y_min_next
    y_range_max_next = y_min_next + 1.5

    # Filter points within the next specified y range
    mask_next = (remaining_points[:, 1] >= y_range_min_next) & (remaining_points[:, 1] <= y_range_max_next)

    # Set points in the next range to red
    colors_next = np.zeros_like(remaining_points)
    colors_next[:, 1] = 1.0  # Default green
    colors_next[mask_next] = [1, 0, 0]  # Red for the next range

    # Combine the colors back
    colors[~mask] = colors_next

# Assign the colors to the point cloud
downsampled_tire.colors = o3d.utility.Vector3dVector(colors)

# Visualize the point cloud
o3d.visualization.draw_geometries([downsampled_tire])


'''
points = np.asarray(downsampled_tire.points)

# Find the minimum y value
y_min = np.min(points[:, 1])

# Define the range for y values
y_range_min = y_min
y_range_max = y_min + 1.5

# Filter points within the specified y range
mask = (points[:, 1] >= y_range_min) & (points[:, 1] <= y_range_max)

# Create colors for the points
colors = np.zeros_like(points)  # Initialize all colors to black
colors[:, 1] = 1.0  # Set all points to green
colors[mask] = [0, 0, 0]  # Set points in the range to black

# Assign the colors to the point cloud
downsampled_tire.colors = o3d.utility.Vector3dVector(colors)

# Visualize the point cloud
o3d.visualization.draw_geometries([downsampled_tire])

'''