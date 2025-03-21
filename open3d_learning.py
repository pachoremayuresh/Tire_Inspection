import open3d as o3d
import numpy as np
import copy
from matplotlib.path import Path

import matplotlib.pyplot as plt

def generate_random_pointcloud():
    angles = np.random.uniform(80, 100, 4)  # Random angles in degrees
    angles = np.deg2rad(angles)  # Convert to radians
    lengths = np.random.uniform(1, 5, 4)  # Random lengths for each side

    points = [(0, 0, 0)]  # Start at origin
    current_angle = 0

    for i in range(4):
        x = points[-1][0] + lengths[i] * np.cos(current_angle)
        y = points[-1][1] + lengths[i] * np.sin(current_angle)
        points.append((x, y, 0))
        current_angle += angles[i]

    # Ensure the shape closes by removing the last duplicate point
    points = points[:-1]

    return np.array(points)

def fill_polygon_with_points(polygon, density=100):
    # Generate a grid of points within the bounding box of the polygon
    x_min, y_min = np.min(polygon[:, :2], axis=0)
    x_max, y_max = np.max(polygon[:, :2], axis=0)

    x_range = np.linspace(x_min, x_max, int((x_max - x_min) * density))
    y_range = np.linspace(y_min, y_max, int((y_max - y_min) * density))
    grid_x, grid_y = np.meshgrid(x_range, y_range)
    grid_points = np.c_[grid_x.ravel(), grid_y.ravel()]

    # Check which points are inside the polygon
    path = Path(polygon[:, :2])
    mask = path.contains_points(grid_points)

    # Return the points inside the polygon
    filled_points = grid_points[mask]
    return np.c_[filled_points, np.zeros(filled_points.shape[0])]

# Generate the polygon
polygon = generate_random_pointcloud()

# Fill the polygon with points
filled_points = fill_polygon_with_points(polygon)

# Create Open3D PointCloud object
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(filled_points)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])


# Perform voxel down sampling
voxel_size = 0.05  # Adjust the voxel size as needed
downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size=voxel_size)

# Visualize the downsampled point cloud
o3d.visualization.draw_geometries([downsampled_point_cloud])


