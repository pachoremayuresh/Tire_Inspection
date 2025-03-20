import open3d as o3d
import numpy as np
import copy

import matplotlib.pyplot as plt





# Create a rectangle-shaped point cloud
width = 1.0  # 100 cm
height = 0.5  # 50 cm
num_points_width = 100
num_points_height = 50

# Generate points for the rectangle
x = np.linspace(0, width, num_points_width)
y = np.linspace(0, height, num_points_height)
xx, yy = np.meshgrid(x, y)
points = np.vstack((xx.ravel(), yy.ravel(), np.zeros_like(xx.ravel()))).T

# Create Open3D point cloud object
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])



# Add random height variations to some points
num_points_to_modify = int(0.1 * len(points))  # Modify 10% of the points
indices_to_modify = np.random.choice(len(points), num_points_to_modify, replace=False)
height_variations = np.random.uniform(-0.01, 0.01, num_points_to_modify)  # Height difference of ±1 cm

# Apply height variations
points[indices_to_modify, 2] += height_variations

# Update the point cloud with modified points
point_cloud.points = o3d.utility.Vector3dVector(points)

rectangle = point_cloud

# Visualize the modified point cloud
o3d.visualization.draw_geometries([point_cloud])



# Create a parallelogram-shaped point cloud
angle = np.radians(60)  # 60 degrees in radians
width = 1.0  # 100 cm
height = 0.5  # 50 cm
num_points_width = 100
num_points_height = 50

# Generate points for the parallelogram
x = np.linspace(0, width, num_points_width)
y = np.linspace(0, height, num_points_height)
xx, yy = np.meshgrid(x, y)

# Apply the parallelogram transformation
xx_transformed = xx + yy * np.tan(angle)
points_parallelogram = np.vstack((xx_transformed.ravel(), yy.ravel(), np.zeros_like(xx.ravel()))).T

# Create Open3D point cloud object for the parallelogram
parallelogram_point_cloud = o3d.geometry.PointCloud()
parallelogram_point_cloud.points = o3d.utility.Vector3dVector(points_parallelogram)

# Visualize the parallelogram point cloud
o3d.visualization.draw_geometries([parallelogram_point_cloud])





# Transform the rectangle to a parallelogram with a 60-degree angle
rectangle_points = np.asarray(rectangle.points)

# Apply the parallelogram transformation
rectangle_points[:, 0] += rectangle_points[:, 1] * np.tan(angle)

# Update the rectangle point cloud with transformed points
rectangle.points = o3d.utility.Vector3dVector(rectangle_points)

# Visualize the transformed parallelogram point cloud
o3d.visualization.draw_geometries([rectangle])

# Calculate the width and height of the parallelogram
rectangle_points = np.asarray(rectangle.points)

# Find the minimum and maximum x and y coordinates
min_x, max_x = np.min(rectangle_points[:, 0]), np.max(rectangle_points[:, 0])
min_y, max_y = np.min(rectangle_points[:, 1]), np.max(rectangle_points[:, 1])

# Calculate width and height
parallelogram_width = max_x - min_x
parallelogram_height = max_y - min_y

print(f"Parallelogram Width: {parallelogram_width}")
print(f"Parallelogram Height: {parallelogram_height}")

# Cut a square of 50 * 50 cm from the center of the parallelogram
center_x = parallelogram_width / 2
center_y = parallelogram_height / 2
half_square_size = 0.25  # 50 cm / 2

# Extract points within the square
rectangle_points = np.asarray(rectangle.points)
mask = (
    (rectangle_points[:, 0] >= center_x - half_square_size) &
    (rectangle_points[:, 0] <= center_x + half_square_size) &
    (rectangle_points[:, 1] >= center_y - half_square_size) &
    (rectangle_points[:, 1] <= center_y + half_square_size)
)

# Create a new point cloud for the square
image1_points = rectangle_points[mask]
image1 = o3d.geometry.PointCloud()
image1.points = o3d.utility.Vector3dVector(image1_points)

# Visualize the square
o3d.visualization.draw_geometries([image1])

# Create a point cloud for the remaining parallelogram
remaining_points = rectangle_points[~mask]
remaining_parallelogram = o3d.geometry.PointCloud()
remaining_parallelogram.points = o3d.utility.Vector3dVector(remaining_points)

# Visualize the remaining parallelogram
o3d.visualization.draw_geometries([remaining_parallelogram])


# Stretch/shrink image1 to fill an 80 * 30 cm rectangle
target_width = 0.8  # 80 cm
target_height = 0.3  # 30 cm

# Get the current bounds of image1
image1_points = np.asarray(image1.points)
min_x, max_x = np.min(image1_points[:, 0]), np.max(image1_points[:, 0])
min_y, max_y = np.min(image1_points[:, 1]), np.max(image1_points[:, 1])

# Calculate scaling factors
scale_x = target_width / (max_x - min_x)
scale_y = target_height / (max_y - min_y)

# Apply scaling
image1_points[:, 0] = (image1_points[:, 0] - min_x) * scale_x
image1_points[:, 1] = (image1_points[:, 1] - min_y) * scale_y

# Update the point cloud
image1.points = o3d.utility.Vector3dVector(image1_points)

# Create rectangle border point clouds
border_points = []

# Bottom border
border_points.append(np.column_stack((np.linspace(0, target_width, 100), np.zeros(100), np.zeros(100))))

# Top border
border_points.append(np.column_stack((np.linspace(0, target_width, 100), np.full(100, target_height), np.zeros(100))))

# Left border
border_points.append(np.column_stack((np.zeros(100), np.linspace(0, target_height, 100), np.zeros(100))))

# Right border
border_points.append(np.column_stack((np.full(100, target_width), np.linspace(0, target_height, 100), np.zeros(100))))

# Combine all border points
border_points = np.vstack(border_points)

# Create Open3D point cloud for the border
border_point_cloud = o3d.geometry.PointCloud()
border_point_cloud.points = o3d.utility.Vector3dVector(border_points)

# Visualize the stretched/shrunk image1 with the rectangle border
o3d.visualization.draw_geometries([image1, border_point_cloud])


