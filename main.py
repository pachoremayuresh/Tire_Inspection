import open3d as o3d
import numpy as np
from scipy.optimize import curve_fit

import matplotlib.pyplot as plt




# Read the PCD file
tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_1.pcd")

# Print basic information about the point cloud
print(tire)

# Perform voxel downsampling
tire = tire.voxel_down_sample(voxel_size=1.5)

# Print the downsampled point cloud
print(tire)


# Convert the point cloud to a numpy array
points = np.asarray(tire.points)

'''
# Find the minimum y value
y_min = np.min(points[:, 1])

# Define the range for y values
y_range_min = y_min
y_range_max = y_min + 1.5

# Create a set of points within the specified y range
temp = set(tuple(point) for point in points if y_range_min <= point[1] <= y_range_max)

# Print the size of the temp set
print(f"Number of points in the temp set: {len(temp)}")

# Extract x and z coordinates from the temp set
temp_points = np.array(list(temp))
x_data = temp_points[:, 0]
z_data = temp_points[:, 2]

# Define a function for curve fitting (e.g., a cubic function for better fitting)
def cubic_curve(x, a, b, c, d):
    return a * x**3 + b * x**2 + c * x + d

# Perform curve fitting
params, _ = curve_fit(cubic_curve, x_data, z_data)

# Extract the fitted parameters
a, b, c, d = params
print(f"Fitted curve parameters: a={a}, b={b}, c={c}, d={d}")

# Calculate the average distance between the points and the curve
z_predicted = cubic_curve(x_data, a, b, c, d)
distances = np.abs(z_data - z_predicted)
average_distance = np.mean(distances)
print(f"Average distance between points and the curve: {average_distance}")

# Identify points within 1.5 * average_distance
threshold = 1.5 * average_distance
close_points_mask = distances <= threshold

# Create a new point cloud for segmented tires
segmented_tires = o3d.geometry.PointCloud()

# Assign colors to points based on their distance to the curve
colors = np.zeros((len(temp_points), 3))  # Initialize all colors to black
colors[close_points_mask] = [0, 1, 0]  # Green for close points
colors[~close_points_mask] = [1, 0, 0]  # Red for far points

# Add points and their corresponding colors to the point cloud
segmented_tires.points = o3d.utility.Vector3dVector(temp_points)
segmented_tires.colors = o3d.utility.Vector3dVector(colors)

# Save the segmented point cloud to a file
o3d.io.write_point_cloud("segmented_tires.pcd", segmented_tires)

# Remove points in temp from the original tire point cloud
remaining_points = np.array([point for point in points if tuple(point) not in temp])

# Update the tire point cloud with the remaining points
tire.points = o3d.utility.Vector3dVector(remaining_points)

# Visualize the remaining point cloud
o3d.visualization.draw_geometries([tire])

# Visualize the segmented point cloud
o3d.visualization.draw_geometries([segmented_tires])

# Plot the original points, marking close points in green
plt.scatter(x_data[~close_points_mask], z_data[~close_points_mask], label="Data Points (Far)", color="blue")
plt.scatter(x_data[close_points_mask], z_data[close_points_mask], label="Data Points (Close)", color="green")

# Plot the fitted curve
x_fit = np.linspace(min(x_data), max(x_data), 500)
z_fit = cubic_curve(x_fit, a, b, c, d)
plt.plot(x_fit, z_fit, label="Fitted Curve", color="red")

plt.xlabel("X-axis")
plt.ylabel("Z-axis")
plt.legend()
plt.show()
plt.plot(x_fit, z_fit, label="Fitted Curve", color="red")
plt.xlabel("X-axis")
plt.ylabel("Z-axis")
plt.legend()
plt.show()

'''



# Calculate the average z value in the tire point cloud
average_z = np.mean(points[:, 2])
print(f"Average Z value: {average_z}")

# Create a mask for points with z values above or equal to the average
above_average_mask = points[:, 2] >= average_z

# Assign colors to points based on their z value
colors = np.zeros((len(points), 3))  # Initialize all colors to black
colors[above_average_mask] = [0, 1, 0]  # Green for points above or equal to the average

# Update the tire point cloud with the new colors
tire.colors = o3d.utility.Vector3dVector(colors)

# Visualize the updated point cloud
o3d.visualization.draw_geometries([tire])