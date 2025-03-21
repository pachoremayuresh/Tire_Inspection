#%%
import open3d as o3d
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


#%%

# Read the PCD file

#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_1.pcd")
#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_2.pcd")
#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_3.pcd")
#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_4.pcd")
tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_5.pcd")
#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_6.pcd")
#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_7.pcd")
#tire = o3d.io.read_point_cloud("Tire_3D_Images/negative_8.pcd")

print(tire)

o3d.visualization.draw_geometries([tire])

# Perform voxel downsampling

tire = tire.voxel_down_sample(voxel_size=1.5)
print(tire)

#o3d.visualization.draw_geometries([tire])

'''

# Data Filtering
# Calculate the average z value
points = np.asarray(tire.points)
average_z = np.mean(points[:, 2])

# Filter out points below the average z value
filtered_points = points[points[:, 2] >= average_z]

# Update the point cloud with the filtered points
tire.points = o3d.utility.Vector3dVector(filtered_points)

# Display the filtered point cloud
o3d.visualization.draw_geometries([tire])

'''

#%%
#Funtion to select points in a line
def points_selector(points, range):
    y_min = np.min(points[:, 1])
    y_range_min = y_min
    y_range_max = y_min + range
    temp = set(tuple(point) for point in points if y_range_min <= point[1] <= y_range_max)
    return np.array(list(temp))

def cubic_curve(x, a, b, c, d):
    return a * x**3 + b * x**2 + c * x + d

def filtering(points,threshold):
    x_data = points[:, 0]
    z_data = points[:, 2]
    params, _ = curve_fit(cubic_curve, x_data, z_data)
    a, b, c, d = params
    z_predicted = cubic_curve(x_data, a, b, c, d)
    distances = np.abs(z_data - z_predicted)
    average_distance = np.mean(distances)
    '''
    plt.figure()
    plt.scatter(x_data, z_data, label="Original Data", color="blue", s=5)
    plt.plot(x_data, z_predicted, label="Fitted Curve", color="red")
    plt.xlabel("X-axis")
    plt.ylabel("Z-axis")
    plt.title("Curve Fitting")
    plt.legend()
    plt.show()
    '''
    return distances <= threshold * average_distance


def segement(line_points, filtered_points):
    colors = np.zeros_like(line_points)
    colors[filtered_points] = [0, 1, 0]
    colors[~filtered_points] = [1, 0, 0]
    return colors

def delete_points(tire_points, line_points):
    return np.array([point for point in tire_points if tuple(point) not in set(tuple(point) for point in line_points)])


#%%
#creating a new point cloud
Segmented_tire = o3d.geometry.PointCloud()


#%%
# Detection of Tire Defects
#Since non of the open3d functions are working, I will use the following code to read the pcd file

# Convert the point cloud to a numpy array
tire_points = np.asarray(tire.points)

#Since the point cloud is generated using Line based 3D scanner, the points are in a form of parallel lines
#Hence we do line wise segmentation by using simple curve Fitting

while len(tire_points) > 0:
    #Selecting the points in line
    line_points = points_selector(tire_points, range=1.5)

    if len(line_points) > 4:
        #Doing Curve Fitting to filtter the points
        filtered_points_mask = filtering(line_points, threshold= 3)

        #Segmenting the rejected points in red color and rest in Green
        line_colors = segement(line_points, filtered_points_mask)

        #Adding the segmented points to the new point cloud
        points = np.asarray(Segmented_tire.points)
        updated_points = np.vstack((points, line_points)) if points.size else line_points
        Segmented_tire.points = o3d.utility.Vector3dVector(updated_points)

        colors = np.asarray(Segmented_tire.colors)
        updated_colors = np.vstack((colors, line_colors)) if colors.size else line_colors
        Segmented_tire.colors = o3d.utility.Vector3dVector(updated_colors)
        pass

    #Removing the points which are already segmented
    tire_points = delete_points(tire_points, line_points)

    print(len(tire_points))
    pass



o3d.visualization.draw_geometries([Segmented_tire])