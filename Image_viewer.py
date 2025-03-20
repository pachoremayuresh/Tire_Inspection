
# Importing the required libraries

import open3d as o3d
import numpy as np



# Reading the Images

Tire = o3d.io.read_point_cloud("/home/mayuresh/Desktop/Image_Processing/Tire_Inspection/Tire_3D_Images/Positive.pcd")
print(Tire)







# Downsampling the Images

# Voxel downsampling voxel size test
'''
voxel_sizes = [0.05, 0.1,0.2, 0.3, 0.4]
for voxel_size in voxel_sizes:
    downpcd_tire = tire.voxel_down_sample(voxel_size=voxel_size)
    print(f"Downsampled point cloud size with voxel size {voxel_size}: {len(downpcd_tire.points)}")
    o3d.visualization.draw_geometries([downpcd_tire], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], 
                                      lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])

# There was a significant downsampling only after 0.2 voxel size since the point cloud image was very dense


voxel_sizes = [0.9, 1.0, 1.5, 2.0, 2.5, 3.0]
for voxel_size in voxel_sizes:
    downpcd_tire = tire.voxel_down_sample(voxel_size=voxel_size)
    print(f"Downsampled point cloud size with voxel size {voxel_size}: {len(downpcd_tire.points)}")
    o3d.visualization.draw_geometries([downpcd_tire], zoom=0.3412, front=[0.4257, -0.2125, -0.8795], 
                                      lookat=[2.6172, 2.0475, 1.532], up=[-0.0694, -0.9768, 0.2024])

# After Testing the voxel sizes in increasing order it was observed that after voxel size 2.0 the point cloud image was lossing significant Data.
# Hence the optimal voxel size for downsampling the image could be anything between 1.5 and 2.0
# Since we are trying to find the smallest possible defect the lover limit would retain more data hence we choice voxel size = 1.5
# The optimal voxel size for downsampling the image is 1.5

'''

Tire = Tire.voxel_down_sample(voxel_size=0.2)
print(Tire)
















#Visualizing the Images

o3d.visualization.draw_geometries([Tire])
