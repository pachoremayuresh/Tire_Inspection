import open3d as o3d
import numpy as np


pcd = o3d.io.read_point_cloud("Tire_3D_Images/Ideal_Image.pcd")
pcd2 = o3d.io.read_point_cloud("Tire_3D_Images/negative_1.pcd")
o3d.visualization.draw_geometries([pcd2])

