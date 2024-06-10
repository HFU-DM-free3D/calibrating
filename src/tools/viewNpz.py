import open3d as o3d
import numpy as np

path_to_npz = ""
pcd_data = np.load(path_to_npz)["data"]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd_data[:, :3])
pcd.colors = o3d.utility.Vector3dVector(pcd_data[:, 3:6])


o3d.visualization.draw_geometries([pcd])