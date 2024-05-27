import open3d as o3d
import json
import numpy as np
import os

loadingAmaount = 1

def getPCDs(pathToFolders, height, width, intrinsic_mat, extrinsic_mat, num_images=301, ):
    pcds = []
    for image_number in range(num_images):
        color_file_path = os.path.join(pathToFolders, "color", f"{image_number:05d}.jpg")
        depth_file_path = os.path.join(pathToFolders, "depth", f"{image_number:05d}.png")
        print("loaded ", image_number, "/", num_images)
        color_raw = o3d.io.read_image(color_file_path)
        depth_raw = o3d.io.read_image(depth_file_path)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color=color_raw, 
            depth=depth_raw, 
            depth_scale=1000.0,
            depth_trunc=10.0, 
            convert_rgb_to_intensity=False)
        
        pcds.append(createPC(rgbd_image, height, width, intrinsic_mat, extrinsic_mat))
        rgbd_image = None
    return pcds

def getIntrinsics(path):
    json_file_path = path + "intrinsic.json"
    with open(json_file_path, "r") as json_file:
        # Load JSON data into a Python dictionary
        data = json.load(json_file)
    intrinsic_matrix = data["intrinsic_matrix"]

    intrinsic_matrix = np.array([
        [intrinsic_matrix[0], intrinsic_matrix[3], intrinsic_matrix[6]],
        [intrinsic_matrix[1], intrinsic_matrix[4], intrinsic_matrix[7]],
        [intrinsic_matrix[2], intrinsic_matrix[5], intrinsic_matrix[8]]
    ])
    height = data["height"]
    width = data["width"]
    return intrinsic_matrix, height, width

def createPC(rgbd_image, height, width, intrinsic_matrix, extrinsic_matrix):
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(height=height, width=width, intrinsic_matrix=intrinsic_matrix))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform(extrinsic_matrix)
    voxeledDown = pcd.voxel_down_sample(voxel_size=0.01)
    return voxeledDown

def combinePointCloudPerFrame(pcds_M, pcds_S1, pcds_S2): 
    all_pcds_per_Frame = []
    for i in range(len(pcds_M)):
        pcd_in_Frame = pcds_M[i] + pcds_S1[i] + pcds_S2[i]
        all_pcds_per_Frame.append(pcd_in_Frame)
    return all_pcds_per_Frame



intr_M, height_M, width_M = getIntrinsics("")#set path to intrinsics
extr_M = [
    [-0.9939503, -0.03950024, 0.10248183, -0.14961497],
    [0.02270801,  -0.98683595, -0.16012228, 1.85014322],
    [0.10745762,  -0.15682643,  0.98176287, -3.9282198],
    [0, 0, 0, 1],
]
pcds_M = getPCDs("", height_M, width_M, intr_M, extr_M, loadingAmaount)#set path to Pics

intr_S1, height_S1, width_S1 = getIntrinsics("")#set path to intrinsics
extr_S1 = [
    [ 0.56199585, -0.29204414,  0.77386749, -2.56310273],
 [ 0.05134139, -0.92147358, -0.38503312,  2.00513958],
 [ 0.82554512,  0.25611844, -0.50287036,  1.73157791],
 [ 0.        ,  0.        ,  0.        ,  1.        ]
]
pcds_S1 = getPCDs("",height_S1, width_S1, intr_S1, extr_S1, loadingAmaount)#set path to Pics

intr_S2, height_S2, width_S2 = getIntrinsics("")#set path to intrinsics
extr_S2 = [
    [ 0.55271681,  0.31597125, -0.77114609,  3.69106138],
 [-0.00467293, -0.92414592, -0.38201108,  1.97714511],
 [-0.83335604,  0.21474746, -0.50931448,  0.9125606 ],
 [ 0.        ,  0.        ,  0.        ,  1.        ]
]
pcds_S2 = getPCDs("",height_S2, width_S2, intr_S2, extr_S2, loadingAmaount)#set path to Pics

all_pcds_per_frame = combinePointCloudPerFrame(pcds_M, pcds_S1, pcds_S2)
pcds_M = None
pcds_S1 = None
pcds_S2 = None

o3d.visualization.draw_geometries([all_pcds_per_frame[0]])

xyz_coordinates = np.asarray(all_pcds_per_frame[0].points)
colors = np.asarray(all_pcds_per_frame[0].colors)
ones_column = np.ones((xyz_coordinates.shape[0], 1))

# Concatenate x, y, z coordinates, ones, and colors horizontally
init_pcd_stack = np.hstack((xyz_coordinates, colors, ones_column))

init_pcd_np = np.asarray(init_pcd_stack)
np.savez("", init_pcd_np)#TODO: Set .npz name
