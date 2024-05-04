import open3d as o3d
import json
import numpy as np
import os

loadingAmaount = 200

def getPCDs(pathToFolders, height, width, intrinsic_mat, extrinsic_mat, num_images=301, ):
    pcds = []
    for image_number in range(num_images):
        color_file_path = os.path.join(pathToFolders, "color", f"{image_number:05d}.jpg")
        depth_file_path = os.path.join(pathToFolders, "depth", f"{image_number:05d}.png")
        print("loaded ", image_number, "/", num_images)
        color_raw = o3d.io.read_image(color_file_path)
        depth_raw = o3d.io.read_image(depth_file_path)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color=color_raw, depth=depth_raw, depth_scale=3000.0)
        
        pcds.append(createPC(rgbd_image, height, width, intrinsic_mat, extrinsic_mat))
        rgbd_image = None
    return pcds

def getIntrinsics(path):
    json_file_path = path + "intrinsic.json"
    with open(json_file_path, "r") as json_file:
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
    pcd.transform(extrinsic_matrix)
    voxeledDown = pcd.voxel_down_sample(voxel_size=0.01)
    return voxeledDown

def combinePointCloudPerFrame(pcds_M, pcds_S1, pcds_S2): 
    all_pcds_per_Frame = []
    for i in range(len(pcds_M)):
        pcd_in_Frame = pcds_M[i] + pcds_S1[i] + pcds_S2[i]
        all_pcds_per_Frame.append(pcd_in_Frame)
    return all_pcds_per_Frame

intr_M, height_M, width_M = getIntrinsics("")
extr_M = [
    [-0.9939503, -0.03950024, 0.10248183, -0.14961497/3],
    [0.02270801,  -0.98683595, -0.16012228, 1.85014322/3],
    [0.10745762,  -0.15682643,  0.98176287, -3.9282198/3],
    [0, 0, 0, 1],
]
pcds_M = getPCDs("", height_M, width_M, intr_M, extr_M, loadingAmaount)

intr_S1, height_S1, width_S1 = getIntrinsics("")
extr_S1 = [
    [0.54274409, -0.28253118, 0.79095195, -2.7321261/3],
    [0.01431065,  -0.9384762,  -0.34504727, 1.87120594/3],
    [0.8397762,  0.1985914,  -0.5053092, 1.88317298/3],
    [0,0,0,1]
]
pcds_S1 = getPCDs("",height_S1, width_S1, intr_S1, extr_S1, loadingAmaount)

intr_S2, height_S2, width_S2 = getIntrinsics("")
extr_S2 = [
    [0.565007,    0.32458066,  -0.7585608, 3.75002756/3],
    [0.0478601,   -0.93071658, -0.36259626, 1.98196877/3],
    [-0.82369685, 0.16856463, -0.54139585, 1.10257667/3],
    [0,0,0,1]
]
pcds_S2 = getPCDs("",height_S2, width_S2, intr_S2, extr_S2, loadingAmaount)

all_pcds_per_frame = combinePointCloudPerFrame(pcds_M, pcds_S1, pcds_S2)
pcds_M = None
pcds_S1 = None
pcds_S2 = None

o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
vis = o3d.visualization.Visualizer()
vis.create_window()

render_options = vis.get_render_option()
render_options.point_size = 3
render_options.light_on = False

counter = 0

geometry = o3d.geometry.PointCloud()
geometry.points = all_pcds_per_frame[counter].points
geometry.colors = all_pcds_per_frame[counter].colors
vis.add_geometry(geometry)

while(True):
    counter = counter +1
    if counter >= loadingAmaount: 
        counter = 0
    print("frame: ", counter)

    geometry.points = all_pcds_per_frame[counter].points
    geometry.colors = all_pcds_per_frame[counter].colors
    vis.update_geometry(geometry)
    
    if not vis.poll_events():
        break
    vis.update_renderer()

vis.destroy_window()
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)    