import open3d as o3d
import json
import numpy as np
import os
import time

loadingAmaount = 20
ShowJustCenter = False

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
            depth_scale=3000.0, #depth_scale=1000.0
            depth_trunc=5.0, 
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

def just_center(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    return inlier_cloud

def db_Scan(pcd):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
    unique_numbers, counts = np.unique(labels, return_counts=True)
    index_of_most_common = np.argmax(counts)
    most_common_number = unique_numbers[index_of_most_common]
    indices_of_most_common = np.where(labels == most_common_number)[0]
    focus_pcd = just_center(pcd, indices_of_most_common)
    return focus_pcd

def groundless(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,ransac_n=3, num_iterations=1000)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return outlier_cloud

def createPC(rgbd_image, height, width, intrinsic_matrix, extrinsic_matrix):
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(height=height, width=width, intrinsic_matrix=intrinsic_matrix))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform(extrinsic_matrix)
    voxeledDown = pcd
    voxeledDown = pcd.voxel_down_sample(voxel_size=0.01)
    if ShowJustCenter == True:
        voxeledDown = db_Scan(voxeledDown)
        voxeledDown = groundless(voxeledDown)
    
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
# extr_S1 = [
#     [0.54274409, -0.28253118, 0.79095195, -2.7321261/3],
#     [0.01431065,  -0.9384762,  -0.34504727, 1.87120594/3],
#     [0.8397762,  0.1985914,  -0.5053092, 1.88317298/3],
#     [0,0,0,1]
# ]
# extr_S1 = [
#     [ 0.54285377, -0.28281817,  0.79077254, -2.65024317/3],
#  [ 0.01499845, -0.93816215, -0.34582666,  1.86244479/3],
#  [ 0.83968532,  0.19959156, -0.50503787,  1.82038517/3],
#  [ 0.        ,  0.        ,  0.        ,  1.        ],
# ]
extr_S1 = [
    [ 0.56199585, -0.29204414,  0.77386749, -2.56310273/3],
 [ 0.05134139, -0.92147358, -0.38503312,  2.00513958/3],
 [ 0.82554512,  0.25611844, -0.50287036,  1.73157791/3],
 [ 0.        ,  0.        ,  0.        ,  1.        ]
]
pcds_S1 = getPCDs("",height_S1, width_S1, intr_S1, extr_S1, loadingAmaount)

intr_S2, height_S2, width_S2 = getIntrinsics("")
# extr_S2 = [
#     [0.565007,    0.32458066,  -0.7585608, 3.75002756/3],
#     [0.0478601,   -0.93071658, -0.36259626, 1.98196877/3],
#     [-0.82369685, 0.16856463, -0.54139585, 1.10257667/3],
#     [0,0,0,1]
# ]
# extr_S2 = [
#     [ 0.56487919,  0.32439232, -0.75873528,  3.71676524/3],
#  [ 0.04687117, -0.93060586 ,-0.36297643,  1.96869398/3],
#  [-0.82382832,  0.16947774 ,-0.54088989,  1.07754278/3],
#  [ 0.        ,  0.         , 0.        ,  1.        ],
# ]
extr_S2 = [
    [ 0.55271681,  0.31597125, -0.77114609,  3.69106138/3],
 [-0.00467293, -0.92414592, -0.38201108,  1.97714511/3],
 [-0.83335604,  0.21474746, -0.50931448,  0.9125606 /3],
 [ 0.        ,  0.        ,  0.        ,  1.        ]
]
pcds_S2 = getPCDs("",height_S2, width_S2, intr_S2, extr_S2, loadingAmaount)

o3d.io.write_point_cloud("main.pcd", pcds_M[0])
o3d.io.write_point_cloud("s1.pcd", pcds_S1[0])
o3d.io.write_point_cloud("s2.pcd", pcds_S2[0])

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

o3d.io.write_point_cloud("combine.pcd", all_pcds_per_frame[0])

frame_rate = 30
frame_duration = 1.0 / frame_rate

counter = 0
start_time = time.time()

while(True):
    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time >= frame_duration:
        counter = counter +1
        if counter >= loadingAmaount: 
            counter = 0
        print("frame: ", counter)

        geometry.points = all_pcds_per_frame[counter].points
        geometry.colors = all_pcds_per_frame[counter].colors
        vis.update_geometry(geometry)
        start_time = time.time()
    
    if not vis.poll_events():
        break
    vis.update_renderer()

vis.destroy_window()
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)    