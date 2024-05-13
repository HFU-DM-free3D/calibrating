import open3d as o3d
import json
import numpy as np
import matplotlib.pyplot as plt
import copy

def getDepthImage(pathToFolders):
    color_raw = o3d.io.read_image(pathToFolders + "color/00180.jpg")
    depth_raw = o3d.io.read_image(pathToFolders +"depth/00180.png")
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color = color_raw, depth=depth_raw, depth_scale=1000.0, depth_trunc=7.0)
    return rgbd_image

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
    pcd.transform(extrinsic_matrix)
    return pcd


rgbd_M = getDepthImage("")
intr_M, height_M, width_M = getIntrinsics("")

extr_M = [
    [-0.9939503, -0.03950024, 0.10248183, -0.14961497],
    [0.02270801,  -0.98683595, -0.16012228, 1.85014322],
    [0.10745762,  -0.15682643,  0.98176287, -3.9282198],
    [0, 0, 0, 1],
]

pcd_M = createPC(rgbd_M, height_M, width_M, intr_M, extr_M)
pcd_M.voxel_down_sample(voxel_size=0.01)

rgbd_S1 = getDepthImage("")
intr_S1, height_S1, width_S1 = getIntrinsics("")

extr_S1 = [
    [0.54274409, -0.28253118, 0.79095195, -2.7321261],
    [0.01431065,  -0.9384762,  -0.34504727, 1.87120594],
    [0.8397762,  0.1985914,  -0.5053092, 1.88317298],
    [0,0,0,1]
]
pcd_S1 = createPC(rgbd_S1, height_S1, width_S1, intr_S1, extr_S1)
pcd_S1.voxel_down_sample(voxel_size=0.01)

rgbd_S2 = getDepthImage("")
intr_S2, height_S2, width_S2 = getIntrinsics("")

extr_S2 = [
    [0.565007,    0.32458066,  -0.7585608, 3.75002756],
    [0.0478601,   -0.93071658, -0.36259626, 1.98196877],
    [-0.82369685, 0.16856463, -0.54139585, 1.10257667],
    [0,0,0,1]
]

pcd_S2 = createPC(rgbd_S2, height_S2, width_S2, intr_S2, extr_S2)
pcd_S2.voxel_down_sample(voxel_size=0.01)

o3d.visualization.draw_geometries([pcd_M, pcd_S1, pcd_S2])

## Clustering
def just_center(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    return inlier_cloud

def db_Scan(pcd):
    pcd_db = copy.deepcopy(pcd)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(pcd_db.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
    max_label = labels.max()

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd_db.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd_db])
    unique_numbers, counts = np.unique(labels, return_counts=True)
    index_of_most_common = np.argmax(counts)
    most_common_number = unique_numbers[index_of_most_common]
    indices_of_most_common = np.where(labels == most_common_number)[0]
    focus_pcd = just_center(pcd, indices_of_most_common)
    return focus_pcd

focus_pcd_M = db_Scan(pcd_M)
focus_pcd_S1 = db_Scan(pcd_S1)
focus_pcd_S2 = db_Scan(pcd_S2)

o3d.visualization.draw_geometries([focus_pcd_M, focus_pcd_S1, focus_pcd_S2])

## 4. Boden entfernen
def groundless(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,ransac_n=3, num_iterations=1000)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    return outlier_cloud

groundless_pcd_M = groundless(focus_pcd_M)
groundless_pcd_S1 = groundless(focus_pcd_S1)
groundless_pcd_S2 = groundless(focus_pcd_S2)

def icp_algo(_source, _target, it):
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    source = _source
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30), fast_normal_computation=True)
    target = _target
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30), fast_normal_computation=True)
    threshold = 0.07
    reg_p2l = o3d.pipelines.registration.registration_icp(
       source, target, threshold, np.identity(4),
       o3d.pipelines.registration.TransformationEstimationPointToPoint(),
       o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=it))
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
    return reg_p2l.transformation

# amountIterations = 20

# transform_S1 = icp_algo(groundless_pcd_S1, groundless_pcd_M, amountIterations)
# pcd_S1.transform(transform_S1)
# groundless_pcd_S1.transform(transform_S1)

# transform_S2 = icp_algo(groundless_pcd_S2, groundless_pcd_M, amountIterations)
# pcd_S2.transform(transform_S2)
# groundless_pcd_S2.transform(transform_S2)
# o3d.visualization.draw_geometries([groundless_pcd_M, groundless_pcd_S1, groundless_pcd_S2])
# o3d.visualization.draw_geometries([pcd_M, pcd_S1, pcd_S2])

amountIterations = 50

transform_S1 = icp_algo(focus_pcd_S1, focus_pcd_M, amountIterations)
pcd_S1.transform(transform_S1)
groundless_pcd_S1.transform(transform_S1)

transform_S2 = icp_algo(focus_pcd_S2, focus_pcd_M, amountIterations)
pcd_S2.transform(transform_S2)
groundless_pcd_S2.transform(transform_S2)
o3d.visualization.draw_geometries([groundless_pcd_M, groundless_pcd_S1, groundless_pcd_S2])
o3d.visualization.draw_geometries([pcd_M, pcd_S1, pcd_S2])

def print_transform(transformation_matrix1, transformation_matrix2, it):
    trans_mat1 = np.array(transformation_matrix1)
    trans_mat2 = np.array(transformation_matrix2)
    
    # Extrahieren des Rotations- und Translationsanteils der Matrizen
    rotation1 = trans_mat1[:3, :3]
    translation1 = trans_mat1[:3, 3]
    rotation2 = trans_mat2[:3, :3]
    translation2 = trans_mat2[:3, 3]
    
    # Multiplikation der Rotationsanteile
    combined_rotation = np.dot(rotation1, rotation2)
    
    # Addition der Translationsteile
    combined_translation = translation1 + translation2
    
    # Erstellen der kombinierten Transformationsmatrix
    combined_transform = np.eye(4)
    combined_transform[:3, :3] = combined_rotation
    combined_transform[:3, 3] = combined_translation
    
    return transformation_matrix1@transformation_matrix2

changed_S1 =   print_transform(transform_S1, extr_S1, amountIterations)  
print(changed_S1)

changed_S2 =   print_transform(transform_S2, extr_S2, amountIterations) 
print(changed_S2)