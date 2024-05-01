import open3d as o3d
import json
import numpy as np

def getDepthImage(pathToFolders):
    color_raw = o3d.io.read_image(pathToFolders + "color/00005.jpg")
    depth_raw = o3d.io.read_image(pathToFolders +"depth/00005.png")
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color = color_raw, depth=depth_raw, depth_scale=3000.0)
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
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform(extrinsic_matrix)
    return pcd

sf = 9

rgbd_M = getDepthImage("")
intr_M, height_M, width_M = getIntrinsics("")
extr_M_o = [
    [0.9939503, -0.03950024, -0.10248183, 0.14961497],
    [0.02270801,  0.98683595, -0.16012228, 1.85014322],
    [0.10745762,  0.15682643,  0.98176287, -3.9282198],
    [0, 0, 0, 1],
]
#From Aruco
extr_M = [
    [-0.9939503, -0.03950024, 0.10248183, -0.14961497/3],
    [0.02270801,  -0.98683595, -0.16012228, 1.85014322/3],
    [0.10745762,  -0.15682643,  0.98176287, -3.9282198/3],
    [0, 0, 0, 1],
]
npMo = np.array(extr_M_o)
npM = np.array(extr_M)
print(npM @ npMo)

#From Dust3r
# extr_M = [[0.6576125 ,  -0.06669493, 0.75039834,  -0.05340749*sf],
#  [0.3033833 ,  -0.8882889 ,  -0.34482086, 0.06798072*sf],
#  [ 0.68956834 , 0.4544168 , -0.5639158,   0.19328779*sf],
#  [ 0.     ,     0.        ,  0.    ,      1.        ]]
pcd_M = createPC(rgbd_M, height_M, width_M, intr_M, extr_M)

rgbd_S1 = getDepthImage("")
intr_S1, height_S1, width_S1 = getIntrinsics("")
# extr_S1 = [
#     [-0.54274409, -0.28253118, -0.79095195, 2.7321261],
#     [0.01431065,  0.9384762,  -0.34504727, 1.87120594],
#     [0.8397762,  -0.1985914,  -0.5053092, 1.88317298],
#     [0,0,0,1]
# ]
#From Aruco
extr_S1 = [
    [0.54274409, -0.28253118, 0.79095195, -2.7321261/3],
    [0.01431065,  -0.9384762,  -0.34504727, 1.87120594/3],
    [0.8397762,  0.1985914,  -0.5053092, 1.88317298/3],
    [0,0,0,1]
]

#From Dust3r
# extr_S1 = [[0.3643276 , 0.36904764 , -0.855026 ,  0.1553486*sf ],
#  [ -0.2872489 ,  -0.8288284  , -0.48013735, 0.06304204*sf],
#  [-0.8858633 ,  0.42053252, -0.19595653 , 0.15701546*sf],
#  [ 0.       ,   0.       ,   0.      ,    1.        ]]
pcd_S1 = createPC(rgbd_S1, height_S1, width_S1, intr_S1, extr_S1)

rgbd_S2 = getDepthImage("")
intr_S2, height_S2, width_S2 = getIntrinsics("")
# extr_S2 = [
#     [-0.565007,    0.32458066,  0.7585608,-3.75002756],
#     [0.0478601,   0.93071658, -0.36259626,1.98196877],
#     [-0.82369685, -0.16856463, -0.54139585,1.10257667],
#     [0,0,0,1]
# ]

#From Aruco
extr_S2 = [
    [0.565007,    0.32458066,  -0.7585608, 3.75002756/3],
    [0.0478601,   -0.93071658, -0.36259626, 1.98196877/3],
    [-0.82369685, 0.16856463, -0.54139585, 1.10257667/3],
    [0,0,0,1]
]

#From Dust3r
# extr_S2 = [[ -0.9999763,  0.00577111,  -0.00374598,  0.,        ],
#  [ -0.0057603 ,  -0.9999792 ,  -0.00289049 , 0.        ],
#  [-0.00376258, -0.00286884 , 0.9999888 ,  0.        ],
#  [ 0.         , 0.         , 0.        ,  1.        ]]
pcd_S2 = createPC(rgbd_S2, height_S2, width_S2, intr_S2, extr_S2)

mesh_frame_mtest = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.4)
mesh_frame_mtest.transform(extr_M)

mesh_frame_S1 = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.4)
mesh_frame_S1.transform(extr_S1)

mesh_frame_S2 = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.4)
mesh_frame_S2.transform(extr_S2)
o3d.visualization.draw_geometries([pcd_M, pcd_S1, pcd_S2, mesh_frame_mtest, mesh_frame_S2, mesh_frame_S1])