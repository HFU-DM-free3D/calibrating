import open3d as o3d
import numpy as np

rotate_M = [
    [0,0,1,],
    [0,1,0,],
    [1,0,0]
]

translate_M = [0,1,-1.5]

extr_M = [
            [-0.7978780605626172, 0.14343101498472147, -0.5855067415609227,1.4524725886782508],
            [-0.6006560566977163, -0.10695817522112872, 0.7923208001218426,-1.031932787235833],
            [0.05101864389795707, 0.9838635539005495, 0.1714922892756251,-0.8499110533759379],
            [0.0,0.0,0.0,1.0]
            ]

extr_M_my = [
            [-0.7978780605626172, -0.14343101498472147, -0.5855067415609227, 1.4524725886782508],
            [-0.6006560566977163, 0.10695817522112872, 0.7923208001218426, -1.031932787235833],
            [0.05101864389795707, -0.9838635539005495, 0.1714922892756251, -0.8499110533759379],
            [0.0,0.0,0.0,1.0]
            ]

m1_trans = np.array([1.4524725886782508, -1.031932787235833, -0.8499110533759379], dtype=np.float64).reshape(3, 1)

extr_S1 = [
            
            [
                0.9159078993102356,0.10644898901561323,-0.38701593341704293,1.0851447521409463
            ],
            [
                -0.401359175326322,0.25453561245944567,-0.879842278122204,2.0170475428105332
            ],
            [
                0.0048510166445684955,0.961186888553651,0.27585545655300237,-0.8346253454888046
            ],
            [
                0.0,0.0, 0.0,1.0
            ]
        ]

extr_S1_my = [
            
            [
                0.9159078993102356,-0.10644898901561323,-0.38701593341704293,1.0851447521409463
            ],
            [
                -0.401359175326322,-0.25453561245944567,-0.879842278122204,-2.0170475428105332
            ],
            [
                0.0048510166445684955,-0.961186888553651,0.27585545655300237,-0.8346253454888046
            ],
            [
                0.0,0.0, 0.0,1.0
            ]
        ]
extr_S2 = [
            [
            -0.0770303761099208,-0.2305086002290312,0.9700165495375924,-1.5575662407868105
        ],
        [
            0.9950195914555074,0.04396024234336676,0.08946233683974136,0.4453272470008728
        ],
        [
            -0.06326400063289606,0.972076788280414,0.2259742992252785,-0.8376607161497036
        ],
        [
            0.0,0.0,0.0,1.0
        ]
        ]

extr_S2_my = [
            [
            -0.0770303761099208,0.2305086002290312,0.9700165495375924,-1.5575662407868105
        ],
        [
            0.9950195914555074,-0.04396024234336676,0.08946233683974136,-0.4453272470008728
        ],
        [
            -0.06326400063289606,-0.972076788280414,0.2259742992252785,-0.8376607161497036
        ],
        [
            0.0,0.0,0.0,1.0
        ]
        ]
mesh_frame_m = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4)
mesh_frame_m.transform(extr_M)

mesh_frame_s1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4)
mesh_frame_s1.transform(extr_S1)

mesh_frame_s2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4)
mesh_frame_s2.transform(extr_S2)

center = np.array([0, 0, 0], dtype=np.float64).reshape(3, 1)
#glb to obj converter: https://convert3d.org/glb-to-obj 13.06
mesh = o3d.io.read_triangle_mesh("C:/Users/Lukas/Downloads/scene (4)/scene (4).obj")
mesh.transform(extr_M_my)
mesh.scale(scale=17, center=m1_trans)
#mesh.rotate(rotate_M)
#mesh.translate(translate_M)

path_to_npz = "F:/Studium/Master/Semester_3/Forsch/Recordings/a2_aufteilung/standing1/aruco/simon1/npzs/ATLFB_1.npz"
pcd_data = np.load(path_to_npz)["data"]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd_data[:, :3])
pcd.colors = o3d.utility.Vector3dVector(pcd_data[:, 3:6])
pcd.rotate(rotate_M)

# Visualize the mesh
o3d.visualization.draw_geometries([mesh, mesh_frame_m, mesh_frame_s1, mesh_frame_s2, pcd])