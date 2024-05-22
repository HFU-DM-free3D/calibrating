import open3d as o3d
import json
import numpy as np
import os
import time
from typing import List
from CamExtris import CamExtris


class Pcd():
    def __init__(self, points, colors):
        self.points = points
        self.colors = colors
    def to_dict(self):
        return {'points': self.points, 'colors': self.colors}

class PointPos():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    def to_dict(self):
        return {'x': self.x, 'y': self.y, 'z': self.z}

class PointCol():
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b
    def to_dict(self):
        return {'x': self.r, 'y': self.g, 'z': self.b}

class PcdHandler():
    def __init__(self, loading_amount, sub_amount, icp_its, just_show_center, path, sub_path, extris: List[CamExtris], create_pcd_json):
        self.sub_path = sub_path
        self.path = path + self.sub_path
        self.path = self.ensure_trailing_backslash(self.path)
        self.icp_its = icp_its
        self.just_show_center = just_show_center
        
        intr_M, height_M, width_M = self.getIntrinsics(self.path + "M/")
        ppM_pcd = self.postpro_focus_pcd(extris[0].extri, self.path + "M/", height_M, width_M, intr_M)
        pcds_M = self.getPCDs(self.path + "M/", height_M, width_M, intr_M, extris[0].extri, loading_amount)

        all_sub_pcds_perFrame = []
        for sub in range(sub_amount):
            intr_S, height_S, width_S = self.getIntrinsics(self.path + "S" + str(sub + 1) + "/")
            pp_pcd = self.postpro_focus_pcd(extris[sub + 1].extri, self.path + "S" + str(sub + 1) + "/", height_S, width_S, intr_S)
            transform_S = self.icp_algo(pp_pcd, ppM_pcd, self.icp_its)
            new_extri = self.compute_transform(transform_S, extris[sub + 1].extri)
            pcds_perFrame = self.getPCDs(self.path + "S" + str(sub + 1) + "/", height_S, width_S, intr_S, new_extri, loading_amount)
            all_sub_pcds_perFrame.append(pcds_perFrame)

        all_pcds_per_frame = self.combinePointCloudPerFrame(pcds_M, all_sub_pcds_perFrame)
        
        if create_pcd_json:
            self.createJson(all_pcds_per_frame, self.path)
        
        self.visualise(all_pcds_per_frame, loading_amount)

    def ensure_trailing_backslash(self, path):
            if not path.endswith("\\"):
                return path + "\\"
            return path

    def CalibPCD(self, pathToFolder, height, width, intrinsic_mat, extrinsic_mat):
        color_raw = o3d.io.read_image(pathToFolder + "color/00000.jpg")
        depth_raw = o3d.io.read_image(pathToFolder +"depth/00000.png")
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color=color_raw, 
                depth=depth_raw, 
                depth_scale=1000.0,
                depth_trunc=7.0, 
                convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, 
                                                             o3d.camera.PinholeCameraIntrinsic(height=height, width=width, intrinsic_matrix=intrinsic_mat))
        pcd.transform(extrinsic_mat)
        return pcd

    def getPCDs(self, pathToFolders, height, width, intrinsic_mat, extrinsic_mat, num_images=301, ):
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
                depth_trunc=7.0, 
                convert_rgb_to_intensity=False)

            pcds.append(self.createPC(rgbd_image, height, width, intrinsic_mat, extrinsic_mat))
            rgbd_image = None
        return pcds

    def getIntrinsics(self, path):
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

    def just_center(self, cloud, ind):
        inlier_cloud = cloud.select_by_index(ind)
        return inlier_cloud

    def db_Scan(self, pcd):
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
        unique_numbers, counts = np.unique(labels, return_counts=True)
        index_of_most_common = np.argmax(counts)
        most_common_number = unique_numbers[index_of_most_common]
        indices_of_most_common = np.where(labels == most_common_number)[0]
        focus_pcd = self.just_center(pcd, indices_of_most_common)
        return focus_pcd

    def groundless(self, pcd):
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,ransac_n=3, num_iterations=1000)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        return outlier_cloud

    def createPC(self, rgbd_image, height, width, intrinsic_matrix, extrinsic_matrix):
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(height=height, width=width, intrinsic_matrix=intrinsic_matrix))
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform(extrinsic_matrix)
        voxeledDown = pcd
        voxeledDown = pcd.voxel_down_sample(voxel_size=0.01)
        if self.just_show_center == True:
            voxeledDown = self.db_Scan(voxeledDown)
            voxeledDown = self.groundless(voxeledDown)

        return voxeledDown

    def combinePointCloudPerFrame(self, pcds_M, pcds_Subs): 
        all_pcds_per_Frame = []
        for i in range(len(pcds_M)):
            pcd_in_Frame = pcds_M[i]
            for sub in pcds_Subs:
                pcd_in_Frame += sub[i]
            all_pcds_per_Frame.append(pcd_in_Frame)
        return all_pcds_per_Frame

    def icp_algo(self, _source, _target, it):
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

    def compute_transform(self, transformation_matrix1, transformation_matrix2):
        return transformation_matrix1@transformation_matrix2
    
    def postpro_focus_pcd(self, oldextri, path_to_pics, pics_height, pics_width, intri_M):
        postProCloud = self.CalibPCD(path_to_pics,pics_height, pics_width, intri_M, oldextri)
        focus_pcd = self.db_Scan(postProCloud)
        return focus_pcd
    
    def all_new_sub_pcd_transforms(self, focus_pcds_subs, focus_pcd_master):
        all_new_transforms = []
        for focus_pcd_sub in focus_pcds_subs:
            transform = self.icp_algo(focus_pcd_sub, focus_pcd_master, self.icp_its)
            all_new_transforms.append(transform)
        return all_new_transforms
    
    def createJson(self, all_pcds_per_frame, path):
        clouds = []
        for pcd in all_pcds_per_frame:
            points_list = np.round(np.asarray(pcd.points),5).tolist()
            points_list_class = [PointPos(x, y, z) for x, y, z in points_list]
            points_dict_list = [point.to_dict() for point in points_list_class]
            colors_list = np.round(np.asarray(pcd.colors),5).tolist()
            colors_list_class = [PointPos(x, y, z) for x, y, z in colors_list]
            colors_dict_list = [point.to_dict() for point in colors_list_class]
            newPcd: Pcd = Pcd(points_dict_list, colors_dict_list)
            clouds.append(newPcd)
            newPcd = None

        all_Clouds_dict = {"clouds": [cloud.to_dict() for cloud in clouds]}

        j = json.dumps(all_Clouds_dict)
        file_path = path + "bigOneClouds.json"
        print("started to create pcd json, pls wait, this needs maybe minutes...or less...maybe stop if it takes more time than 10 minutes :D")
        with open(file_path, 'w') as f:
            f.write(j)

    def visualise(self, all_pcds_per_frame, loading_amount):
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

        frame_rate = 30
        frame_duration = 1.0 / frame_rate
        start_time = time.time()

        while(True):
            current_time = time.time()
            elapsed_time = current_time - start_time

            if elapsed_time >= frame_duration:
                counter = counter +1
                if counter >= loading_amount: 
                    counter = 0

                geometry.points = all_pcds_per_frame[counter].points
                geometry.colors = all_pcds_per_frame[counter].colors
                vis.update_geometry(geometry)
                start_time = time.time()

            if not vis.poll_events():
                break
            vis.update_renderer()

        vis.destroy_window()
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)