import cv2
import cv2.aruco as aruco
import os
import sys
import math
import numpy as np
import json
from MarkerFilterManager import MarkerFilterManager, TrackedMarker


class ExtrinsicCalculator:
    def __init__(self, path, marker_length, cam_role, use_Charuco, ch_marker_len, ch_square_len, ch_dict, ch_board_size):
        self.path = path
        self.marker_length = marker_length
        self.cam_role = cam_role
        self.transformation_matrix = np.eye(4,4)
        self.inputVideo = cv2.VideoCapture(self.path + "calib" + self.cam_role + ".mkv")
        self.all_tracked_markers = []
        if self.intri_Json_exists():
            intri_Mtx, dist = self.get_dist_and_intri_mtx()
            if use_Charuco:
                
                ch_all_tvecs, ch_all_R = self.charuco_marker_Tracking(intri_MTx=intri_Mtx, 
                    dist=dist, 
                    ch_marker_len=ch_marker_len, 
                    ch_square_len=ch_square_len, 
                    ch_dict=ch_dict, 
                    ch_board_size=ch_board_size)
                
                medianTrans = np.median(ch_all_tvecs, axis=0)
                closestToMedian, indexOfClosest = self.ch_giveShortestDistanceFromMedian(ch_all_tvecs, medianTrans)
                t, R = self.ch_trans_In_Board_Coord(ch_all_tvecs[indexOfClosest], ch_all_R[indexOfClosest])
                self.transformation_matrix = self.ch_4x4_tMat(t, R)
            else:
                self.marker_Tracking(intri_Mtx, dist)
                mfm = MarkerFilterManager(self.all_tracked_markers)
                self.transformation_matrix = mfm.get_o3d_extrinsic_matrix()
        else:
            print("no intri Json found") 
    
    def get_extrinsic_matrix(self):
        return self.transformation_matrix

    def intri_Json_exists(self):
        if os.path.exists(self.path + "intri" + self.cam_role + ".json"):
            return True
        else: return False

    def get_dist_and_intri_mtx(self):
        json_file_path = self.path + "intri" + self.cam_role + ".json"
        if os.path.exists(json_file_path):
            with open(json_file_path, "r") as json_file:
                calibration_data = json.load(json_file)
            camera_parameters = calibration_data["CalibrationInformation"]["Cameras"][1]["Intrinsics"]["ModelParameters"]
            intrinsic_camera = np.array([
                [camera_parameters[2]*1920, 0, camera_parameters[0]*1920],
                [0, camera_parameters[3]*(1080*(4/3)), camera_parameters[1]*1080],
                [0, 0, 1]
            ])
            distortion = np.array(
                [   camera_parameters[4],
                    camera_parameters[5],
                    camera_parameters[13],
                    camera_parameters[12],
                    camera_parameters[6],
                    camera_parameters[7],
                    camera_parameters[8],
                    camera_parameters[9]
                ])
        return intrinsic_camera, distortion
    
    aruco_dict_switch = {
        0: cv2.aruco.DICT_6X6_250,
    }

    def get_aruco_dict(self, value):
        return self.aruco_dict_switch.get(value, "Invalid key")

    def marker_Tracking(self, intri_Mtx, dist):    
        objPoints = np.zeros((4, 1, 3), dtype=np.float32)
        objPoints[0] = np.array([[-self.marker_length/2, self.marker_length/2, 0]])
        objPoints[1] = np.array([[self.marker_length/2, self.marker_length/2, 0]])
        objPoints[2] = np.array([[self.marker_length/2, -self.marker_length/2, 0]])
        objPoints[3] = np.array([[-self.marker_length/2, -self.marker_length/2, 0]])

        # Initialisiere den Aruco-Detektor
        detectorParams = aruco.DetectorParameters()
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        detector = aruco.ArucoDetector(dictionary, detectorParams)
        rvecs, tvecs = [], []
        while self.inputVideo.isOpened():
            ret, image = self.inputVideo.read()

            if not ret:
                break
            imageCopy = image.copy()

            # detect Marcer in Pic
            corners, ids, rejected_img_points = detector.detectMarkers(image)

            # if one Marker is tracked
            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(imageCopy, corners)
                nMarkers = len(corners)

                rvecs, tvecs = [], []
                # calculate Pose of marker
                for i in range(nMarkers):
                    _, rvec, tvec = cv2.solvePnP(objPoints, corners[i], intri_Mtx, dist)
                    rvecs.append(rvec)
                    tvecs.append(tvec)
                    R, _ = cv2.Rodrigues(rvec)

                    if ids[i] in [1, 15, 22, 30, 99, 435]:self.all_tracked_markers.append(TrackedMarker(ids[i], R, tvec))
                    cv2.drawFrameAxes(imageCopy, intri_Mtx, dist, rvec, tvec, self.marker_length) 

            # Show Video in window
            cv2.imshow("out", imageCopy)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        self.inputVideo.release()
        cv2.destroyAllWindows()

    def charuco_marker_Tracking(self, intri_MTx, dist, ch_marker_len, ch_square_len, ch_dict=0, ch_board_size=(3,5)):
        square_len = ch_square_len
        marker_len = ch_marker_len

        cam_matrix, dist_coefficients = intri_MTx, dist
        aruco_dict = self.get_aruco_dict(ch_dict)
        aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        board_size = ch_board_size
        print(board_size)
        print(square_len)
        print(marker_len)
        print(aruco_dict)
        board = cv2.aruco.CharucoBoard(board_size, square_len, marker_len, aruco_dict)
        charuco_detector = cv2.aruco.CharucoDetector(board)

        input_video = self.inputVideo
        image = input_video.retrieve()[1] if input_video.grab() else None

        wait_time = 10
        if image is None:
            print("Error: unable to open video/image source")
            sys.exit(0)

        all_tvecs = []
        all_R = []

        while image is not None:
            image_copy = np.copy(image)
            charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(image)
            if not (marker_ids is None) and len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(image_copy, marker_corners)

            if not (charuco_ids is None) and len(charuco_ids) > 0:
                cv2.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
                if len(cam_matrix) > 0 and len(charuco_ids) >= 4:
                    try:
                        obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
                        flag, rvec, tvec = cv2.solvePnP(obj_points, img_points, cam_matrix, dist_coefficients)
                        if flag:
                            cv2.drawFrameAxes(image_copy, cam_matrix, dist_coefficients, rvec, tvec, .2)
                            #print("tvec: ",tvec)
                            R, _ = cv2.Rodrigues(rvec)
                            all_tvecs.append(tvec)
                            all_R.append(R)
                    except cv2.error as error_inst:
                        print("SolvePnP recognize calibration pattern as non-planar pattern. To process this need to use "
                              "minimum 6 points. The planar pattern may be mistaken for non-planar if the pattern is "
                              "deformed or incorrect camera parameters are used.")
                        print(error_inst.err)
            cv2.imshow("out", image_copy)
            key = cv2.waitKey(wait_time)
            if key == 27:
                break
            image = input_video.retrieve()[1] if input_video is not None and input_video.grab() else None
        self.inputVideo.release()
        cv2.destroyAllWindows()
        return all_tvecs, all_R

    def ch_trans_In_Board_Coord(self, t, R):
        R_Cam = np.transpose(R)
        t = -(R_Cam@t)
        return t, R_Cam

    def ch_computeOriginDistance(self, trans1, trans2):
            x1 = trans1[0]
            y1 = trans1[1]
            z1 = trans1[2]
            x2 = trans2[0]
            y2 = trans2[1]
            z2 = trans2[2]
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            return distance

    def ch_giveShortestDistanceFromMedian(self, array, median):
            candidate = None
            shortest_dist = float('inf')
            index = -1

            for i, trans in enumerate(array):
                dist = self.ch_computeOriginDistance(trans, median)
                if dist < shortest_dist:
                    shortest_dist = dist
                    candidate = trans
                    index = i
            return candidate, index
    
    def ch_4x4_tMat(self, t, R):
        transformation_matrix = np.eye(4,4)
        translation_vector = t.flatten()
        transformation_matrix[0:3, 0:3] = R
        transformation_matrix[0:3, 3] = translation_vector
        return transformation_matrix