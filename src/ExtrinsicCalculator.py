import cv2
import cv2.aruco as aruco
import os
import numpy as np
import json
from MarkerFilterManager import MarkerFilterManager, TrackedMarker


class ExtrinsicCalculator:
    def __init__(self, path, marker_length, cam_role):
        self.path = path
        self.marker_length = marker_length
        self.cam_role = cam_role
        self.transformation_matrix = np.eye(4,4)
        self.inputVideo = cv2.VideoCapture(self.path + "calib" + self.cam_role + ".mkv")
        self.all_tracked_markers = []
        if self.intri_Json_exists():
            intri_Mtx, dist = self.get_dist_and_intri_mtx()
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

                    if ids[i] in [1, 15, 22, 30, 435]:self.all_tracked_markers.append(TrackedMarker(ids[i], R, tvec))
                    cv2.drawFrameAxes(imageCopy, intri_Mtx, dist, rvec, tvec, self.marker_length) 

            # Show Video in window
            cv2.imshow("out", imageCopy)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        self.inputVideo.release()
        cv2.destroyAllWindows()