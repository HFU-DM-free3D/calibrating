#This Scripts computes marker Positions out of 3 mkv Files and 3 calibration.json files
#change paths in line 164, 165, 178, 179, 192, 193

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os

class TrackedMarker:
    def __init__(self, frame, id, tvec_inv, R_inv):
        self.frame = float(frame)  # Ensure frame is of type double
        self.id = int(id)         # Ensure id is an integer
        self.tvec_inv = np.array(tvec_inv, dtype=float).reshape(3, 1)  # 3x1 matrix
        self.R_inv = np.array(R_inv, dtype=float).reshape(3, 3)        # 3x3 matrix

def getCameraParameters(json_file_path):
    if os.path.exists(json_file_path):
        with open(json_file_path, "r") as json_file:
            calibration_data = json.load(json_file)

        # Extract camera parameters from calibration data
        camera_parameters = calibration_data["CalibrationInformation"]["Cameras"][1]["Intrinsics"]["ModelParameters"]
        return camera_parameters

def createCameraIntrinsicMat(camera_parameters):
    # intrinsic_camera = np.array([
    #     [camera_parameters[3]*1920, 0, camera_parameters[0]*1920],
    #     [0, camera_parameters[2]*1080, camera_parameters[1]*1080],
    #     [0, 0, 1]
    # ])
    intrinsic_camera = np.array([
        [613.0575342,    0.,         660.64849309],
        [  0.,         610.61722469, 364.54884815],
        [  0.,           0.,           1.,        ],
    ])
    return intrinsic_camera

def createCameraDistortionParams(camera_parameters):
    # distortion = np.array(
    #     [camera_parameters[4],
    #     camera_parameters[5],
    #     camera_parameters[12],
    #     camera_parameters[13],
    #     camera_parameters[6],
    #     camera_parameters[7],
    #     camera_parameters[8],
    #     camera_parameters[9]]
    #     )
    distortion = np.array(
        [0.09586925,
        -0.05745232,
        -0.00254828,
        0.01196302,
        -0.00026813]
        )
    return distortion

def detectMarkerInVideo(markerLength, intrinsic_camera, distortion_params, inputVideo):

    detectedMarkerIDs = []
    allTrackedMarkers = []

    # update the Marker Size in Meter. To compute the correkt Position you need to tell the right size of the Marker. If you set this to small the distance will be longer than in reality
    # if its too big the measured Distance will be shorter than in reality
    #https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html#details struktur intrinsics calibrierung
    #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera 8koeffizienten für opencv solve PNP

    # Koordsystem
    objPoints = np.zeros((4, 1, 3), dtype=np.float32)
    objPoints[0] = np.array([[-markerLength/2, markerLength/2, 0]])
    objPoints[1] = np.array([[markerLength/2, markerLength/2, 0]])
    objPoints[2] = np.array([[markerLength/2, -markerLength/2, 0]])
    objPoints[3] = np.array([[-markerLength/2, -markerLength/2, 0]])

    # Initialisiere den Aruco-Detektor
    detectorParams = aruco.DetectorParameters()
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    detector = aruco.ArucoDetector(dictionary, detectorParams)

    allTrackedMarkers = []


    while inputVideo.isOpened():
        ret, image = inputVideo.read()
        if not ret:
            break

        imageCopy = image.copy()
        frameNumber = inputVideo.get(cv2.CAP_PROP_POS_FRAMES)

        # detect Marker in Pic
        corners, ids, rejected_img_points = detector.detectMarkers(image)

        if ids is None or len(ids) == 0:
            detectedMarkerIDs.clear()

        # if one Marker is tracked
        if ids is not None and len(ids) > 0:

            aruco.drawDetectedMarkers(imageCopy, corners)
            nMarkers = len(corners)

            rvecs, tvecs = [], []

            # calculate Pose of marker
            for i in range(nMarkers):
                _, rvec, tvec = cv2.solvePnP(objPoints, corners[i], intrinsic_camera, distortion_params)
                #if id == 22: print(tvec)
                # start from: https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp 24.03.24
                # Convert rvec to rotation matrix R
                R, _ = cv2.Rodrigues(rvec)

                #print("Rotation Mat:", R, "transVec: " , tvec)
                # project 3D points to image plane
                cv2.drawFrameAxes(imageCopy, intrinsic_camera, distortion_params, rvec, tvec, 0.15) 

                # Invert the rotation matrix
                R_inv = np.transpose(R)
                #print("R: ", R, "\nR_inv:", R_inv)

                # Invert the translation vector
                tvec_inv = -np.dot(R_inv, tvec)

                #[frame][id][tvec_inv][R_inv]
                allTrackedMarkers.append(TrackedMarker(frameNumber, ids[i],tvec_inv, R_inv))


        # Show Video in window
        cv2.imshow("out", imageCopy)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    inputVideo.release()
    cv2.destroyAllWindows()
    return allTrackedMarkers

def getDataforSpecificMarker(marker_id, all_tracked_markers):
    markerArray = [marker for marker in all_tracked_markers if marker.id == marker_id]
    tvec_inv_list = [marker.tvec_inv for marker in markerArray]
    medianTvec_inv = np.median(tvec_inv_list, axis=0)
    fitting_pos = [pos for pos in markerArray if np.allclose(medianTvec_inv, pos.tvec_inv)]
    R_inv_list = [marker.R_inv for marker in markerArray]
    median_R_inv = np.median(R_inv_list, axis = 0)
    fitting_Rot = [rot for rot in fitting_pos if np.allclose(median_R_inv, rot.R_inv)]
    if len(fitting_Rot) > 0:
        return fitting_Rot[0]
    if len(fitting_pos) > 0: return fitting_pos[0]
    else:
        print("no fitting median for pos at id: ", marker_id)
        return markerArray[0]

def getDataForAllMarkers(all_tracked_markers):
    all_tracked_markers = [marker for marker in all_tracked_markers if marker.id in (1, 15, 22, 30, 99, 435)]
    unique_ids = list({marker.id for marker in all_tracked_markers})
    data_all_Markers = []
    for i in range(len(unique_ids)):
        data_all_Markers.append(getDataforSpecificMarker(unique_ids[i], all_tracked_markers))
    return data_all_Markers

def main():
    markerLength = 0.15
    inputVideo = cv2.VideoCapture("")
    json_file_path = ""
    camera_parameters_main = getCameraParameters(json_file_path)
    intrinsic_matrix_main = createCameraIntrinsicMat(camera_parameters_main)
    distotrion_params_main = createCameraDistortionParams(camera_parameters_main)
    all_markers_main = detectMarkerInVideo(markerLength, intrinsic_matrix_main, distotrion_params_main, inputVideo) 
    marker_data_main = []
    marker_data_main = getDataForAllMarkers(all_markers_main)
    print("Results from main-cam: ")
    for i in range(len(marker_data_main)):
        print("\nid: ", marker_data_main[i].id, "\ntvec_inv: ", marker_data_main[i].tvec_inv, "\nR_inv: ", marker_data_main[i].R_inv)
    print("first done")
    print("")

    video_Sub1 = cv2.VideoCapture("")
    json_Sub1 = ""
    camera_parameters_s1 = getCameraParameters(json_Sub1)
    intrinsic_matrix_s1 = createCameraIntrinsicMat(camera_parameters_s1)
    distotrion_params_s1 = createCameraDistortionParams(camera_parameters_s1)
    all_markers_s1 = detectMarkerInVideo(markerLength, intrinsic_matrix_s1, distotrion_params_s1, video_Sub1) 
    marker_data_s1 = []
    marker_data_s1 = getDataForAllMarkers(all_markers_s1)
    print("Results from sub1: ")
    for i in range(len(marker_data_s1)):
        print("id: ", marker_data_s1[i].id, "\ntvec_inv: ", marker_data_s1[i].tvec_inv, "\nR_inv: ", marker_data_s1[i].R_inv)
    print("second done")
    print("")

    video_Sub2 = cv2.VideoCapture("")
    json_Sub2 = ""
    camera_parameters_s2 = getCameraParameters(json_Sub2)
    intrinsic_matrix_s2 = createCameraIntrinsicMat(camera_parameters_s2)
    distotrion_params_s2 = createCameraDistortionParams(camera_parameters_s2)
    all_markers_s2 = detectMarkerInVideo(markerLength, intrinsic_matrix_s2, distotrion_params_s2, video_Sub2) 
    marker_data_s2 = []
    marker_data_s2 = getDataForAllMarkers(all_markers_s2)
    print("Results from sub2: ")
    for i in range(len(marker_data_s2)):
        print("id: ", marker_data_s2[i].id, "\ntvec_inv: ", marker_data_s2[i].tvec_inv, "\nR_inv: ", marker_data_s2[i].R_inv)
    print("third done")

main()