# same as CamToMarker.py, but just with one camera
import numpy as np
from MarkerFilterManager import MarkerFilterManager, TrackedMarker

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os

detectedMarkerIDs = []
allPositions = []
axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)

all_Tracked_Markers = []
path_to_videos = ""


# Add the fitting video Path with the videos with the markers
# Add the fitting json Path from the video (you can extract the json from the video with the command: ffmpeg -dump_attachment:3 calibration2.json -i output-4.mkv)

inputVideo = cv2.VideoCapture(path_to_videos + "calibM.mkv")
json_file_path = path_to_videos + "intriM.json"

# update the Marker Size in Meter. To compute the correkt Position you need to tell the right size of the Marker. If you set this to small the distance will be longer than in reality
# if its too big the measured Distance will be shorter than in reality
markerLength = 0.15

if os.path.exists(json_file_path):
    with open(json_file_path, "r") as json_file:
        calibration_data = json.load(json_file)
    
    # Extract camera parameters from calibration data
    camera_parameters = calibration_data["CalibrationInformation"]["Cameras"][1]["Intrinsics"]["ModelParameters"]
    
    intrinsic_camera = np.array([
        [camera_parameters[2]*1920, 0, camera_parameters[0]*1920],
        [0, camera_parameters[3]*(1080*(4/3)), camera_parameters[1]*1080],
        [0, 0, 1]
    ])
    print(camera_parameters)
    print("intrinsic:", intrinsic_camera)
    
    #https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html#details struktur intrinsics calibrierung
    #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera 8koeffizienten für opencv solve PNP
    #
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
    print("distortion coeffs: ",distortion)

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
rvecs, tvecs = [], []

while inputVideo.isOpened():
    ret, image = inputVideo.read()
    
    if not ret:
        break
    imageCopy = image.copy()

    # detect Marcer in Pic
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
            _, rvec, tvec = cv2.solvePnP(objPoints, corners[i], intrinsic_camera, distortion)
            rvecs.append(rvec)
            tvecs.append(tvec)
            # start from: https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp 24.03.24
            # Convert rvec to rotation matrix R
            R, _ = cv2.Rodrigues(rvec)
            
            R_t = np.transpose(R)
            tvec_t = -R_t @ tvec

            print("Rotation transposed: ", R_t)
            print("\nPosition cam: ", tvec_t)
            
            if ids[i] in [1, 15, 22, 30, 435]:all_Tracked_Markers.append(TrackedMarker(ids[i], R, tvec))
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvec, tvec, intrinsic_camera, distortion)
            cv2.drawFrameAxes(imageCopy, intrinsic_camera, distortion, rvec, tvec, 0.15) 

    # Show Video in window
    cv2.imshow("out", imageCopy)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

inputVideo.release()
cv2.destroyAllWindows()