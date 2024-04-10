# same as CamToMarker.py, but just with one camera

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d
import numpy as np

class Viewer:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.points = []
        self.lines = []
        self.polygons = []
    
    def add_point(self, point):
        self.points.append(point)
    
    def add_line(self, start_point, end_point):
        self.lines.append((start_point, end_point))
    
    def add_polygon(self, points):
        self.polygons.append(points)
    
    def render(self):
        for point in self.points:
            self.ax.scatter(*point)
        
        for line in self.lines:
            line_segments = art3d.Line3DCollection([line])
            self.ax.add_collection(line_segments)
        
        for polygon in self.polygons:
            polygon_collection = art3d.Poly3DCollection([polygon], linewidths=1, edgecolors='k', alpha=0.3)
            polygon_collection.set_facecolor('b')
            self.ax.add_collection3d(polygon_collection)
    
    def show(self):
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_box_aspect([2, 2, 2])
        self.ax.set_xlim(-3, 5)  # Anpassung des x-Bildausschnitts
        self.ax.set_ylim(-3, 5)  # Anpassung des y-Bildausschnitts
        self.ax.set_zlim(-3, 5)  # Anpassung des z-Bildausschnitts
        self.render()
        plt.show()

import cv2
import cv2.aruco as aruco
import numpy as np
import json
import os
import glob

detectedMarkerIDs = []
allPositions = []
axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)


# Add the fitting video Path with the videos with the markers
# Add the fitting json Path from the video (you can extract the json from the video with the command: ffmpeg -dump_attachment:3 calibration2.json -i output-4.mkv)

inputVideo = cv2.VideoCapture("")
json_file_path = ""

# update the Marker Size in Meter. To compute the correkt Position you need to tell the right size of the Marker. If you set this to small the distance will be longer than in reality
# if its too big the measured Distance will be shorter than in reality
markerLength = 0.15

if os.path.exists(json_file_path):
    with open(json_file_path, "r") as json_file:
        calibration_data = json.load(json_file)
    
    # Extract camera parameters from calibration data
    camera_parameters = calibration_data["CalibrationInformation"]["Cameras"][1]["Intrinsics"]["ModelParameters"]
    
    # intrinsic_camera = np.array([
    #     [camera_parameters[3]*1920, 0, camera_parameters[0]*1920],
    #     [0, camera_parameters[2]*1080, camera_parameters[1]*1080],
    #     [0, 0, 1]
    # ])
    intrinsic_camera = np.array([
        [613.0575342,    0.,         660.64849309],
        [  0.,         610.61722469, 364.54884815],
        [  0.,           0.,           1.,        ]
    ])
    
    #https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html#details struktur intrinsics calibrierung
    #https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera 8koeffizienten für opencv solve PNP
    #
    # distortion = np.array(
    #     [camera_parameters[4],
    #      camera_parameters[5],
    #      camera_parameters[12],
    #      camera_parameters[13],
    #      camera_parameters[6],
    #      camera_parameters[7],
    #      camera_parameters[8],
    #      camera_parameters[9]]
    #      )
    distortion = np.array(
        [0.09586925,
        -0.05745232,
        -0.00254828,
        0.01196302,
        -0.00026813]
        )
    
    # intrinsic_camera = np.array(camera_parameters[:9]).reshape((3, 3))
    # distortion = np.array(camera_parameters[9:])
    # print(intrinsic_camera)
    # print(distortion)

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
            if id == 22: print(tvec)
            # start from: https://stackoverflow.com/questions/18637494/camera-position-in-world-coordinate-from-cvsolvepnp 24.03.24
            # Convert rvec to rotation matrix R
            R, _ = cv2.Rodrigues(rvec)

            print("Rotation Mat:", R, "transVec: " , tvec)
             # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvec, tvec, intrinsic_camera, distortion)
            #print(imgpts)
            #img = draw(imageCopy,corners[i],imgpts)
            cv2.drawFrameAxes(imageCopy, intrinsic_camera, distortion, rvec, tvec, 0.15) 

            # Invert the rotation matrix
            #R_inv = np.transpose(R)

            # Invert the translation vector
            #tvec_inv = -np.dot(R_inv, tvec)

            # Construct the transformation matrix T
            T = np.eye(4, dtype=R.dtype)
            #T[:3, :3] = R_inv
            #T[:3, 3] = tvec_inv.flatten()
            #end
            
            #rvec_inv, _ = cv2.Rodrigues(R_inv)
            #print(str("trans:"), tvec_inv, str("rot:"), rvec_inv)

            for id in ids:
                if id not in detectedMarkerIDs:
                    detectedMarkerIDs.append(id)
                    print(str(detectedMarkerIDs[-1]), ": ", str(tvec))
                    if id == 99: allPositions.append(tvec)

    # Show Video in window
    cv2.imshow("out", imageCopy)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

inputVideo.release()
cv2.destroyAllWindows()

#Visualises the tracked Marker Positions in an Plot
# viewer = Viewer()
# for pos in allPositions:
#     viewer.add_point(pos)

# viewer.show()