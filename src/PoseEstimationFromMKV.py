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

import matplotlib

detectedMarkerIDs = []
allPositions = []

#inputVideo = cv2.VideoCapture("E:/Studium/Master/Semester_1/ForschProj/testVids/t2711/sub2.mkv")
#inputVideo = cv2.VideoCapture("C:/Users/Lukas/OneDrive/Desktop/testVids2711/output-4.mkv")
inputVideo = cv2.VideoCapture("E:/Studium/Master/Semester_1/ForschProj/testVids/t0412/size20.mkv")
#inputVideo = cv2.VideoCapture("E:/Studium/Master/Semester_1/ForschProj/testVids/t0412/depth6cm/depth.mp4")
json_file_path = "E:/Studium/Master/Semester_1/ForschProj/testVids/t0412/size20.json"

if os.path.exists(json_file_path):
    with open(json_file_path, "r") as json_file:
        calibration_data = json.load(json_file)
    
    # Extract camera parameters from calibration data
    camera_parameters = calibration_data["CalibrationInformation"]["Cameras"][1]["Intrinsics"]["ModelParameters"]
    intrinsic_camera = np.array(camera_parameters[:9]).reshape((3, 3))
    distortion = np.array(camera_parameters[9:])
    print(intrinsic_camera)
    print(distortion)


# Read Params
cameraMatrix = np.zeros((3, 3), dtype=np.float32)
distCoeffs = np.zeros((5, 1), dtype=np.float32)

# Markerwidth
markerLength = 0.20

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

    # if one Marcer is tracked
    if ids is not None and len(ids) > 0:
        aruco.drawDetectedMarkers(imageCopy, corners)
        nMarkers = len(corners)
        
        rvecs, tvecs = [], []
        # calculate Pose of marker
        for i in range(nMarkers):
            _, rvec, tvec = cv2.solvePnP(objPoints, corners[i], intrinsic_camera, distortion)
            rvecs.append(rvec)
            tvecs.append(tvec)

            for id in ids:
                if id not in detectedMarkerIDs:
                    detectedMarkerIDs.append(id)
                    print(str(detectedMarkerIDs[-1]), ": ", str(tvec))
                    if id == 99: allPositions.append(tvec)
            

        # # Draw axis (doesnt work somehow)
        # for i in range(len(ids)):
        #     aruco.drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)

    # Ergebnisbild anzeigen und Fenster schließen
    cv2.imshow("out", imageCopy)

    key = cv2.waitKey(1) & 0xFF
    # if key == ord("p"):
    #     print(tvecs)
    if key == ord("q"):
        break

inputVideo.release()
cv2.destroyAllWindows()

viewer = Viewer()
for pos in allPositions:
    viewer.add_point(pos)

viewer.show()