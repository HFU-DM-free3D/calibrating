import numpy as np
import cv2
import cv2.aruco as aruco

#TODO: Extract all needed tracks from that one mkv (using MKVToolNix or ffmpeg)
# Needed Tracks:
# Track 1: ffmpeg -i input.mkv -map 0:v:0 output.mp4 --> Cam 1, Color Track (i don't know yet if we have to save an output.mp4 or we can catch the frames directly)
# Track 2: ffmpeg -i input.mkv -map 0:v:1 output.mp4 --> Cam 1, Depth Track
# Track 4: ffmpeg -i input.mkv -map 0:v:3 output.mp4 --> Cam 2, Color Track
# Track 5: ffmpeg -i input.mkv -map 0:v:4 output.mp4 --> Cam 2, Depth Track
# Track 7: ffmpeg -i input.mkv -map 0:v:6 output.mp4 --> Cam 3, Color Track
# Track 8: ffmpeg -i input.mkv -map 0:v:7 output.mp4 --> Cam 3, Depth Track

#TODO: Extract all needed attatchments from that mkv
#Needed Attatchments:
# Attatchment 1: ffmpeg -dump_attachment:3 calibration1.json -i output-4.mkv (i don't know yet how the attatchments behave if you maerge them in an mkv so the numbers are more random here, but the first should be 3...)
# Attatchment 2: ffmpeg -dump_attachment:3 calibration2.json -i output-4.mkv
# Attatchment 3: ffmpeg -dump_attachment:3 calibration3.json -i output-4.mkv

# Marker Length should be 15 cm if the createt Marker Cube is used...if not change this here
markerLength = 0.15

# TODO: Think about if we need new Object points. This creates an Object for one marker (a plane) maybe we generate the full cube with the sizes of the cube and always 3 planes have to be tracked to track this object
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

#TODO: Think about a smart way to create the json where the position and rotation of the cameras to the Marker is saved.
# Maybe the Marker could be the tvec(0,0,0) and rvec(0,0,0) so the computet vectors like in "PoseEstimationFromMKV" are the inverse
# Or maybe sth else.

#TODO: Save the vectors in a json