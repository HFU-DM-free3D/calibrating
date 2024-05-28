#Many code Fragements from here: https://github.com/opencv/opencv/blob/4.x/samples/python/aruco_detect_board_charuco.py 06.05.2024

import numpy as np
import cv2 as cv
import sys
import os
import json
import math


def get_cMtx_and_dist(path):
    if os.path.exists(path):
        with open(path, "r") as json_file:
            calibration_data = json.load(json_file)
    
    # Extract camera parameters from calibration data
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


def poseEstimation():
    square_len = 0.150
    marker_len = 0.112

    cam_param_path = ""
    cam_matrix, dist_coefficients = get_cMtx_and_dist(cam_param_path)

    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    board_size = (3, 5)
    board = cv.aruco.CharucoBoard(board_size, square_len, marker_len, aruco_dict)
    charuco_detector = cv.aruco.CharucoDetector(board)

    input_video = cv.VideoCapture("")
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
            cv.aruco.drawDetectedMarkers(image_copy, marker_corners)
        
        if not (charuco_ids is None) and len(charuco_ids) > 0:
            cv.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
            if len(cam_matrix) > 0 and len(charuco_ids) >= 4:
                try:
                    obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
                    flag, rvec, tvec = cv.solvePnP(obj_points, img_points, cam_matrix, dist_coefficients)
                    if flag:
                        cv.drawFrameAxes(image_copy, cam_matrix, dist_coefficients, rvec, tvec, .2)
                        #print("tvec: ",tvec)
                        R, _ = cv.Rodrigues(rvec)
                        all_tvecs.append(tvec)
                        all_R.append(R)
                except cv.error as error_inst:
                    print("SolvePnP recognize calibration pattern as non-planar pattern. To process this need to use "
                          "minimum 6 points. The planar pattern may be mistaken for non-planar if the pattern is "
                          "deformed or incorrect camera parameters are used.")
                    print(error_inst.err)
        cv.imshow("out", image_copy)
        key = cv.waitKey(wait_time)
        if key == 27:
            break
        image = input_video.retrieve()[1] if input_video is not None and input_video.grab() else None
    return all_tvecs, all_R

def trans_In_Board_Coord(t, R):
    R_Cam = np.transpose(R)
    t = -(R_Cam@t)
    return t, R_Cam

def computeOriginDistance(trans1, trans2):
        x1 = trans1[0]
        y1 = trans1[1]
        z1 = trans1[2]
        x2 = trans2[0]
        y2 = trans2[1]
        z2 = trans2[2]
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance

def giveShortestDistanceFromMedian(array, median):
        candidate = None
        shortest_dist = float('inf')
        index = -1

        for i, trans in enumerate(array):
            dist = computeOriginDistance(trans, median)
            if dist < shortest_dist:
                shortest_dist = dist
                candidate = trans
                index = i
        return candidate, index



if __name__ == "__main__":
    all_tvecs, all_R = poseEstimation()
    medianTrans = np.median(all_tvecs, axis=0)
    closestToMedian, indexOfClosest = giveShortestDistanceFromMedian(all_tvecs, medianTrans)
    t, R = trans_In_Board_Coord(all_tvecs[indexOfClosest], all_R[indexOfClosest])
    print(t, "\n", R)