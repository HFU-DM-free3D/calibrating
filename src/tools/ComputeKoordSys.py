import numpy as np

class MarkerOnCube:
    def __init__(self,id, rotation, translation) -> None:
        self.id = id
        self.originRotation = rotation
        self.originTranslation = translation

def markersOnCubeInUnity():
    marker_1_Rotation = np.array( [
        [0,1,0],
        [1,0,0],
        [0,0,1]
    ])
    marker_1_Translation = np.array([0,0,0.5*0.17])
    marker_1 = MarkerOnCube(id=1, rotation=marker_1_Rotation, translation=marker_1_Translation)
    marker_15_Rotation = np.array( [
        [0,0,-1],
        [1,0,0],
        [0,1,0]
    ])
    marker_15_Translation = np.array([-0.5*0.17,0,0])
    marker_15 = MarkerOnCube(id=15, rotation=marker_15_Rotation, translation=marker_15_Translation)
    marker_22_Rotation = np.array( [
        [0,1,0],
        [-1,0,0],
        [0,0,-1]
    ])
    marker_22_Translation = np.array([0,0,-0.5*0.17])
    marker_22 = MarkerOnCube(id=22, rotation=marker_22_Rotation, translation=marker_22_Translation)
    
    marker_30_Rotation = np.array( [
        [0,0,1],
        [0,1,0],
        [1,0,0]
    ])
    marker_30_Translation = np.array([0.5*0.17,0,0])
    marker_30 = MarkerOnCube(id=30, rotation=marker_30_Rotation, translation=marker_30_Translation)

    marker_435_Rotation = np.array( [
        [0,1,0],
        [0,0,1],
        [-1,0,0]
    ])
    marker_435_Translation = np.array([0,0.5*0.17,0])
    marker_435 = MarkerOnCube(id=435, rotation=marker_435_Rotation, translation=marker_435_Translation)
    allMarkers = [marker_1, marker_15, marker_22, marker_30, marker_435]
    return allMarkers

all_Markers = markersOnCubeInUnity()

### Tracked Markers from OCV Cam
#Master Marker 22
R_22 = np.array([
     [-0.017760, 0.997304, -0.071203],
    [0.970572, 0.034301, 0.238355],
    [0.240155, -0.064875, -0.968564]])

transVec_22 = np.array([0.220527, 1.174703, 3.972793])

#Main RGB Transform from Kinect
R_RGB_m = np.array([
    [0.99999445676803589, 0.0032223907765001059, -0.0008539882255718112], 
    [-0.0031210724264383316, 0.995002269744873, 0.099803544580936432], 
    [0.0011713262647390366, -0.099800325930118561, 0.995006799697876]
])
t_RGB_m = np.array([-0.032150775194168091, -0.0018429256742820144, 0.0038166081067174673])

#Sub 1 Marker 30
R_30 = np.array([
    [ 0.83293438,  0.00215087, -0.55336759],
    [ 0.20313343, -0.93136955,  0.30213833],
    [-0.51473987, -0.36406886, -0.77620663]])
transVec_30 = np.array([-0.16808066, 0.61675334, 3.59506188])

#Sub 1 RGB Transform from Kinect
R_RGB_s1 = np.array([
    [0.99999713897705078, 0.0019759682472795248, -0.0013582889223471284], 
    [-0.0018450334900990129, 0.99590045213699341, 0.0904369130730629], 
    [0.0015314210904762149, -0.090434141457080841, 0.99590128660202026]
])
t_RGB_s1 = np.array([-0.031990431249141693, -0.0020193634554743767, 0.00399469630792737])

#Sub 2 Marker 15
R_15 = np.array([
    [ 0.0581231,  -0.82034033,  0.56891428],
    [-0.91646056,  0.18214016,  0.35626536],
    [-0.39588098, -0.54209474, -0.741223]])
transVec_15 = np.array([-1.25329658, 0.46878802, 4.08229681])

#Sub 2 RGB Transform from Kinect
R_RGB_s2 = np.array([
    [0.99999088048934937, 0.003501951927319169, -0.0024544622283428907], 
    [-0.003232145681977272, 0.99473541975021362, 0.10242555290460587], 
    [0.00280022993683815, -0.10241668671369553, 0.99473762512207031]
])
t_RGB_s2 = np.array([-0.032119434326887131, -0.001887562801130116, 0.003847010200843215])

###### Transform Marker in OpenCV Cam in Unity Cam
def fromOCVCamInUnityCam(rot, trans):
    OCV_in_U = np.array([[1,  0,  0],
     [0,  -1, 0],
     [0, 0, 1]])
    R_in_Unity_Cam = OCV_in_U @ rot
    T_in_Unity_Cam = OCV_in_U @ trans
    #print("Rotate in Unity Cam: ", R_in_Unity_Cam, "\n transform in unity Cam: ", T_in_Unity_Cam)
    return R_in_Unity_Cam, T_in_Unity_Cam

###### Transform Camera in Unity Marker
#OpenCV Coord in Unity Coord
def fromCamInMarkerKoord(rot, trans):
    OCV_M_in_U = np.array([
        [-1,  0,  0],
        [0,  1, 0],
        [0, 0, 1]])
    RC_in_Marker = np.transpose(rot)
    TC_in_Marker = -RC_in_Marker@trans
    #print("\nRotate in Unity Marker: ", RC_in_Unity_Marker, "\n transform in unity Marker: ", TC_in_Unity_Marker)
    return RC_in_Marker, TC_in_Marker

#Rotation Computation: CubeRotationInUnityRotation @ MarkerRotationInCubeRotation @ CamRotationInMarkerRotation
def fromMarkerInUnityCubeSystem(R, trans, MarkerIdIndex):
    RC_in_Cube = all_Markers[MarkerIdIndex].originRotation @ R
    TC_in_Cube = all_Markers[MarkerIdIndex].originTranslation + (all_Markers[MarkerIdIndex].originRotation @ trans)
    return RC_in_Cube, TC_in_Cube

#Compute Rot and Trans of Depth Camera Depending on unity RGB Cam
def computeDepthCam(R_RGB_Unity_Cam, R_RGB_Kinect, trans_RGB_Cam, trans_RGB_Kinect):
    R_Depth_in_Unity = R_RGB_Unity_Cam @ np.transpose(R_RGB_Kinect)
    trans_Depth_in_Unity = trans_RGB_Cam + R_Depth_in_Unity @ trans_RGB_Kinect
    return R_Depth_in_Unity, trans_Depth_in_Unity

#1 id 0, 15 id 1, 22 id 2, 30 id 3
def main():
    #Main Cam
    rot_Unity_Cam_Main, trans_Unity_Cam_Main = fromOCVCamInUnityCam(R_22, transVec_22)
    rot_Marker_Main, trans_Marker_Main = fromCamInMarkerKoord(rot_Unity_Cam_Main, trans_Unity_Cam_Main)
    rot_Unity_Main, trans_Unity_Main = fromMarkerInUnityCubeSystem(rot_Marker_Main, trans_Marker_Main, 2)
    print("Main Cam: ")
    print("Rotate in Unity Cube: ", rot_Unity_Main, "\n transform in unity Cube: ", trans_Unity_Main)
    rot_Unity_Cam_Main_Depth, trans_Unity_Cam_Main_Depth = computeDepthCam(rot_Unity_Main, R_RGB_m, trans_Unity_Main, t_RGB_m)
    print("Depth Rot: ", rot_Unity_Cam_Main_Depth, "\n trans: ", trans_Unity_Cam_Main_Depth)

    #Sub 1
    rot_Unity_Cam_S1, trans_Unity_Cam_S1 = fromOCVCamInUnityCam(R_30, transVec_30)
    rot_Marker_S1, trans_Marker_S1 = fromCamInMarkerKoord(rot_Unity_Cam_S1, trans_Unity_Cam_S1)
    rot_Unity_S1, trans_Unity_S1 = fromMarkerInUnityCubeSystem(rot_Marker_S1, trans_Marker_S1, 3)
    print("\nS1: ")
    print("Rotate in Unity Cube: ", rot_Unity_S1, "\n transform in unity Cube: ", trans_Unity_S1)

    #Sub 2
    rot_Unity_Cam_S2, trans_Unity_Cam_S2 = fromOCVCamInUnityCam(R_15, transVec_15)
    rot_Marker_S2, trans_Marker_S2 = fromCamInMarkerKoord(rot_Unity_Cam_S2, trans_Unity_Cam_S2)
    rot_Unity_S2, trans_Unity_S2 = fromMarkerInUnityCubeSystem(rot_Marker_S2, trans_Marker_S2, 1)
    print("\nS2: ")
    print("Orientation: ", rot_Unity_S2, "\n Translation: ", trans_Unity_S2)

main()