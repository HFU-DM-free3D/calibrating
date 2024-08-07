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

#Master Marker 22
R_22 = np.array([
     [-0.017760, 0.997304, -0.071203],
    [0.970572, 0.034301, 0.238355],
    [0.240155, -0.064875, -0.968564]])

transVec_22 = np.array([0.220527, 1.174703, 3.972793])

#Sub 1 Marker 30
R_30 = np.array([
    [ 0.83293438,  0.00215087, -0.55336759],
    [ 0.20313343, -0.93136955,  0.30213833],
    [-0.51473987, -0.36406886, -0.77620663]])
transVec_30 = np.array([-0.16808066, 0.61675334, 3.59506188])

#Sub 2 Marker 15
R_15 = np.array([
    [ 0.0581231,  -0.82034033,  0.56891428],
    [-0.91646056,  0.18214016,  0.35626536],
    [-0.39588098, -0.54209474, -0.741223]])
transVec_15 = np.array([-1.25329658, 0.46878802, 4.08229681])

###### Transform Marker in OpenCV Cam in Unity Cam --> Works "Marker to Cam"
#OpenCV Kamera Coord in Unity Coord
OCV_in_U = np.array([[1,  0,  0],
 [0,  -1, 0],
 [0, 0, 1]])
R_in_Unity_Cam = OCV_in_U @ R_22
T_in_Unity_Cam = OCV_in_U @ transVec_22

print("Rotate in Unity Cam: ", R_in_Unity_Cam, "\n transform in unity Cam: ", T_in_Unity_Cam)

###### Transform Camera in Unity Marker -->Works "One new Scene"
#OpenCV Coord in Unity Coord
OCV_M_in_U = np.array([
    [-1,  0,  0],
    [0,  1, 0],
    [0, 0, 1]])
RC_in_Marker = np.transpose(R_in_Unity_Cam)
TC_in_Marker = -RC_in_Marker@T_in_Unity_Cam

RC_in_Unity_Marker = OCV_M_in_U @ RC_in_Marker
TC_in_Unity_Marker = OCV_M_in_U @ TC_in_Marker
print("\nRotate in Unity Marker: ", RC_in_Unity_Marker, "\n transform in unity Marker: ", TC_in_Unity_Marker)

###### Transform Camera in unity Cube
cubeInUnityRot = np.array([
    [-1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
 ])

#Rotation Computation: CubeRotationInUnityRotation @ MarkerRotationInCubeRotation @ CamRotationInMarkerRotation --> doesnt work yet

def fromMarkerInUnityCubeSystem(R, trans, MarkerIdIndex):
    RC_in_Cube = all_Markers[MarkerIdIndex].originRotation @ R
    TC_in_Cube = all_Markers[MarkerIdIndex].originTranslation + (all_Markers[MarkerIdIndex].originRotation @ trans)
    return RC_in_Cube, TC_in_Cube

#1 id 0, 15 id 1, 22 id 2, 30 id 3
RC_in_Unity_Cube, TC_in_Unity_Cube = fromMarkerInUnityCubeSystem(RC_in_Marker, TC_in_Marker, 2)
print("\nRotate in Unity Cube: ", RC_in_Unity_Cube, "\n transform in unity Cube: ", TC_in_Unity_Cube)