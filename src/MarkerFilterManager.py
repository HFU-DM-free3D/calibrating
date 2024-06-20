import numpy as np
import math
from typing import List, Dict

class TrackedMarker:
    def __init__(self, id=None, rotation=None, translation=None) -> None:
        self.id: int = id
        self.rotation = rotation
        self.translation = translation
        self.depCamInCubeRot = np.array([])
        self.depCamInCubeTrans = np.array([])

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
    marker_1_Translation = np.array([
        [0],
        [0],
       [ 0.5*0.17]])
    marker_1 = MarkerOnCube(id=1, rotation=marker_1_Rotation, translation=marker_1_Translation)
    marker_15_Rotation = np.array( [
        [0,0,-1],
        [1,0,0],
        [0,1,0]
    ])
    marker_15_Translation = np.array([
        [-0.5*0.17],
        [0],
        [0]])
    marker_15 = MarkerOnCube(id=15, rotation=marker_15_Rotation, translation=marker_15_Translation)
    marker_22_Rotation = np.array( [
        [0,1,0],
        [-1,0,0],
        [0,0,-1]
    ])
    marker_22_Translation = np.array([
        [0],
        [0],
        [-0.5*0.17]])
    marker_22 = MarkerOnCube(id=22, rotation=marker_22_Rotation, translation=marker_22_Translation)
    
    marker_30_Rotation = np.array( [
        [0,0,1],
        [0,1,0],
        [1,0,0]
    ])
    marker_30_Translation = np.array([
        [0.5*0.17],
        [0],
        [0]])
    marker_30 = MarkerOnCube(id=30, rotation=marker_30_Rotation, translation=marker_30_Translation)

    marker_99_Rotation = np.array( [
        [0,1,0],
        [0,0,-1],
        [1,0,0]
    ])
    marker_99_Translation = np.array([
        [0],
        [-0.5*0.17],
        [0]])
    marker_99 = MarkerOnCube(id=99, rotation=marker_99_Rotation, translation=marker_99_Translation)

    marker_435_Rotation = np.array( [
        [0,1,0],
        [0,0,1],
        [-1,0,0]
    ])
    marker_435_Translation = np.array([
        [0],
        [0.5*0.17],
        [0]])
    marker_435 = MarkerOnCube(id=435, rotation=marker_435_Rotation, translation=marker_435_Translation)
    allMarkers = [marker_1, marker_15, marker_22, marker_30, marker_99, marker_435]
    return allMarkers

all_Markers: MarkerOnCube = markersOnCubeInUnity()

class MarkerFilterManager:
    def __init__(self, all_Tracked_Markers: List[TrackedMarker]) -> None:
        self.all_Tracked_Markers = all_Tracked_Markers
        sorted_Markers: Dict[str, List[TrackedMarker]] = {
            "id_1_markers" : [marker for marker in self.all_Tracked_Markers if marker.id == 1],
            "id_15_markers" : [marker for marker in self.all_Tracked_Markers if marker.id == 15],
            "id_22_markers" : [marker for marker in self.all_Tracked_Markers if marker.id == 22],
            "id_30_markers" : [marker for marker in self.all_Tracked_Markers if marker.id == 30],
            "id_99_markers" : [marker for marker in self.all_Tracked_Markers if marker.id == 99],
            "id_435_markers" : [marker for marker in self.all_Tracked_Markers if marker.id == 435]
        }

        for key, markerArray in sorted_Markers.items():
            markerArray = self.filteredMarkers(markerArray)
            self.safeAllDependetCamValues(markerArray)
        
        count_tracked_markers = self.countAmountMarkersWhereTracked(sorted_Markers)
        
        self.closestToMedian: TrackedMarker
        if count_tracked_markers >= 2:
            self.candidate_1_15_1, self.candidate_1_15_15 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_1_markers"], sorted_Markers["id_15_markers"])
            self.candidate_dist_1_22_1, self.candidate_dist_1_22_22 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_1_markers"], sorted_Markers["id_22_markers"])
            self.candidate_dist_1_30_1, self.candidate_dist_1_30_30 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_1_markers"], sorted_Markers["id_30_markers"])
            self.candidate_dist_1_99_1, self.candidate_dist_1_99_99 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_1_markers"], sorted_Markers["id_99_markers"])
            self.candidate_dist_1_435_1, self.candidate_dist_1_435_435 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_1_markers"], sorted_Markers["id_435_markers"])
            self.candidate_dist_15_22_15, self.candidate_dist_15_22_22 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_15_markers"], sorted_Markers["id_22_markers"])
            self.candidate_dist_15_30_15, self.candidate_dist_15_30_30 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_15_markers"], sorted_Markers["id_30_markers"])
            self.candidate_dist_15_99_15, self.candidate_dist_15_99_99 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_15_markers"], sorted_Markers["id_99_markers"])
            self.candidate_dist_15_435_15, self.candidate_dist_15_435_435 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_15_markers"], sorted_Markers["id_435_markers"])
            self.candidate_dist_22_30_22, self.candidate_dist_22_30_30 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_22_markers"], sorted_Markers["id_30_markers"])
            self.candidate_dist_22_99_22, self.candidate_dist_22_99_99 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_22_markers"], sorted_Markers["id_99_markers"])
            self.candidate_dist_22_435_22, self.candidate_dist_22_435_435 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_22_markers"], sorted_Markers["id_435_markers"])
            self.candidate_dist_30_99_30, self.candidate_dist_30_99_99 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_30_markers"], sorted_Markers["id_99_markers"])
            self.candidate_dist_30_435_30, self.candidate_dist_30_435_435 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_30_markers"], sorted_Markers["id_435_markers"])
            self.candidate_dist_99_435_99, self.candidate_dist_99_435_435 = self.giveShortestDistanceFromTwoMarkerArrays(sorted_Markers["id_99_markers"], sorted_Markers["id_435_markers"])

            all_candidates = [
                self.candidate_1_15_1, self.candidate_1_15_15, self.candidate_dist_1_22_1, self.candidate_dist_1_22_22,
                self.candidate_dist_1_30_1, self.candidate_dist_1_30_30, self.candidate_dist_1_99_1, self.candidate_dist_1_99_99, self.candidate_dist_1_435_1, self.candidate_dist_1_435_435,
                self.candidate_dist_15_22_15, self.candidate_dist_15_22_22, self.candidate_dist_15_30_15, self.candidate_dist_15_30_30, self.candidate_dist_15_99_15, self.candidate_dist_15_99_99,
                self.candidate_dist_15_435_15, self.candidate_dist_15_435_435, self.candidate_dist_22_30_22, self.candidate_dist_22_30_30, self.candidate_dist_22_99_22, self.candidate_dist_22_99_99,
                self.candidate_dist_22_435_22, self.candidate_dist_22_435_435, self.candidate_dist_30_435_30, self.candidate_dist_30_99_30, self.candidate_dist_30_99_99, self.candidate_dist_30_435_435, self.candidate_dist_99_435_99, self.candidate_dist_99_435_435
            ]
            
            translation_array = [marker.translation for marker in all_candidates if marker.translation is not None]
            marker_array = [marker for marker in all_candidates if marker.translation is not None]
            medianTrans = np.median(translation_array, axis=0)
            self.closestToMedian = self.giveShortestDistanceFromMedian(marker_array, medianTrans)
        else:
            for key, markerArray in sorted_Markers.items():
                if markerArray != []:
                    translation_array = [marker.translation for marker in markerArray if marker.translation is not None]
                    medianTrans = np.median(translation_array, axis=0)
                    self.closestToMedian = self.giveShortestDistanceFromMedian(markerArray, medianTrans)

    def get_o3d_extrinsic_matrix(self):
        unity_to_o3d_y = np.array([
            [1,0,0],
            [0,-1,0],
            [0,0,1]
        ])

        unity_to_o3d_x = np.array([
            [-1,0,0],
            [0,1,0],
            [0,0,1]
        ])
        o3d_rot = unity_to_o3d_x @ self.closestToMedian.depCamInCubeRot @ unity_to_o3d_y
        o3d_trans = unity_to_o3d_x @ self.closestToMedian.depCamInCubeTrans
        transformation_matrix = np.eye(4,4)
        translation_vector = o3d_trans.flatten()
        transformation_matrix[0:3, 0:3] = o3d_rot
        transformation_matrix[0:3, 3] = translation_vector
        return transformation_matrix
            
    def countAmountMarkersWhereTracked(self, sorted_Markers: Dict[str, List[TrackedMarker]]):
        count = 0
        for key, markerArray in sorted_Markers.items():
            if markerArray != []: count = count+1
        return count

    def filteredMarkers(self, array: List[TrackedMarker]):
        filtered_markers: List[TrackedMarker] = []
        for i in range(len(array)):
            # Assume no duplicates initially
            duplicate = False
            for filtered_marker in filtered_markers:
                if np.all(array[i].translation == filtered_marker.translation):
                    # Translation is the same, indicating a duplicate
                    duplicate = True
                    break
            if not duplicate:
                # If no duplicate is found, add the marker to filtered_1_markers
                filtered_markers.append(array[i])
        return filtered_markers
    
    def find_marker_by_id(self, marker_array, target_id):
        for marker in marker_array:
            if marker.id == target_id:
                return marker
        return None  # Rückgabe, falls die ID nicht gefunden wurde

    ###### Transform Marker in OpenCV Cam in Unity Cam
    def fromOCVCamInUnityCam(self, rot, trans):
        OCV_in_U = np.array([[1,  0,  0],
         [0,  -1, 0],
         [0, 0, 1]])
        R_in_Unity_Cam = OCV_in_U @ rot
        T_in_Unity_Cam = OCV_in_U @ trans
        #print("Rotate in Unity Cam: ", R_in_Unity_Cam, "\n transform in unity Cam: ", T_in_Unity_Cam)
        return R_in_Unity_Cam, T_in_Unity_Cam
    
    ###### Transform Camera in Unity Marker
    #OpenCV Coord in Unity Coord
    def fromCamInMarkerKoord(self, rot, trans):
        OCV_M_in_U = np.array([
            [-1,  0,  0],
            [0,  1, 0],
            [0, 0, 1]])
        RC_in_Marker = np.transpose(rot)
        TC_in_Marker = -RC_in_Marker@trans
        #print("\nRotate in Unity Marker: ", RC_in_Unity_Marker, "\n transform in unity Marker: ", TC_in_Unity_Marker)
        return RC_in_Marker, TC_in_Marker
    
    #Rotation Computation: CubeRotationInUnityRotation @ MarkerRotationInCubeRotation @ CamRotationInMarkerRotation
    def fromMarkerInUnityCubeSystem(self, R, trans, MarkerId):
        cubeMarker: MarkerOnCube = self.find_marker_by_id(all_Markers, MarkerId)
        RC_in_Cube = cubeMarker.originRotation @ R
        TC_in_Cube = cubeMarker.originTranslation + (cubeMarker.originRotation @ trans)
        return RC_in_Cube, TC_in_Cube
    
    def safeAllDependetCamValues(self, id_Marker_Array: List[TrackedMarker]):
        for id_Marker in id_Marker_Array:
            rot_Unity_Cam_Main, trans_Unity_Cam_Main = self.fromOCVCamInUnityCam(id_Marker.rotation, id_Marker.translation)
            rot_Marker_Main, trans_Marker_Main = self.fromCamInMarkerKoord(rot_Unity_Cam_Main, trans_Unity_Cam_Main)
            rot_Unity_Main, trans_Unity_Main = self.fromMarkerInUnityCubeSystem(rot_Marker_Main, trans_Marker_Main, id_Marker.id)
            
            id_Marker.depCamInCubeRot = rot_Unity_Main
            id_Marker.depCamInCubeTrans = trans_Unity_Main
            print(id_Marker.id)
            print(id_Marker.depCamInCubeRot)
            
    def computeOriginDistance(self, trans1, trans2):
        x1 = trans1[0]
        y1 = trans1[1]
        z1 = trans1[2]
        x2 = trans2[0]
        y2 = trans2[1]
        z2 = trans2[2]
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance

    def giveShortestDistanceFromTwoMarkerArrays(self, array1: List[TrackedMarker], array2: List[TrackedMarker]):
        if len(array1) > 0 and len(array2) > 0:
            shortest1 = None
            shortest2 = None

            # initialize the shortest Distance with the biggest value
            shortest_dist = float('inf')

            for marker1 in array1:
                for marker2 in array2:
                    try:
                        dist = self.computeOriginDistance(marker1.depCamInCubeTrans, marker2.depCamInCubeTrans)
                        if dist < shortest_dist:
                            shortest_dist = dist
                            shortest1 = marker1
                            shortest2 = marker2
                    except Exception as e:
                        pass
                    
            if shortest1 is None or shortest2 is None:
                print("Keine gültigen Marker gefunden.")
                return TrackedMarker(), TrackedMarker()
            else:
                return shortest1, shortest2
        else:
            #One of the Arrays is empty
            return TrackedMarker(), TrackedMarker()
        
    def giveShortestDistanceFromMedian(self, array: List[TrackedMarker], median):
        candidate = None
        shortest_dist = float('inf')
        for marker in array:
            dist = self.computeOriginDistance(marker.translation, median)
            if dist < shortest_dist:
                shortest_dist = dist
                candidate = marker
        return candidate