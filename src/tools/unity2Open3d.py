import numpy as np

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

def newRot(unityRot):
    return  unity_to_o3d_x @ unityRot @ unity_to_o3d_y

def newTrans(newRot, unityTrans):
    return unity_to_o3d_x @ unityTrans

masterRot = np.array([
    [0.9939503, -0.03950024, -0.10248183],
    [0.02270801,  0.98683595, -0.16012228],
    [0.10745762,  0.15682643,  0.98176287],
])
masterTrans = np.array([0.14961497, 1.85014322, -3.9282198])

o3d_Master_Rot = newRot(masterRot)
o3d_Master_Trans = newTrans(o3d_Master_Rot, masterTrans)
print("M Rot: \n", o3d_Master_Rot, "\n trans: \n", o3d_Master_Trans)

Sub1Rot = np.array([
    [-0.54274409, -0.28253118, -0.79095195],
    [0.01431065,  0.9384762,  -0.34504727],
    [0.8397762,  -0.1985914,  -0.5053092 ],
])
Sub1Trans = np.array([2.7321261, 1.87120594, 1.88317298])

o3d_S1_Rot = newRot(Sub1Rot)
o3d_S1_Trans = newTrans(o3d_S1_Rot, Sub1Trans)
print("S1 Rot: \n", o3d_S1_Rot, "\n trans: \n", o3d_S1_Trans)

Sub2Rot = np.array([
    [-0.565007,    0.32458066,  0.7585608],
    [0.0478601,   0.93071658, -0.36259626],
    [-0.82369685, -0.16856463, -0.54139585],
])
Sub2Trans = np.array([-3.75002756, 1.98196877, 1.10257667])

o3d_S2_Rot = newRot(Sub2Rot)
o3d_S2_Trans = newTrans(o3d_S2_Rot, Sub2Trans)
print("S2 Rot: \n", o3d_S2_Rot, "\n trans: \n", o3d_S2_Trans)

