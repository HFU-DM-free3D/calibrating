import json

masterInstrinsic = [
    [
        915.6092529296875,
        0.0,
        961.67230224609375,
    ],
    [
        0.0,
        915.8900146484375,
        549.2357177734375,
    ],
    [
        0.0,
        0.0,
        1.0
    ]
]

subOneInstrinsic =[
    [
        	900.690185546875,
            0.0,
            960.36346435546875,
    ],
    [
        0.0,
        900.589111328125,
        	548.78515625,
    ],
    [
        0.0,
        0.0,
        1.0
    ]
]

subTwoInstrinsic =[
    [
        909.39447021484375,
        0.0,
        961.20184326171875,
    ],
    [
        0.0,
        	909.46435546875,
            554.30865478515625
    ],
    [
        0.0,
        0.0,
        1.0
    ]
]

frames = []

for index in range(298):
    frame = [
        masterInstrinsic,
        subOneInstrinsic,
        subTwoInstrinsic
    ]
    frames.append(frame)

# with open("k.json","w") as json_datei:
#     json.dump(frames, json_datei,indent=4)

extr_M = [
[-0.9939503, -0.03950024, 0.10248183, -0.14961497],
[0.02270801, -0.98683595, -0.16012228, 1.85014322],
[0.10745762, -0.15682643, 0.98176287, -3.9282198],
[0, 0, 0, 1],
]

extr_S1 = [
[ 0.54285377, -0.28281817, 0.79077254, -2.65024317],
[ 0.01499845, -0.93816215, -0.34582666, 1.86244479],
[ 0.83968532, 0.19959156, -0.50503787, 1.82038517],
[ 0. , 0. , 0. , 1. ],
]

extr_S2 = [
[ 0.56487919, 0.32439232, -0.75873528, 3.71676524],
[ 0.04687117, -0.93060586 ,-0.36297643, 1.96869398],
[-0.82382832, 0.16947774 ,-0.54088989, 1.07754278],
[ 0. , 0. , 0. , 1. ],
]

frames_extr = []

for index in range(298):
    frame = [
        extr_M,
        extr_S1,
        extr_S2
    ]
    frames_extr.append(frame)


# with open("w2c.json","w") as json_datei:
#     json.dump(frames_extr, json_datei,indent=4)


outer_array = []

for i in range(298):
    inner_array = []
    for j in range(3):
        inner_array.append("{}/{}.jpg".format(j+1, str(i).zfill(5)))
    outer_array.append(inner_array)

# print(outer_array)

# with open("fn.json","w") as json_datei:
#     json.dump(outer_array, json_datei,indent=4)


ids = []

for _ in range(298):
    id = [i for i in range(1, 4)]
    ids.append(id)

# print(outer_array)

with open("cam_id.json","w") as json_datei:
    json.dump(ids, json_datei,indent=4)