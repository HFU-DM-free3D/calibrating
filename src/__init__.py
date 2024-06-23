import asyncio
import websockets
from pyee import AsyncIOEventEmitter
import json
import argparse
import os
from CamParamsExtractor import CamParamsExtractor
from ExtrinsicCalculator import ExtrinsicCalculator
from CamExtris import CamExtris
from ExtriJsonHandlers import ExtriJsonCreator, ExtriJsonLoader
from ComKinectRecordingExtractor import ComKinectRecordingExtractor
from PcdHandler import PcdHandler


webSocketAdress: str = "192.168.178.34"
webSocketPort: str = "8080"



class WebSocketHooks(AsyncIOEventEmitter):
    NEW_CONNECTION = "NEW_CONNECTION"

    async def DispatchHook(self, hook, body):
        self.emit(hook, body)

    def SubscribeHookListener(self, hook, listener):
        self.on(hook, listener)

    def UnsubscripeListener(self, hook, listener):
        self.remove_listener(hook, listener)


class ReceivedEvent:
    def __init__(self, name: str):
        self.eventName = name
        self.data = {}

    @property
    def JSONString(self) -> str:
        return json.dumps(self.__dict__)

    def addData(self, key: str, value):
        self.data[key] = value

def CreateStateUpdate(state:str):
    stateUpdate: ReceivedEvent = ReceivedEvent("PROCESS_CALIBRATOR")
    stateUpdate.addData("State",state)
    return stateUpdate.JSONString

class Calibrator:

    def get_parsed_args(self):
        parser = argparse.ArgumentParser(description="Code to Extract Pointclouds an Extrinsic Params of Cams.")
        # parser.add_argument("--recordings_path", type=str, help="Path to all Recordings")
        # parser.add_argument("--mkvToolNix_path", type=str, help="Path to installed MKVToolNix")
        # parser.add_argument("--amount_Subs", type=int, help="e.g. If you have 3 Cams, you have 2 Subs")
        # parser.add_argument("--marker_length", type=float, help="length of the Marker...they should be square")        
        # # parser.add_argument("--sub_path", type=str, default="", help="Folder name of Scene")       
        #  # parser.add_argument("--create_pcd_json", action='store_true', help="Flag if an pcd Json of multiple Frames should be created")

        parser.add_argument("--use_charuco", action='store_true', help="Flag to set if the calibration Target was a charuco board")
        # parser.add_argument("--kinect_pic_rec_extractor", default=".\\venv\\Lib\\site-packages\\open3d\\examples\\reconstruction_system\\sensors", type=str)
        parser.add_argument("--icp_itteration", type=int, default=25, help="Amount of ICP Itterations for the Pointcloud Postpro")
        parser.add_argument("--amount_pcd_frames", type=int, default=5, help="Amount of Frames which should be saved in pcd json. If --create_pcd is False, this is not necessary.")
        parser.add_argument("--create_dg_init_npz", action='store_true', help="If you want to create a dynamic Gaussian, you need a init pcd an you should set this to yes")
        parser.add_argument("--pcd_just_center", action='store_true', help="If the whole set should be shown or just the center")
        parser.add_argument("--create_npzs", action='store_true', help="If you want to use dynamic gaussian spaltting you may need some npzs...this could become very disc intense (maybe 10GB or more)")
        return parser.parse_args()

    def ensure_trailing_backslash(self,path):
            if not path.endswith("\\"):
                return path + "\\"
            return path

    def get_all_extrinsic_Matrices(self,recording_path, marker_length, amount_subs, use_charuco, ch_marker_len = 0.112, ch_square_len = 0.150, ch_dict = 0, ch_board_size=(3,5)):
        all_extris = []
        if not os.path.isfile(recording_path + "extris.json"):
            print("Extrinsic Json doesn't exist. Creating new one")
            all_extris = []
            trackerM = ExtrinsicCalculator(
                path=recording_path, 
                marker_length=marker_length, 
                cam_role="M", 
                use_Charuco=use_charuco, 
                ch_marker_len=ch_marker_len, 
                ch_square_len=ch_square_len,
                ch_dict=ch_dict,
                ch_board_size=ch_board_size
                )
            extrM = CamExtris("M", trackerM.get_extrinsic_matrix())
            all_extris.append(extrM)

            for sub in range(amount_subs):
                tracker = ExtrinsicCalculator(
                    path=recording_path, 
                    marker_length=marker_length, 
                    cam_role="S" + str(sub + 1),
                    use_Charuco=use_charuco, 
                    ch_marker_len=ch_marker_len, 
                    ch_square_len=ch_square_len,
                    ch_dict=ch_dict,
                    ch_board_size=ch_board_size)
                extr = CamExtris("S"+str(sub + 1), tracker.get_extrinsic_matrix())
                all_extris.append(extr)
                print("working: " + str(all_extris))

            ExtriJsonCreator(all_extris, recording_path)
        else: 
            print("Extrinsic Json exist. Continuing with old values")
            extri_loader = ExtriJsonLoader(str(recording_path + "extris.json"))
            all_extris = extri_loader.get_all_extris()
        return all_extris

    async def main(self,websocket ,subCount,markerLength,subPath,createPCDJSON,useCharuco,icpItteration,amountPcdFrames,pcdJustCenter,createNpzs,createDgInitNpz=False ):
        recordingPath = "C:\\Users\\nicka\\Desktop\\Test"
        toolKitPath = "C:\\Program Files\\MKVToolNix"
        kinectPicRecExtractor =".\\venv\\Lib\\site-packages\\open3d\\examples\\reconstruction_system\\sensors"

        await websocket.send(CreateStateUpdate("Calibrating"))


        # args = self.get_parsed_args()
        recordingPath = self.ensure_trailing_backslash(recordingPath)
        toolKitPath = self.ensure_trailing_backslash(toolKitPath)
        # Extract Cam Params
        CamParamsExtractor(recordingPath, toolKitPath, subCount)
        # Get Extrinsic Cam-Params
        all_extris = []
        all_extris = self.get_all_extrinsic_Matrices(
            recording_path=recordingPath, 
            marker_length=markerLength, 
            amount_subs=subCount, 
            use_charuco=useCharuco)
        
        
        # Create rgb and Depth Images
        ComKinectRecordingExtractor(recordingPath, subPath, kinectPicRecExtractor, subCount)
        #PCD creation, postpro and visualisation
        #Open3d will visualise the pcd of whole Scene but the pcd.json will only be the person in the middle
        PcdHandler(
            loading_amount=amountPcdFrames, 
            sub_amount=subCount, 
            icp_its=icpItteration, 
            just_show_center=pcdJustCenter, 
            path=recordingPath, 
            sub_path=subPath, 
            extris=all_extris, 
            create_pcd_json=createPCDJSON, 
            create_pcd_npz=createNpzs
            )
        
        await websocket.send(CreateStateUpdate("Finished"))

        # await websocket.send(CreateStateUpdate("Finished"))



async def connect_to_server():
    calibrationHooks = WebSocketHooks()
    calibrator = Calibrator()
    url: str = f"ws://{webSocketAdress}:{webSocketPort}"
    async with websockets.connect(url) as websocket:
        async def OnTriggerCalibration(body:object):
            subCount = body["SubCount"]
            markerLength = body["MarkerLength"]
            subPath= body["SubPath"]
            createPCDJSON = body["CreateJSON"]

            useCharuco = body["UseCharuco"]
            # kinectPicRecExtractor = body.KinectPicRecExtractor
            icpItteration = body["IcpItteration"]
            amountPcdFrames = body["AmountPcdFrames"]
            # createDgInitNpz = body.CreateDgInitNpz
            pcdJustCenter = body["PcdJustCenter"]
            createNpzs = body["CreateNpzs"]

            # print(subCount,markerLength,subPath,createPCDJSON,useCharuco,icpItteration,amountPcdFrames,pcdJustCenter,createNpzs)

            await calibrator.main(websocket=websocket,subCount=subCount,markerLength=markerLength,subPath=subPath,createPCDJSON=createPCDJSON,useCharuco=useCharuco,icpItteration=icpItteration,amountPcdFrames=amountPcdFrames,pcdJustCenter=pcdJustCenter,createNpzs=createNpzs)

        calibrationHooks.SubscribeHookListener(hook="ON_TRIGGER_CALIBRATION",listener=OnTriggerCalibration)

        initCalibrator: ReceivedEvent = ReceivedEvent("INIT_CALIBRATOR")
        await websocket.send(initCalibrator.JSONString)

        while True:
            response = await websocket.recv()
            
            body = json.loads(response)

            event = body.get("eventName", "")
            data = body.get("data", {})

            print(event)
            print(data)

            await calibrationHooks.DispatchHook(hook=event,body=data)

            











asyncio.get_event_loop().run_until_complete(connect_to_server())