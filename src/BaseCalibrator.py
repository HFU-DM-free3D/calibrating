import os
from CamExtris import CamExtris
from CamParamsExtractor import CamParamsExtractor
from ComKinectRecordingExtractor import ComKinectRecordingExtractor
from ExtriJsonHandlers import ExtriJsonCreator, ExtriJsonLoader
from ExtrinsicCalculator import ExtrinsicCalculator
from PcdHandler import PcdHandler
from abc import ABC, abstractmethod
import time


class BaseCalibrator(ABC):
    def __init__(self) -> None:
        print("init BaseCalibrator")

    @abstractmethod
    def GetProxy(self, baseProxy):
        pass

    @abstractmethod
    async def SetStatus(self, status):
        pass
    
    def ensure_trailing_backslash(self, path):
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
    
    async def main(self, baseProxy):

        await self.SetStatus("Calibrating")

        args = self.GetProxy(baseProxy=baseProxy)
        args.recordings_path = self.ensure_trailing_backslash(args.recordings_path)
        args.mkvToolNix_path = self.ensure_trailing_backslash(args.mkvToolNix_path)
        # Extract Cam Params
        CamParamsExtractor(args.recordings_path, args.mkvToolNix_path, args.amount_Subs)
        # Get Extrinsic Cam-Params
        all_extris = []
        all_extris = self.get_all_extrinsic_Matrices(
            recording_path=args.recordings_path, 
            marker_length=args.marker_length, 
            amount_subs=args.amount_Subs, 
            use_charuco=args.use_charuco)
        
        
        # Create rgb and Depth Images
        ComKinectRecordingExtractor(args.recordings_path, args.sub_path, args.kinect_pic_rec_extractor, args.amount_Subs)
        #PCD creation, postpro and visualisation
        #Open3d will visualise the pcd of whole Scene but the pcd.json will only be the person in the middle
        PcdHandler(
            loading_amount=args.amount_pcd_frames, 
            sub_amount=args.amount_Subs, 
            icp_its=args.icp_itteration, 
            just_show_center=args.pcd_just_center, 
            path=args.recordings_path, 
            sub_path=args.sub_path, 
            extris=all_extris, 
            create_pcd_json=args.create_pcd_json, 
            create_pcd_npz=args.create_npzs
            )
        
        await self.SetStatus("Finished")

        time.sleep(2)

        await self.SetStatus("Idle")