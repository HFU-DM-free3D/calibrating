import argparse
import asyncio
from BaseCalibrator import BaseCalibrator


class ScriptCalibrator(BaseCalibrator):
    async def SetStatus(self, status):
        print(status)
        pass

    def GetProxy(self, baseProxy):
        parser = argparse.ArgumentParser(description="Code to Extract Pointclouds an Extrinsic Params of Cams.")
        parser.add_argument("--recordings_path", type=str, help="Path to all Recordings")
        parser.add_argument("--mkvToolNix_path", type=str, help="Path to installed MKVToolNix")
        parser.add_argument("--amount_Subs", type=int, help="e.g. If you have 3 Cams, you have 2 Subs")
        parser.add_argument("--marker_length", type=float, help="length of the Marker...they should be square")
        parser.add_argument("--use_charuco", action='store_true', help="Flag to set if the calibration Target was a charuco board")
        parser.add_argument("--sub_path", type=str, default="", help="Folder name of Scene")
        parser.add_argument("--kinect_pic_rec_extractor", default=".\\venv\\Lib\\site-packages\\open3d\\examples\\reconstruction_system\\sensors", type=str)
        parser.add_argument("--icp_itteration", type=int, default=25, help="Amount of ICP Itterations for the Pointcloud Postpro")
        parser.add_argument("--create_pcd_json", action='store_true', help="Flag if an pcd Json of multiple Frames should be created")
        parser.add_argument("--amount_pcd_frames", type=int, default=5, help="Amount of Frames which should be saved in pcd json. If --create_pcd is False, this is not necessary.")
        parser.add_argument("--create_dg_init_npz", action='store_true', help="If you want to create a dynamic Gaussian, you need a init pcd an you should set this to yes")
        parser.add_argument("--pcd_just_center", action='store_true', help="If the whole set should be shown or just the center")
        parser.add_argument("--create_npzs", action='store_true', help="If you want to use dynamic gaussian spaltting you may need some npzs...this could become very disc intense (maybe 10GB or more)")
        return parser.parse_args()
    

scriptCalibrator: ScriptCalibrator = ScriptCalibrator()

asyncio.run(scriptCalibrator.main(None))