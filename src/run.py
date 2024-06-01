import argparse
import os
from CamParamsExtractor import CamParamsExtractor
from ExtrinsicCalculator import ExtrinsicCalculator
from CamExtris import CamExtris
from ExtriJsonHandlers import ExtriJsonCreator, ExtriJsonLoader
from ComKinectRecordingExtractor import ComKinectRecordingExtractor
from PcdHandler import PcdHandler

def get_parsed_args():
    parser = argparse.ArgumentParser(description="Code to Extract Pointclouds an Extrinsic Params of Cams.")
    parser.add_argument("--recordings_path", type=str, help="Path to all Recordings")
    parser.add_argument("--mkvToolNix_path", type=str, help="Path to installed MKVToolNix")
    parser.add_argument("--amount_Subs", type=int, help="e.g. If you have 3 Cams, you have 2 Subs")
    parser.add_argument("--marker_length", type=float, help="length of the Marker...they should be square")
    parser.add_argument("--sub_path", type=str, default="", help="Folder name of Scene")
    parser.add_argument("--kinect_pic_rec_extractor", default=".\\venv\\Lib\\site-packages\\open3d\\examples\\reconstruction_system\\sensors", type=str)
    parser.add_argument("--icp_itteration", type=int, default=25, help="Amount of ICP Itterations for the Pointcloud Postpro")
    parser.add_argument("--create_pcd_json", action='store_true', help="Flag if an pcd Json of multiple Frames should be created")
    parser.add_argument("--amount_pcd_frames", type=int, default=5, help="Amount of Frames which should be saved in pcd json. If --create_pcd is False, this is not necessary.")
    parser.add_argument("--create_dg_init_npz", action='store_true', help="If you want to create a dynamic Gaussian, you need a init pcd an you should set this to yes")
    parser.add_argument("--pcd_just_center", action='store_true', help="If the whole set should be shown or just the center")
    parser.add_argument("--create_npzs", action='store_true', help="If you want to use dynamic gaussian spaltting you may need some npzs...this could become very disc intense (maybe 10GB or more)")
    return parser.parse_args()

def ensure_trailing_backslash(path):
        if not path.endswith("\\"):
            return path + "\\"
        return path

def get_all_extrinsic_Matrices(recording_path, marker_length, amount_subs):
    all_extris = []
    if not os.path.isfile(recording_path + "extris.json"):
        print("Extrinsic Json doesn't exist. Creating new one")
        all_extris = []
        trackerM = ExtrinsicCalculator(recording_path, marker_length, "M")
        extrM = CamExtris("M", trackerM.get_extrinsic_matrix())
        all_extris.append(extrM)

        for sub in range(amount_subs):
            tracker = ExtrinsicCalculator(recording_path, marker_length, "S" + str(sub + 1))
            extr = CamExtris("S"+str(sub + 1), tracker.get_extrinsic_matrix())
            all_extris.append(extr)

        ExtriJsonCreator(all_extris, recording_path)
    else: 
        print("Extrinsic Json exist. Continuing with old values")
        extri_loader = ExtriJsonLoader(str(recording_path + "extris.json"))
        all_extris = extri_loader.get_all_extris()
    return all_extris

def main():
    args = get_parsed_args()
    args.recordings_path = ensure_trailing_backslash(args.recordings_path)
    args.mkvToolNix_path = ensure_trailing_backslash(args.mkvToolNix_path)
    # Extract Cam Params
    CamParamsExtractor(args.recordings_path, args.mkvToolNix_path, args.amount_Subs)
    # Get Extrinsic Cam-Params
    all_extris = get_all_extrinsic_Matrices(args.recordings_path, args.marker_length, args.amount_Subs)
    # Create rgb and Depth Images
    ComKinectRecordingExtractor(args.recordings_path, args.sub_path, args.kinect_pic_rec_extractor, args.amount_Subs)
    #PCD creation, postpro and visualisation
    #Open3d will visualise the pcd of whole Scene but the pcd.json will only be the person in the middle
    print("args just show center: " + str(args.pcd_just_center))
    print("args just show center: " + str(args.create_npzs))
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

main()