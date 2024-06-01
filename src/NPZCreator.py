import numpy as np
import os

class NPZCreator:
    def __init__(self, all_pcds_per_frame, loadingAmount, path) -> None:
        if(self.checkNPZDirectory(path)):
            print("npz-Folder exists")
        else:
            try:
                os.makedirs(path + "npzs/", exist_ok=True)
                print(f"Directory '{path}' created successfully")
            except OSError as error:
                print(f"Error creating directory '{path}': {error}")
        self.saveFrames(all_pcds_per_frame, loadingAmount, path)

    def checkNPZDirectory(self, path)-> bool:
        print(path + "npzs/")
        return os.path.exists(path + "npzs/")
    
    def saveFrames (self, all_pcds_per_frame, loadingAmount, path):
        for frame in range(loadingAmount):
            print("creating npzs: ", frame, " of ", loadingAmount)
            xyz_coordinates = np.asarray(all_pcds_per_frame[frame].points)
            colors = np.asarray(all_pcds_per_frame[frame].colors)
            ones_column = np.ones((xyz_coordinates.shape[0], 1))

            # Concatenate x, y, z coordinates, ones, and colors horizontally
            init_pcd_stack = np.hstack((xyz_coordinates, colors, ones_column))

            init_pcd_np = np.asarray(init_pcd_stack)
            np.savez(path + "npzs/ATLFB_" + str(frame+1) + ".npz", data=init_pcd_np)