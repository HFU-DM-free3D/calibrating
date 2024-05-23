# This program should extract the camera parameters of the Kinect from the MKV. 
# The distortion values and intrinsic camera matrices of the color camera are required 
# for the camera calibration. However, this step must/can only be carried out with 
# the calibration scene as the Aruco marker can be seen there.

# 21.05.24 lkswllmnn 

import os
import subprocess
import locale

class CamParamsExtractor:
    """
    Class to extract the file which contains the distortion values and intrinsic camera matrix of the Azure Kinect.
    The PATH attribute requires the path to the folder system of the MKV recordings.
    The PATH_TO_TOOLNIX attribute requires the path to the MKVToolNix program. This must be installed. https://mkvtoolnix.download/downloads.html(May 2024)
    The tool mkvextract for Windows in version mkvextract v83.0 ('Circle Of Friends') 64-bit was used for extraction.
    """

    def __init__(self, path, path_to_toolNix, sub_amount):
        self.path = path
        self.path_to_toolNix = path_to_toolNix
        self.sub_amount = sub_amount
        if not self.CameraParamsJsonsExist():
            self.createJsonWithToolNix()
    
    def CameraParamsJsonsExist(self):
        if not os.path.isfile(self.path + "intriM.json"):
            print("Master Camera Params .json is missing. Creating new Jsons.") 
            return False
        for sub in range(self.sub_amount):
            if not os.path.isfile(self.path + "intriS"+ str(sub + 1) +".json"):
                print("One Sub Camera Params .json is missing. Creating new Jsons")
                return False
        print("every Cam Param Json exists. Using the existing ones.")
        return True
    
    def createJsonWithToolNix(self):
        commands = []
        path_to_toolNix = self.path_to_toolNix
        path = self.path
        commandM = f'"{path_to_toolNix}mkvextract" "{path}calibM.mkv" attachments 1:"{path}intriM.json"'
        commands.append(commandM)

        for sub in range(self.sub_amount):
            commandS = f'"{path_to_toolNix}mkvextract" "{path}calibS{sub + 1}.mkv" attachments 1:"{path}intriS{sub + 1}.json"'
            print(commandS)
            commands.append(commandS)

        preferred_encoding = locale.getpreferredencoding()

        for command in commands:
            result = subprocess.run(command, shell=True, capture_output=True, text=True, encoding=preferred_encoding)
            print("stdout:", result.stdout)
            print("stderr:", result.stderr)