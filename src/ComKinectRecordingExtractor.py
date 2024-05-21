import os
import subprocess

class ComKinectRecordingExtractor:
    def __init__(self, path, sub_path, kinect_pic_rec_extractor, sub_amount):
        self.sub_path = sub_path
        self.path = path + self.sub_path
        self.path = self.ensure_trailing_backslash(self.path)
        self.sub_amount = sub_amount
        self.pic_extractor = self.ensure_trailing_backslash(kinect_pic_rec_extractor)
        self.createImages()

    def ensure_trailing_backslash(self, path):
        if not path.endswith("\\"):
            return path + "\\"
        return path

    def Image_Extracts_Exist(self):
        if not os.path.isfile(self.path + "intriM.json"):
            print("Master Camera Params .json is missing. Creating new Jsons.") 
            return False
        for sub in range(self.sub_amount):
            if not os.path.isfile(self.path + "intriS"+ str(sub + 1) +".json"):
                print("One Sub Camera Params .json is missing. Creating new Jsons")
                return False
        print("every Cam Param Json exists. Using the existing ones.")
        return True
    
    def createImages(self):
        calib_mkv_name = ""
        if self.sub_path == "":
            calib_mkv_name = "calib"
        commands = [] 
        commandM = "python " + self.pic_extractor + "azure_kinect_mkv_reader.py --input " + self.path + calib_mkv_name + "M.mkv --output " + self.path + "M\\"
        commands.append(commandM)
        for sub in range(self.sub_amount):
            commandS = "python " + self.pic_extractor + "azure_kinect_mkv_reader.py --input " + self.path + calib_mkv_name + "S" + str(sub + 1) + ".mkv --output " + self.path + "S" + str(sub + 1) + "\\"
            print(commandS)
            commands.append(commandS)
        
        for command in commands:
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            print("stdout:", result.stdout)
            print("stderr:", result.stderr)