import os
import subprocess
import locale

class ComKinectRecordingExtractor:
    def __init__(self, path, sub_path, kinect_pic_rec_extractor, sub_amount):
        self.sub_path = sub_path
        self.path = path + self.sub_path
        self.path = self.ensure_trailing_backslash(self.path)
        self.sub_amount = sub_amount
        self.pic_extractor = self.ensure_trailing_backslash(kinect_pic_rec_extractor)
        if self.Image_Extracts_Exist():
            print("It seems that there are already pictures. To create new ones delete the old Folders M, S1 ... first.")
        else:
            print("No existing rgb and depth images. New ones are generated.")
            self.createImages()

    def ensure_trailing_backslash(self, path):
        if not path.endswith("\\"):
            return path + "\\"
        return path

    def Image_Extracts_Exist(self):
        if not os.path.isdir(self.path + "M\\"):
            print("Master Picture Folder exists") 
            return False
        for sub in range(self.sub_amount):
            if not os.path.isdir(self.path + "S" + str(sub + 1) + "\\"):
                print("Sub", str(sub + 1), " exists")
                return False
        print("all needed Directorys exist.")
        return True
    
    def createImages(self):
        calib_mkv_name = ""
        if self.sub_path == "":
            calib_mkv_name = "calib"
        pic_extractor = self.pic_extractor
        path = self.path

        commands = [] 
        commandM = f'python "{pic_extractor}azure_kinect_mkv_reader.py" --input "{path}{calib_mkv_name}M.mkv" --output "{path}M/"'
        commands.append(commandM)
        for sub in range(self.sub_amount):
            commandS = f'python "{pic_extractor}azure_kinect_mkv_reader.py" --input "{path}{calib_mkv_name}S{sub + 1}.mkv" --output "{path}S{sub + 1}/"'
            print(commandS)
            commands.append(commandS)
        
        preferred_encoding = locale.getpreferredencoding()

        for command in commands:
            result = subprocess.run(command, shell=True, capture_output=True, text=True, encoding=preferred_encoding)
            print("stdout:", result.stdout)
            print("stderr:", result.stderr)