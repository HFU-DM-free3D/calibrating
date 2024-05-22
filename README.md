# How to start the Repo
1. Clone the repo
2. Install MKVToolNix https://mkvtoolnix.download/downloads.html
3. Create a venv ( python -m venv venv)
4. Check Restrictions (on Windows --> cmd or powershell: Get-ExecutionPolicy). If they are restricted you have to unrestrict them (cmd: Set-ExecutionPolicy Unrestricted -Scope Process)
5. Activate venv (cmd: venv/Scripts/activate)
6. install numpy (cmd: pip install numpy)
7. install opencv (cmd: pip install opencv-contrib-python)
8. install open3d (cmd: pip install open3d)
9. run the run.py Script e.g. with a command to extract a pcd Json (python src/run.py --recordings_path [your own path e.g. C:\Users\User\Desktop\Videos] --mkvToolNix_path [your own path e.g. D:\MKVToolNix] --amount_Subs 2 --marker_length 0.15 --sub_path ATLFB --create_pcd_json True)

# calibrating
This project centers on calibrating three Kinect cameras for Free3D, employing Aruco markers. This calibration trys to ensure precise alignment, vital for accurate 3D reconstructions, enhancing overall project quality and effectiveness.

## Project Overview

### System Requirements
| Component   | Version   |
|-------------|-----------|
| Python      | 3.11      |
| OpenCV      | 4.8.1     |
| Open3D      | 0.18      |

### Tested on
- Windows 11
- Nvidia GTX 1670Ti

## About
This repository is created in conjunction with the Free3D project at the Faculty of DM, Hochschule Furtwangen. Its purpose is to calibrate three Kinect cameras.

## Using this tool
The structure of the MKV recordings and names should be as following if there are 2 Subs:

-- recordings/
--- calibM.mkv
--- calibS1.mkv
--- calibS2.mkv
--- sub_folder/
---- M.mkv
---- S1.mkv
---- S2.mkv
--- sub_folder2/
...

The naming of the sub_folders doesnt matter. The Foldername to generate the pointcloud has to be named in the command with the flag --sub_path 