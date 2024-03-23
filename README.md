# How to start the Repo
1. Clone the repo
2. Create a venv ( python -m venv venv)
3. Check Restrictions (on Windows --> cmd or powershell: Get-ExecutionPolicy). If they are restricted you have to unrestrict them (cmd: Set-ExecutionPolicy Unrestricted -Scope Process)
4. Activate venv (cmd: venv/Scripts/activate)
5. install numpy (cmd: pip install numpy)
6. install opencv (cmd: pip install opencv-contrib-python)
7. install matplotlib (cmd: pip install matplotlib)

# calibrating
This project centers on calibrating three Kinect cameras for Free3D, employing Aruco markers. This calibration ensures precise alignment, vital for accurate 3D reconstructions, enhancing overall project quality and effectiveness.

## Project Overview

### System Requirements
| Component   | Version   |
|-------------|-----------|
| Python      | 3.11      |
| OpenCV      | 4.8.1     |
| matplotlib      | 3.8.2     |

### Tested on
- Windows 11
- Nvidia GTX 1670Ti

## About
This repository is created in conjunction with the Free3D project at the Faculty of DM, Hochschule Furtwangen. Its purpose is to calibrate three Kinect cameras.

## Calibration Data Extraction

### Using ffmpeg
To extract calibration data from an mkv file:
```bash
ffmpeg -dump_attachment:3 calibration2.json -i output-4.mkv
```

###Using MKVToolNix
```bash
mkvtoolNix: output-4.mkv attachments 1:calibration_output4.json
```

## Using this tool
This is a Project in development. In the src/tools path you can get runable scripts that will do small tasks like creating a aruco Marker on a DinA4-Page or tracking a Aruco-Marker in a Video.
The Main Programm isn't running yet. It is planned that you can give a mkv-File to the Programm with the following structure:
| Track       | Type        | Name             | Camera    |
|-------------|-------------|------------------|-----------|
| Track 1     | Video       | Color            | 1         |
| Track 2     | Video       | Depth            | 1         |
| Track 3     | Video       | IR               | 1         |
|-------------|-------------|------------------|-----------|
| Track 4     | Video       | Color            | 2         |
| Track 5     | Video       | Depth            | 2         |
| Track 6     | Video       | IR               | 2         |
|-------------|-------------|------------------|-----------|
| Track 7     | Video       | Color            | 3         |
| Track 8     | Video       | Depth            | 3         |
| Track 9     | Video       | IR               | 3         |
|-------------|-------------|------------------|-----------|
|             | Attatchment | Calibration Json | 1         |
|             | Attatchment | Calibration Json | 2         |
|             | Attatchment | Calibration Json | 3         |

The File should be generated with the "recording"-Repository.

This tool then records a view seconds of a scene with aruco-Markers, so the cameras can be calibrated. Out of this recording (with the recording repository so this mkv-Structure from above can be generated) we can create with this repository a "MultipleCamCalibration.json" in witch we safe the position and rotation of each camera to eachother.
When the "calibrating video" with the marker is done the cameras should not be moved before and while the true recording.