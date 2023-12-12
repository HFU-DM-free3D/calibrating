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
