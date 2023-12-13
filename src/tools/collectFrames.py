#This is not Ready for Use (12.12.23 lw)

import subprocess
import json
import numpy as np

def videoInfo(filename):
    proc = subprocess.run([
        "ffprobe", "-v", "quiet", "-print_format", "json", "-show_format", "-show_streams", filename
    ], capture_output=True)
    proc.check_returncode()
    return json.loads(proc.stdout)

def readVideo(filename):
    cmd = [
        "ffmpeg", "-i", filename,
        "-map", "0:0",
        "-map", "0:1",
        "-analyzeduration", "2147483647",  # Maximum 32-bit signed integer value
        "-probesize", "2147483647",        # Maximum 32-bit signed integer value
        "-f", "rawvideo", "-pix_fmt", "rgb24", "-"
    ]
    shape = np.array([2, 1080, 1920, 3])  # Assuming both streams have the same resolution
    with subprocess.Popen(cmd, stdout=subprocess.PIPE) as proc:
        while True:
            data = proc.stdout.read(shape.prod())
            if not data:
                return
            yield np.frombuffer(data, dtype=np.uint8).reshape(shape)

if __name__ == "__main__":
    # Replace 'your_video_file.mkv' with the actual path to your video file
    input_video_path = 'E:/Studium/Master/Semester_1/ForschProj/testVids/output-2.mkv'

    # Iterate over frames from color and depth streams
    for frames in readVideo(input_video_path):
        # Process frames as needed
        # 'frames[0]' corresponds to the color stream
        # 'frames[1]' corresponds to the depth stream
        # Example: print the shape of the color frames
        print("Color Stream Shape:", frames[0].shape)
        # Example: print the shape of the depth frames
        print("Depth Stream Shape:", frames[1].shape)
        break  # You can remove this line to process all frames