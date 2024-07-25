# Use Our Data to Test the Project

You are welcome to use our data to test our project. You can access a test data set [here](https://bwsyncandshare.kit.edu/s/etCYZpMDzjfYgza).

To execute the project, you can download the data set and execute the following commands, for example. Switch to the Branch "local-usage" to have access to the simple terminal tool without the need of the webapp Tool.

For using the charuco recording:
```python src/run.py --recordings_path [Path to Dataset]\standing2\charuco --mkvToolNix_path [Path to MKVToolNix] --amount_Subs 2 --use_charuco --icp_itteration 0 --amount_pcd_frames 5 --sub_path laura --kinect_pic_rec_extractor [Path to the MKVReader from Open3d (in your venv)]  --create_pcd_json ```

For using the aruco recording:
```python src/run.py --recordings_path [Path to Dataset]\standing2\aruco --mkvToolNix_path [Path to MKVToolNix] --amount_Subs 2 --icp_itteration 0 --amount_pcd_frames 5 --sub_path laura --kinect_pic_rec_extractor [Path to the MKVReader from Open3d (in your venv)]  --create_pcd_json --marker_length 0.15 ```