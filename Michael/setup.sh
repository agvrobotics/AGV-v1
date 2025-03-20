#--------Python Libraries (For AI & Object Detection)------
pip install opencv-python opencv-contrib-python
pip install ultralytics
pip install depthai
pip install deep-sort-realtime
pip install supervision

ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
ros2 param set /v4l2_camera pixel_format yuv422_yuy2
