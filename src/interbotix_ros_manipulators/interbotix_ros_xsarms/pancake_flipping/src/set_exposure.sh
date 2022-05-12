#!/bin/bash
rosrun dynamic_reconfigure dynparam set /camera1/rgb_camera exposure 40
rosrun dynamic_reconfigure dynparam set /camera2/rgb_camera exposure 40
rosrun dynamic_reconfigure dynparam set /camera3/rgb_camera exposure 40