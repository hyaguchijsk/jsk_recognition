#!/bin/bash
echo "Please do:"
echo "demo/kenji/hironx/run.sh"
echo "hiro.servoOn(); go_wide_prepare_pose(); sync()"
echo "python model/get_model.py"
echo "*** if depth_world not defined, check if rostopic /my_camera/mat/xyz_world is published ***"
echo "this is probably because coordinate_transform is not compiled. add detect_rim to ROS_PACKAGE_PAHT and try catkin_make install in demo/kenji/catkin_ws"
ipython -i `rospack find clothbag_rim_detector`/scripts/main.py
