# bin/bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install&&
echo "BUILD COMPLETED..." &&
#echo &&
echo "running..." &&
source ./install/setup.bash &&
#ros2 run camera_calibration camera_calibration_node_exe # ros2 run <package> <executable>
ros2 launch camera_calibration camera_calibration.launch.py

