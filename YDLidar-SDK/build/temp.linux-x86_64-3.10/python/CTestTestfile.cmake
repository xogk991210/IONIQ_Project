# CMake generated Testfile for 
# Source directory: /home/kth/ros2_ws/src/YDLidar-SDK/python
# Build directory: /home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/python
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(ydlidar_py_test "/usr/bin/python3" "/home/kth/ros2_ws/src/YDLidar-SDK/python/test/pytest.py")
set_tests_properties(ydlidar_py_test PROPERTIES  ENVIRONMENT "PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/kth/ros2_ws/src/YDLidar-SDK/build/temp.linux-x86_64-3.10/python" _BACKTRACE_TRIPLES "/home/kth/ros2_ws/src/YDLidar-SDK/python/CMakeLists.txt;42;add_test;/home/kth/ros2_ws/src/YDLidar-SDK/python/CMakeLists.txt;0;")
subdirs("examples")
