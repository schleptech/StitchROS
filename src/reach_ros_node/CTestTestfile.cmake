# CMake generated Testfile for 
# Source directory: /home/stitch0001/catkin_WS/src/reach_ros_node
# Build directory: /home/stitch0001/catkin_WS/src/reach_ros_node
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_reach_ros_node_roslint_package "/home/stitch0001/catkin_WS/src/reach_ros_node/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/melodic/share/catkin/cmake/test/run_tests.py" "/home/stitch0001/catkin_WS/src/reach_ros_node/test_results/reach_ros_node/roslint-reach_ros_node.xml" "--working-dir" "/home/stitch0001/catkin_WS/src/reach_ros_node" "--return-code" "/opt/ros/melodic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/stitch0001/catkin_WS/src/reach_ros_node/test_results/reach_ros_node/roslint-reach_ros_node.xml make roslint_reach_ros_node")
subdirs("gtest")
