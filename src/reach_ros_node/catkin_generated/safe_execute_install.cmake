execute_process(COMMAND "/home/stitch0001/catkin_WS/src/reach_ros_node/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/stitch0001/catkin_WS/src/reach_ros_node/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
