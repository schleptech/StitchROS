

set(realsense2_VERSION_MAJOR "@REALSENSE_VERSION_MAJOR@")
set(realsense2_VERSION_MINOR "@REALSENSE_VERSION_MINOR@")
set(realsense2_VERSION_PATCH "@REALSENSE_VERSION_PATCH@")

set(realsense2_VERSION ${realsense2_VERSION_MAJOR}.${realsense2_VERSION_MINOR}.${realsense2_VERSION_PATCH})

set_and_check(realsense2_INCLUDE_DIR "@PACKAGE_CMAKE_INSTALL_INCLUDEDIR@")

include("${CMAKE_CURRENT_LIST_DIR}/realsense2Targets.cmake")
set(realsense2_LIBRARY realsense2::realsense2)
