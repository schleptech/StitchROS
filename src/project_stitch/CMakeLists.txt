cmake_minimum_required(VERSION 2.8)
project( project_stitch )

set(Spinnaker_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")



find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package( OpenCV 3 REQUIRED )
find_package( OpenGL REQUIRED)
find_package( catkin REQUIRED COMPONENTS roscpp std_msgs genmsg xmlrpcpp image_transport cv_bridge message_filters)
#find_package(GLFW 3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#catkin_package()

#include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable( PCPublisher src/pcpublisher/PointCloudPublisher.cpp)
#target_link_libraries(PCPublisher PRIVATE ${catkin_LIBRARIES} ouster_client ${PCL_LIBRARIES} )

add_executable( Subscriber src/subscriber/subscriber.cpp)
target_link_libraries(Subscriber ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable( PubKill src/PubKill/PubKill.cpp)

add_executable( RunRecorder src/RunRecorder/RunRecorder.cpp)