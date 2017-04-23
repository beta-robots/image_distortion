cmake_minimum_required(VERSION 2.8.3)
project(image_distortion)

add_definitions(-std=c++11)

# find depencencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    dynamic_reconfigure
    sensor_msgs
    image_transport 
    cv_bridge)

find_package(OpenCV REQUIRED)

# add include directories
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

## catkin specific configuration ##
catkin_package(
#   INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    dynamic_reconfigure
    sensor_msgs
    image_transport 
    cv_bridge)
    
## Build ##
add_executable(${PROJECT_NAME}
    src/image_distortion_node.cpp
    src/image_distortion_node_main.cpp)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})
