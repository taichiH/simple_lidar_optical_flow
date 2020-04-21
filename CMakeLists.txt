cmake_minimum_required(VERSION 2.8.3)
project(simple_lidar_optical_flow)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  image_transport
  geometry_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf
  opencv_apps
  )

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_definitions(-std=c++14)

catkin_package(
  CATKIN_DEPENDS
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
)

add_executable(lidar_to_image src/lidar_to_image.cpp)
target_link_libraries(lidar_to_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
add_executable(flow_viewer src/flow_viewer.cpp)
target_link_libraries(flow_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
