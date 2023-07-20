cmake_minimum_required(VERSION 3.0.2)
project(gazebo_ros_optical_flow)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  gazebo_dev
  gazebo_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  image_transport
  cv_bridge
  gazebo_ros_optical_flow_msgs
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV COMPONENTS core highgui REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES gazebo_ros_optical_flow
   CATKIN_DEPENDS gazebo_ros roscpp std_msgs message_runtime gazebo_plugins
#  DEPENDS system_lib
)

add_subdirectory( external/PX4-OpticalFlow OpticalFlow )
set( OpticalFlow_LIBS "OpticalFlow" )

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpticalFlow_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/gazebo_ros_optical_flow.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpticalFlow_LIBS})