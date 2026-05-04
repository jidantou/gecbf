cmake_minimum_required(VERSION 3.0.2)
project(distance_logger)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(distance_logger_node src/distance_logger_node.cpp)

target_link_libraries(distance_logger_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)
