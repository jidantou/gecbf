cmake_minimum_required(VERSION 3.0.2)
project(gcbf_qp_solver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sphere_detector
  nav_msgs
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  nominal_controller
  # plan_env
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OsqpEigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gcbf_qp_solver
 CATKIN_DEPENDS roscpp quadrotor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(gcbf_lib
#   src/gcbf.cpp
# )

# target_link_libraries(gcbf_lib
#   ${catkin_LIBRARIES}
#   # plan_env
#   sphere_detector_lib
#   OsqpEigen::OsqpEigen
#   nominal_pd_controller
# )

# add_library(gcbf_lib_min
#   src/gcbf_min.cpp
# )

# target_link_libraries(gcbf_lib_min
#   ${catkin_LIBRARIES}
#   # plan_env
#   sphere_detector_lib
#   OsqpEigen::OsqpEigen
#   nominal_pd_controller
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(gcbf_qp_solver_node src/gcbf_qp_node.cpp)
# add_executable(gcbf_qp_solver_min_node src/gcbf_qp_node_min.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} plan_env)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} sphere_detector_lib)

## Specify libraries to link a library or executable target against
# target_link_libraries(gcbf_qp_solver_node
#   ${catkin_LIBRARIES}
#   gcbf_lib
#   OsqpEigen::OsqpEigen
# )

# target_link_libraries(gcbf_qp_solver_min_node
#   ${catkin_LIBRARIES}
#   gcbf_lib_min
#   OsqpEigen::OsqpEigen
# )
