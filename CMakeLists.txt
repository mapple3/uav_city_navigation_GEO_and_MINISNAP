cmake_minimum_required(VERSION 3.0.2)
project(uav_city_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  mavros_msgs
  tf
  tf2
  tf2_ros
  dynamic_reconfigure
  quadrotor_msgs
  controller_msgs
)

find_package(Eigen3 REQUIRED)

# This generates the header file you were missing
generate_dynamic_reconfigure_options(
  cfg/GeometricController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs visualization_msgs mavros_msgs tf tf2 tf2_ros dynamic_reconfigure quadrotor_msgs controller_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## 1. Geometric Controller Node
# Add ALL the supporting .cpp files, including the one with the main() function
add_executable(geometric_controller_node 
  src/geometric_controller_node.cpp 
  src/geometric_controller.cpp
  src/nonlinear_geometric_control.cpp
  src/nonlinear_attitude_control.cpp
  src/jerk_tracking_control.cpp
)
add_dependencies(geometric_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometric_controller_node ${catkin_LIBRARIES})

## 2. Trajectory Generator Node
# Added src/trajectory_generator_waypoint.cpp to fix "undefined reference"
add_executable(trajectory_generator_node 
  src/trajectory_generator_node.cpp 
  src/trajectory_generator_waypoint.cpp
)
add_dependencies(trajectory_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_generator_node ${catkin_LIBRARIES})

## 3. Waypoint Generator Node
add_executable(waypoint_generator_node src/waypoint_generator.cpp)
add_dependencies(waypoint_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_generator_node ${catkin_LIBRARIES})

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/rviz_city_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)