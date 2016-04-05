cmake_minimum_required(VERSION 2.8.3)
project(kinova_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kinova_driver
  kinova_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  kdl_parser
  roscpp
  rospy
  dmp
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY
    action
   FILES
    Force.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs sensor_msgs actionlib_msgs

)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinova_demo
  CATKIN_DEPENDS kinova_driver kinova_msgs roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
message("${Eigen_INCLUDE_DIRS}")

add_executable(gravity_compensation src/kdl_gravity_comp.cpp)
target_link_libraries(gravity_compensation ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/kinova_demo/cartesian_workout.py
  nodes/kinova_demo/grip_workout.py
  nodes/kinova_demo/joint_angle_workout.py
  nodes/handshake_control/force_control.py
  nodes/handshake_control/force_server.py
  nodes/handshake_control/force_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
