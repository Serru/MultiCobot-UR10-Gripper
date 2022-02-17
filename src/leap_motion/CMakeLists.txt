cmake_minimum_required(VERSION 2.8.3)
project(leap_motion)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  camera_calibration_parsers 
  geometry_msgs
  message_generation
  visualization_msgs

  # For backwards compatibility with the old driver files
  roslib
  rospy
  sensor_msgs
)

find_package(OpenMP)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Arm.msg
  Bone.msg
  Finger.msg
  Gesture.msg
  Hand.msg
  Human.msg

  # For backwards compatibility with the old driver files
  leap.msg
  leapros.msg
  leapcobotright.msg
  leapcobotleft.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

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
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  roscpp
  std_msgs
  visualization_msgs

  # For backwards compatibility with the old driver files
  rospy
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/LeapSDK/include/
)

# If you want to use this on a 32-bit machine, here's the place to change
set(LIB_LEAP ${PROJECT_SOURCE_DIR}/LeapSDK/lib/x64/libLeap.so)
#set(LIB_LEAP ${PROJECT_SOURCE_DIR}/LeapSDK/lib/x86/libLeap.so)

## Declare a C++ library
add_library(lmc_listener 
  src/lmc_listener.cpp
)
add_dependencies(lmc_listener 
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(lmc_listener
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/LeapSDK/include/Leap.h
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# The executable node that talks to the Leap Motion controller
add_executable( ${PROJECT_NAME}_driver_node 
  src/lmc_driver_node.cpp
)
add_dependencies( ${PROJECT_NAME}_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_driver_node
  ${catkin_LIBRARIES}
  lmc_listener
  ${LIB_LEAP}
)

# Node that visualizes the data it gets from the controller
add_executable(${PROJECT_NAME}_visualizer_node 
  src/lmc_visualizer_node.cpp
)

add_dependencies(${PROJECT_NAME}_visualizer_node  
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}_visualizer_node  
  ${catkin_LIBRARIES}
  ${LIB_LEAP}
)

# Node that publishes raw camera images. Be sure to 
# have "Allow images" enabled in LeapControlPanel!
add_executable(${PROJECT_NAME}_camera_node 
  src/lmc_camera_node.cpp
)
target_link_libraries(${PROJECT_NAME}_camera_node
  ${catkin_LIBRARIES} 
  ${LIB_LEAP}
)

# Filter node implementing a 2nd-order Butterworth lowpass filter
add_executable(${PROJECT_NAME}_filter_node 
  src/lmc_filter_node.cpp
)
add_dependencies(${PROJECT_NAME}_filter_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_filter_node
  ${catkin_LIBRARIES} 
  ${LIB_LEAP}
)

#########################################################
# For backwards compatibility with the old driver files #
#                     DEPRECATED                        #
#########################################################

add_executable(leap_hands 
  src/leap_hands.cpp
)
target_link_libraries(leap_hands
  ${catkin_LIBRARIES} ${LIB_LEAP}
)

add_executable(leap_camera src/leap_camera.cpp)
target_link_libraries(leap_camera
  ${catkin_LIBRARIES} ${LIB_LEAP}
)

install(TARGETS leap_camera leap_hands
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/leap_interface.py scripts/sender.py scripts/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch
  config/camera_info
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)