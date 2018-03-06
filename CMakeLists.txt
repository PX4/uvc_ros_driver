cmake_minimum_required(VERSION 2.8.3)
project(uvc_ros_driver)

# Check if Neon instructions are available.
include(cmake/FindARM.cmake)

if (NEON_FOUND)
  set(NEON_FLAGS "-DENABLE_NEON -mfloat-abi=hard -mfpu=neon-vfpv4 -mcpu=cortex-a15 -Ofast")
endif(NEON_FOUND)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -Wno-format-security ${NEON_FLAGS} -march=native -Wall -D__STRICT_ANSI__")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${NEON_FLAGS}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(libuvc REQUIRED)

find_package(PkgConfig)

find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
  message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
  set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
  set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
  message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
  pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  ait_ros_messages
  image_transport
)

## Generate messages
add_message_files(
  FILES
  calibration.msg
)
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uvc_ros_driver
)

###########
## Build ##
###########

include_directories(
  include
  ${YamlCpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${libuvc_INCLUDE_DIRS}
  /usr/include/libusb-1.0
  src/nodes/c_library
  src/nodes
)

## Declare a C++ library
add_library(uvc_ros_driver SHARED
  src/uvc_ros_driver.cpp
  src/serial_port.cpp
  src/stereo_homography.cpp
  src/fpga_calibration.cpp
)

target_link_libraries(uvc_ros_driver
  ${YamlCpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${libuvc_LIBRARIES}
)

## Declare a C++ executable
add_executable(uvc_ros_driver_node
	src/nodes/uvc_ros_driver_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(uvc_ros_driver_node
  uvc_ros_driver
  ${catkin_LIBRARIES}
)
