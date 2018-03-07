cmake_minimum_required(VERSION 2.8.3)
project(uvc_ros_driver)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(libuvc REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${YamlCpp_INCLUDE_DIRS}
  ${libuvc_INCLUDE_DIRS}
  /usr/include/libusb-1.0
  src/nodes/c_library
  src/nodes
)

cs_add_library(uvc_ros_driver
  src/uvc_ros_driver.cpp
  src/serial_port.cpp
  src/stereo_homography.cpp
  src/fpga_calibration.cpp
)

cs_add_executable(uvc_ros_driver_node
  src/nodes/uvc_ros_driver_node.cpp
)

target_link_libraries(uvc_ros_driver
  ${YamlCpp_LIBRARIES}
  ${libuvc_LIBRARIES}
)

target_link_libraries(uvc_ros_driver_node
  uvc_ros_driver
)

cs_install()
cs_export()
