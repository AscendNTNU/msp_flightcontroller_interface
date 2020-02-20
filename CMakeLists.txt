cmake_minimum_required(VERSION 2.8.3)
project(msp_fc_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mav_msgs
  message_generation
)

add_message_files(
    FILES
    RcData.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS src
    CATKIN_DEPENDS roscpp message_runtime
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(msp_fc_interface src/msp_fc_interface.cpp src/serial.cpp)
target_link_libraries(msp_fc_interface
    ${catkin_LIBRARIES})
