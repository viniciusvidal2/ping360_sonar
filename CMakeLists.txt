cmake_minimum_required(VERSION 3.0.2)
project(ping360_sonar)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
# add_compile_options(-std=c++14)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
  roslint
)
find_package(OpenCV REQUIRED)

# catkin_python_setup()
# roslint_python()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SonarEcho.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/sonar.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sonar-ping360
 CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

# add_executable(my_node src/ping360_sonar/listener.cpp)
# add_dependencies(my_node ${catkin_EXPORTED_TARGETS})
# TARGET_LINK_LIBRARIES(my_node ${catkin_LIBRARIES})

install(PROGRAMS
     nodes/ping360_node
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
