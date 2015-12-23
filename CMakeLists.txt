cmake_minimum_required(VERSION 2.8.3)
project(davinci_comm)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 std_msgs
 message_generation
) 

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  imu.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES davinci_comm
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
 DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)

## Declare a C++ executable
add_executable(davinci_comm_node src/davinci_comm_node.cpp src/serial_port.cpp src/qbot_comm.cpp src/qbot_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(davinci_comm_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

