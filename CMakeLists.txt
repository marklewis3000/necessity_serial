cmake_minimum_required(VERSION 2.8.3)
project(pressure_serial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED system thread)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pressure_serial_msg.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  pressure_serial
# CATKIN_DEPENDS control_toolbox controller_manager hardware_interface  moveit_core 
  DEPENDS Boost 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})


## Declare a cpp library
add_library(pressure_serial
   src/SerialPort.cpp
   src/serial.cpp
 )
add_dependencies(pressure_serial pressure_serial_generate_messages_cpp)

## Declare a cpp executable
add_executable(serial_node node/serial_node.cpp)
target_link_libraries(serial_node 
  pressure_serial)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(xio_reader_node xio_reader_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pressure_serial
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

install(TARGETS serial_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(TARGETS pressure_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


