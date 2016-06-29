cmake_minimum_required(VERSION 2.8.3)
project(PID_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs tf roscpp)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate services in the 'srv' folder
add_service_files(
   FILES
   controlserver.srv
)

add_message_files(
   FILES
   motoroutputs.msg
   quadstate.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
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
catkin_package(CATKIN_DEPENDS geometry_msgs tf roscpp)


## Declare a C++ executable
add_executable(quadcontrol_server nodes/quadcontrol_server.cpp)
add_executable(quadcontrol_message nodes/quadcontrol_message.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(quadcontrol_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(quadcontrol_message ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(quadcontrol_server
   ${catkin_LIBRARIES}
)

target_link_libraries(quadcontrol_message
   ${catkin_LIBRARIES}
)
