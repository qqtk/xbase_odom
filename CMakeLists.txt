cmake_minimum_required(VERSION 2.8.3)
project(xbase_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  sensor_msgs
  robbase_msg
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

## Generate messages in the 'msg' folder 're' use robbase_msg ROSpkg
# add_message_files(
#  DIRECTORY
#  msg
#  FILES
#  WheelSpeed.msg
#  Encoder.msg
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp message_runtime std_msgs geometry_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(xbase_odom src/xbase_odom.cpp)
target_link_libraries(xbase_odom
  ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${Boost_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(xbase_odom xbase_odom_gencpp)
# add_dependencies(xbase_odom xbase_odom_generate_messages_cpp)

# see' https://github.com/wjwwood/catkin_demos/tree/ros_answers_52744

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS xbase_odom
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
