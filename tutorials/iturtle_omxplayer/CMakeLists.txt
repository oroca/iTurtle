cmake_minimum_required(VERSION 2.8.3)
project(iturtle_omxplayer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  roslib
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp std_msgs roslib
)



###########
## Build ##
###########

# special directories
set(CMAKE_CXX_FLAGS "-DCATKIN_PROJECT_NAME='\"${PROJECT_NAME}\"' -DCATKIN_PACKAGE_SHARE_DESTINATION='\"${CATKIN_PACKAGE_SHARE_DESTINATION}\"' ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/omxplayer.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# I don't know how to get the abspath of 'CATKIN_PACKAGE_SHARE_DESTINATION' but path to the package
# CATKIN_PACKAGE_SHARE_DESTINATION only contains part of its abspath
# `share/iturtle_omxplayer`
install(FILES
  launch/omxplayer.launch
  #mp3/hello.m4a
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  mp3/hello.m4a
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
