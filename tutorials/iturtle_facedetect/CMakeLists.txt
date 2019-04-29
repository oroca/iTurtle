cmake_minimum_required(VERSION 2.8.3)
project(iturtle_facedetect)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
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
CATKIN_DEPENDS message_runtime rospy std_msgs
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

catkin_install_python(PROGRAMS
  src/facedetect.py
  src/facedetect_listener.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  launch/facedetect.launch
  src/haarcascade_frontalface_alt.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

