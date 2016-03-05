cmake_minimum_required(VERSION 2.8.3)
project(rapp_api_nao)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  rapp_ros_naoqi_wrappings
  rapp_core_agent
eigen_conversions
)

## System dependencies are found with CMake's conventions
find_package(Rapp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
##find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files( FILES
  Matrix4x4.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
  std_msgs 
  sensor_msgs
)

################################
## libraries names definition ##
################################
SET(COMMUNICATION_LIB rapp_robot_communication)
SET(VISION_LIB rapp_robot_vision)
SET(NAVIGATION_LIB rapp_robot_navigation)
SET(DYNAMIC_NAV_LIB rapp_dynamic_navigation)
SET(DYNAMIC_VIS_LIB rapp_dynamic_vision)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${RAPP_CXX_FLAGS})
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${COMMUNICATION_LIB}
  LIBRARIES ${VISION_LIB}
  LIBRARIES ${NAVIGATION_LIB}
  LIBRARIES ${DYNAMIC_NAV_LIB}
  LIBRARIES ${DYNAMIC_VIS_LIB}
  CATKIN_DEPENDS roscpp rapp_ros_naoqi_wrappings rapp_core_agent
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${RAPP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_directories(
	${RAPP_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${DYNAMIC_NAV_LIB}
  src/dynamic/navigation/localization.cpp
)

add_library(${COMMUNICATION_LIB}
  src/communication/communication.cpp src/communication/CommunicationImpl.cpp
)


add_library(${VISION_LIB}
  src/vision/vision.cpp src/vision/VisionImpl.cpp
)

add_library(${DYNAMIC_VIS_LIB}
  src/dynamic/vision/vision.cpp
)

add_library(${NAVIGATION_LIB}
  src/navigation/navigation.cpp src/navigation/NavigationImpl.cpp
)
target_link_libraries(${DYNAMIC_NAV_LIB}
  ${COMMUNICATION_LIB}
  ${VISION_LIB}
  ${NAVIGATION_LIB}
  ${catkin_LIBRARIES}
  ${RAPP_LIBRARIES_STATIC}
)
target_link_libraries(${DYNAMIC_VIS_LIB}
  ${COMMUNICATION_LIB}
  ${VISION_LIB}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RAPP_LIBRARIES_STATIC}
)
target_link_libraries(${COMMUNICATION_LIB}
  ${catkin_LIBRARIES}
  ${RAPP_LIBRARIES_STATIC}
)

target_link_libraries(${VISION_LIB}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RAPP_LIBRARIES_STATIC}
)
target_link_libraries(${NAVIGATION_LIB}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${RAPP_LIBRARIES_STATIC}
)
add_dependencies(${DYNAMIC_NAV_LIB} ${catkin_EXPORTED_TARGETS})
add_dependencies(${DYNAMIC_VIS_LIB} ${catkin_EXPORTED_TARGETS})
add_dependencies(${COMMUNICATION_LIB} ${catkin_EXPORTED_TARGETS})
add_dependencies(${VISION_LIB} ${catkin_EXPORTED_TARGETS})
add_dependencies(${NAVIGATION_LIB} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${VISION_LIB} /usr/lib/libzbar.so)
target_link_libraries(${DYNAMIC_VIS_LIB} /usr/lib/libzbar.so)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${COMMUNICATION_LIB} ${VISION_LIB} ${NAVIGATION_LIB} ${DYNAMIC_NAV_LIB} ${DYNAMIC_VIS_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rapp_api_nao.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
