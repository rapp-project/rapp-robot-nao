cmake_minimum_required(VERSION 2.8.3)
project(cloud_services)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
)

add_service_files(
  FILES
  FindObjects.srv
)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(find_objects_node src/find_objects_node.cpp src/find_objects.cpp)

target_link_libraries(find_objects_node ${catkin_LIBRARIES})

add_dependencies(find_objects_node ${PROJECT_NAME}_generate_messages_cpp)


#############
## Install ##
#############

install(TARGETS find_objects_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch 
)

#############
## Testing ##
#############
