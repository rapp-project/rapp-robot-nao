cmake_minimum_required(VERSION 2.8.3)
project(rapp_dynamic_agent)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
    
install(DIRECTORY scripts/
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############
