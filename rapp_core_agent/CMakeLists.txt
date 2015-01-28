cmake_minimum_required(VERSION 2.8.3)
project(rapp_core_agent)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	cv_bridge
)


 add_service_files(
   FILES
   Say.srv
   GetEmailAddress.srv
   Record.srv
   SendEmail.srv
   DetectQRcodes.srv
	GetImage.srv
	PhotoCapture.srv
 )

 add_message_files(
	FILES
	Matrix4x4.msg
	#Matrix4x4[].msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
	CATKIN_DEPENDS message_runtime
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
    
install(DIRECTORY nodes/
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############
