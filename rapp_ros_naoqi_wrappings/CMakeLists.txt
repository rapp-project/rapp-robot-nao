cmake_minimum_required(VERSION 2.8.3)
project(rapp_ros_naoqi_wrappings)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	cv_bridge
	sensor_msgs
	image_transport
)

 add_service_files(
   FILES
### COMMUNICATION SERVICES
   Say.srv
	MicrophoneEnergy.srv
   RecognizeWord.srv
   GetEmailAddress.srv
   Record.srv
   SendEmail.srv

###  VISION SERVICES 
   #DetectQRcodes.srv
   #DetectHazardByQRcodes.srv
   GetImage.srv
	SetCameraParam.srv
   GetTransform.srv
   #PhotoCapture.srv
   #LearnFace.srv
   #RecognizeFaces.srv

###  MOVE & NAVIGATION SERVICES
   #FollowMarker.srv
   #MoveTo.srv
   #MoveVel.srv
   #MoveHead.srv
   #MoveStop.srv
   #GetPlan.srv
   #GetPose.srv
   #UpdatePose.srv
   #MoveGetCollisionStatus.srv	
 )

 add_message_files(
	FILES
	Matrix4x4.msg
	#MatrixRot.msg 
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
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
