cmake_minimum_required(VERSION 2.8.3)
project(rapp_core_agent)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rapp_ros_naoqi_wrappings message_generation)

add_service_files(
	FILES
	DynamicAgentStatus.srv	
)


add_message_files(
  FILES
  Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime)




###########
## Build ##
###########


add_executable(core_agent src/core_agent.cpp)


target_link_libraries(core_agent ${catkin_LIBRARIES})
add_dependencies(core_agent ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############
install(TARGETS core_agent
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
