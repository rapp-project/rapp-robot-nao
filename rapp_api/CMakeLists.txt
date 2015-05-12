cmake_minimum_required(VERSION 2.8.3)
project(rapp_api)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  rapp_ros_naoqi_wrappings
  rapp_core_agent
)

find_package(vmime)
find_package(OpenCV REQUIRED)

add_message_files(FILES Matrix4x4.msg)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES NaoCommunication NaoVision #NaoNavigation  
  CATKIN_DEPENDS roscpp rapp_ros_naoqi_wrappings rapp_core_agent
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

#add_library(NaoNavigation SHARED src/NaoNavigation.cpp)
add_library(NaoVision SHARED src/NaoVision.cpp)
add_library(NaoCommunication SHARED src/NaoCommunication.cpp src/NaoSendEmail.cpp src/NaoDAStatus.cpp)
add_library(NaoDAStatus SHARED src/NaoDAStatus.cpp)

target_link_libraries(NaoVision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#target_link_libraries(NaoNavigation 
#  ${catkin_LIBRARIES}
# )

target_link_libraries(NaoCommunication
  ${catkin_LIBRARIES}
  ${vmime_LIBRARIES}
)

target_link_libraries(NaoDAStatus
  ${catkin_LIBRARIES}
)

add_dependencies(NaoCommunication ${catkin_EXPORTED_TARGETS})
add_dependencies(NaoDAStatus ${catkin_EXPORTED_TARGETS}) 
#add_dependencies(NaoNavigation ${catkin_EXPORTED_TARGETS})
add_dependencies(NaoVision ${catkin_EXPORTED_TARGETS})

target_link_libraries(NaoVision /usr/lib/libzbar.so)

#############
## Install ##
#############

install(TARGETS NaoCommunication NaoVision NaoDAStatus #NaoNavigation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )

