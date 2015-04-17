cmake_minimum_required(VERSION 2.8.3)
project(helloworld)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  rapp_libraries
)

catkin_package(CATKIN_DEPENDS roscpp rapp_libraries)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(helloworld src/helloworld.cpp)

target_link_libraries(helloworld ${catkin_LIBRARIES})

add_dependencies(helloworld ${catkin_EXPORTED_TARGETS})

install(TARGETS helloworld
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
