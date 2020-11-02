cmake_minimum_required(VERSION 2.8.3)
project(jackal_car_control)

find_package(catkin REQUIRED COMPONENTS roslaunch 
  roscpp 
  rospy
  sensor_msgs
  serial
  std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

roslaunch_add_file_check(launch/jackal_car_control.launch)
roslaunch_add_file_check(launch/teleop.launch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_executable(reJointStateArduino_node src/reJointStateArduino.cpp)
target_link_libraries(reJointStateArduino_node  ${catkin_LIBRARIES})
add_dependencies(reJointStateArduino_node reJointStateArduino_messages_cpp)

add_executable(turtle_teleop_joy src/joy_twitst.cpp)
target_link_libraries(turtle_teleop_joy ${catkin_LIBRARIES})
