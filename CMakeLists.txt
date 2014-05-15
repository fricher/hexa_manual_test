cmake_minimum_required(VERSION 2.8.3)
project(hexa_manual_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamixel_control
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hexa_keyboard src/hexa_keyboard.cpp)
target_link_libraries(hexa_keyboard ${catkin_LIBRARIES})
