cmake_minimum_required(VERSION 2.8.3)
project(libks_msgs)

find_package(catkin REQUIRED message_generation std_msgs sensor_msgs)

add_message_files(
  FILES
  ImageSet.msg
  MultiCameraImage.msg
  SharedImageSet.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs)

