cmake_minimum_required(VERSION 2.8.3)
project(pi_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation 
)

#catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ApriltagPose.msg
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetApriltagDetections.srv
  GetPredictions.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

