cmake_minimum_required(VERSION 2.8.3)
project(pi_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation 
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotorSetSpeed.msg
  MotorSetPosition.msg
  ButtonEvent.msg
  MotorInfo.msg
  Sensor3Axes.msg
  SensorStatusChange.msg
  #ApriltagPose.msg
  #AprilTagDetection.msg
  #AprilTagDetectionArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MotorGetPosition.srv
  GetInt32.srv
  SetInt32.srv
  GetMotorsInfo.srv
  SensorGet3Axes.srv
  GetPowerState.srv
  GetSensorInfo.srv
  GetString.srv
  SetString.srv
  GetStrings.srv
  #GetApriltagDetections.srv
)

generate_messages(
  #DEPENDENCIES
  #geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

