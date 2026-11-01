cmake_minimum_required(VERSION 3.10)
project(open_apero_msgs)

# Dependencies
set(MSG_DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${MSG_DEPENDENCIES}
)

# Interfaces
add_message_files(
  FILES
  CollectiblePoint.msg
  CollectiblePointArray.msg
  RobotState.msg
  CommandResult.msg
  RobotTrajectory.msg
)

generate_messages(
  DEPENDENCIES
  ${MSG_DEPENDENCIES}
)

# Package exports
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    ${MSG_DEPENDENCIES}
)
