cmake_minimum_required(VERSION 2.8.3)
project(mpcc_msgs)

find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

add_message_files(DIRECTORY msg
    FILES
    Input.msg
    State.msg
    Trajectory.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)
