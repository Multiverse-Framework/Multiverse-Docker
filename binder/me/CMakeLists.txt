cmake_minimum_required(VERSION 3.0.2)
project(me)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND rm -rf Multiverse
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND mkdir -p Multiverse/giskard_ws/src
)

execute_process(
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMAND ln -s ${CMAKE_CURRENT_SOURCE_DIR}/../giskardpy Multiverse/giskard_ws/src/giskardpy
)

