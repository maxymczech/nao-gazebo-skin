cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)
generate_messages(DEPENDENCIES std_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(contact SHARED ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_libraries})
