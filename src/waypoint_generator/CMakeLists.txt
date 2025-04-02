cmake_minimum_required(VERSION 2.8.3)
project(waypoint_generator)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
)
catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_generator src/waypoint_generator.cpp)

target_link_libraries(waypoint_generator
   ${catkin_LIBRARIES}
)
