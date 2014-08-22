cmake_minimum_required(VERSION 2.8.3)
project(amigo_ref_interpolator)

find_package(catkin REQUIRED COMPONENTS
  amigo_msgs
  roscpp
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(interpolator_lib
  src/interpolator.cpp
)


