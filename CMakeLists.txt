cmake_minimum_required(VERSION 3.0.2)
project(amigo_ref_interpolator)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  amigo_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES interpolator_lib
  CATKIN_DEPENDS amigo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(interpolator_lib
  src/interpolator.cpp
)
target_link_libraries(interpolator_lib
  ${catkin_LIBRARIES}
)

add_dependencies(interpolator_lib ${catkin_EXPORTED_TARGETS})
