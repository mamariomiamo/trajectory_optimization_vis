cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

add_subdirectory(include/trajectory_visualization/trajectory_lib)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_visualization
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_vis
  src/main.cpp
  )

target_link_libraries(trajectory_vis
  ${catkin_LIBRARIES}
  min_snap_traj_gen
)