cmake_minimum_required(VERSION 2.8.3)
project(husky)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav2d_navigator
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_sphere_detector
 CATKIN_DEPENDS roscpp geometry_msgs std_msgs nav2d_navigator
#  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(control2 src/control2.cpp)
target_link_libraries(control2 ${catkin_LIBRARIES})
