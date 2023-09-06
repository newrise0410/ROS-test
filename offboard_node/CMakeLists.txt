cmake_minimum_required(VERSION 3.0.2)
project(offboard_node)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  eigen_conversions
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(offboard_cpp src/offboard.cpp)
add_executable(swarm_voxl2 src/swarm_multicopter.cpp)
target_link_libraries(offboard_cpp ${catkin_LIBRARIES})
target_link_libraries(swarm_voxl2 ${catkin_LIBRARIES})
