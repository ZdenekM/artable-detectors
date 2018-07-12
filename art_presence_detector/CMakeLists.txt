cmake_minimum_required(VERSION 2.8.3)
project(art_presence_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  art_msgs
  std_msgs
  sensor_msgs
  pcl_conversions
)



add_definitions(
  ${PCL_DEFINITIONS}
)

catkin_package(
  CATKIN_DEPENDS art_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
