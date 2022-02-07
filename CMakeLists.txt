cmake_minimum_required(VERSION 3.0.2)
project(hmap_on_board)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories((${PCL_INCLUDE_DIRS}))
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(hmap src/hmap.cpp)
target_link_libraries(hmap ${catkin_LIBRARIES} ${PCL_LIBRARIES})