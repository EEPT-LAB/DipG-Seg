cmake_minimum_required(VERSION 3.0.2)
project(dipgseg)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 -O3")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

find_package(PCL 1.2 REQUIRED)
find_package(Boost 1.50 REQUIRED)

catkin_package(
CATKIN_DEPENDS sensor_msgs
)

include_directories(
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  # include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kitti_loader
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(kitti_loader_lib
  src/kitti_loader/kitti_loader.cpp
)

find_package(OpenCV REQUIRED)
find_package(Boost 1.54 REQUIRED)

add_executable(offline_kitti_node src/offline_kitti_node.cpp)
target_link_libraries(offline_kitti_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} kitti_loader_lib)

add_executable(demo_node src/demo.cpp)
target_link_libraries(demo_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})