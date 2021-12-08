cmake_minimum_required(VERSION 3.0.2)
project(livox_dedistortion_pkg)

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  livox_ros_driver
  eigen_conversions
  pcl_conversions
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
	    ${EIGEN3_INCLUDE_DIR}
	    ${PCL_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs 
  pcl_conversions pcl_ros
  eigen_conversions
  DEPENDS PCL 
  INCLUDE_DIRS include
)

add_executable(deskew_node src/livox_dedistortion.cpp src/data_process.cpp src/gyr_int.cpp)

target_link_libraries(deskew_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(livox_repub_node src/livox_repub.cpp)

target_link_libraries(livox_repub_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


