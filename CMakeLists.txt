cmake_minimum_required(VERSION 2.8.3)
project(intensity_cloud_map_pub)
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_FLAGS "-march=native -O2 -std=c++11 -fopenmp")
#set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS   "-march=native -O2 -fopenmp")
#set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "-march=native -O2 -fopenmp")
#set(CMAKE_EXE_LINKER_FLAGS "-march=native -O2 -fopenmp")

set(PCL_DIR "/usr/share/pcl-1.7/")

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
  PCL REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES intensity_cloud_map_pub
  CATKIN_DEPENDS laser_geometry 
  pcl_conversions 
  pcl_ros roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include/intensity_cloud_map_pub include/intensity_cloud_map_pub/map 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(intensity_cloud_map src/intensity_cloud_map_pub/map/map.cpp)
target_link_libraries(intensity_cloud_map)

add_executable(intensity_cloud_map_pub src/intensity_cloud_map_pub_node.cpp)
target_link_libraries(intensity_cloud_map_pub
  intensity_cloud_map
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES})

install( TARGETS
  intensity_cloud_map_pub intensity_cloud_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/intensity_cloud_map_pub/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
