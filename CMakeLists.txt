cmake_minimum_required(VERSION 2.8.3)
project(map_merging)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  nav_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  nav_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  
)

add_executable(elevation_map_publisher src/elevation_map_publisher.cpp)
target_link_libraries(elevation_map_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(plain_map_publisher src/plain_map_publisher.cpp)
target_link_libraries(plain_map_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
