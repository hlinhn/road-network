cmake_minimum_required(VERSION 3.1)
project(road_network)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_CATKIN_DEPS
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPS}
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
set(YAML_CPP_LIBS yaml-cpp)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PROJECT_CATKIN_DEPS}
  DEPENDS
    OpenCV
)

add_definitions(${catkin_DEFINITIONS})

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
  SYSTEM ${catkin_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}_lib
  src/road_network/helper.cpp
  src/road_network/detect_route_graph.cpp
  )

add_executable(
  route_graph
  src/road_network/main.cpp
  )

target_link_libraries(
  route_graph
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBS}
  ${OpenCV_LIBRARIES}
  )

########################################################
# Installs
########################################################

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
