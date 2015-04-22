cmake_minimum_required(VERSION 2.8.3)
project(pr_ompl)

find_package(catkin REQUIRED)
find_package(OMPL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pr_ompl
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS OMPL
)

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_rrtconnect SHARED
  src/RRTConnect.cpp
)
target_link_libraries(${PROJECT_NAME}_rrtconnect
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
