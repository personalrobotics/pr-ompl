cmake_minimum_required(VERSION 2.8.3)
project(pr_ompl)

find_package(catkin REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pr_ompl
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS ompl
)

include_directories(include)

add_library(${PROJECT_NAME}_rrtconnect SHARED src/RRTConnect.cpp)
