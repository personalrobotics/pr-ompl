cmake_minimum_required(VERSION 2.8.3)
project(or_pr_ompl)

# TODO: cmake_modules is only required to find Eigen
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  or_ompl
  pr_ompl
  openrave_catkin
)

# TODO: Eigen is a transitive dependency of or_ompl.
find_package(Eigen REQUIRED)
find_package(OMPL REQUIRED)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES pr_ompl
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS ompl or_ompl pr_ompl
)

include_directories(
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

openrave_plugin(${PROJECT_NAME}_plugin
  src/or_pr_ompl_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
