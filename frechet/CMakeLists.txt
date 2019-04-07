cmake_minimum_required(VERSION 2.8.3)
project(pr-ompl-frechet)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Eigen3_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/util.cpp
  src/LPAStar.cpp
  src/NNFrechet.cpp
)

#================================================================================
# Installation

  install(TARGETS pr-ompl-frechet EXPORT pr-ompl-frechetConfig
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  install(DIRECTORY include/ DESTINATION include/pr_ompl_frechet)

  # This makes the project importable from the install directory
  install(EXPORT pr-ompl-frechetConfig DESTINATION share/pr-ompl-frechet/cmake)

  # This makes the project importable from the build directory
  export(TARGETS pr-ompl-frechet FILE pr-ompl-frechetConfig.cmake)

  # Install the package.xml file (to satisfy REP-136).
  install(FILES "package.xml"
    DESTINATION "share/pr-ompl-frechet")
