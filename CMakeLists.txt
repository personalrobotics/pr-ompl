cmake_minimum_required(VERSION 2.8.3)
project(pr-ompl)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
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
  src/LRAstar.cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#================================================================================
# Installation

  install(TARGETS pr-ompl EXPORT pr-omplConfig
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

  install(DIRECTORY include/ DESTINATION include)

  # This makes the project importable from the install directory
  install(EXPORT pr-omplConfig DESTINATION share/pr-ompl/cmake)

  # This makes the project importable from the build directory
  export(TARGETS pr-ompl FILE pr-omplConfig.cmake)

  # Install the package.xml file (to satisfy REP-136).
  install(FILES "package.xml"
    DESTINATION "share/pr-ompl")

# This custom target doesn't affect on building aikido projects but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE aikido_headers "include/*.hpp")
add_custom_target(headers SOURCES ${aikido_headers})
