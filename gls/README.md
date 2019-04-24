# generalized_lazy_search

Framework implements various lazy search algorithms. The planners have been implemented in OMPL.

Dependencies:
- C++11 or higher
- cmake
- OMPL
- Boost Graph Library

The CMakeLists.txt file supports catkin tools. Once you have created and initialized your workspace, 
you should be able to build the package by running `catkin build gls`.

The planner implemented under GLS returns the shortest path on the roadmap graph it is planning on.
