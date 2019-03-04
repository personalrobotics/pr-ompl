#ifndef NN_FRECHET_HPP_
#define NN_FRECHET_HPP_

#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <exception>
#include <iostream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/program_options.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace NNFrechet {

  /// The OMPL Planner class that implements the algorithm
  class NNFrechet: public ompl::base::Planner
  {

  };

} // namespace LRAstar
#endif // NN_FRECHET_HPP_
