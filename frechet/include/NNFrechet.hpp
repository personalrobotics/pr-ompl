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
    struct VProp
    {
      // To identify TPG nodes.
      std::string name;

      // Robot configuration.
      ompl::base::State* state;

      // End-effector pose.
      Eigen::Isometry3d poseEE;

      // For figuring weight of CPG nodes.
      double frechetDistance;
    }; // struct VProp

    struct EProp
    {
      // TPG Edge Weight.
      double length;

      // Evaluation marker for LazySP.
      bool evaluated;
    }; // struct EProp

    // Boost Graph definitions
    typedef boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, VProp, EProp> Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef boost::graph_traits<Graph>::edge_descriptor Edge;
    typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
    typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
    typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

    // Boost Graph Property Map definitions
    // Vertex Maps
    typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type VertexIndexMap;
    typedef boost::property_map<Graph, std::string VProp::*>::type VPNameMap;
    typedef boost::property_map<Graph, ompl::base::State* VProp::*>::type VPStateMap;
    typedef boost::property_map<Graph, Eigen::Isometry3d VProp::*>::type VPPoseEEMap;
    typedef boost::property_map<Graph, double VProp::*>::type VPFrechetDistanceMap;

    // Edge Maps
    typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;
    typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;
    typedef boost::property_map<Graph, double EProp::*>::type EPEvaluatedMap;

    // TODO.
  };

} // namespace LRAstar
#endif // NN_FRECHET_HPP_
