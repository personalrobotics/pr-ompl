#ifndef UTIL_
#define UTIL_

#include <Eigen/Dense>

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

/// Boost Graph definitions.
struct VProp
{
  // To identify TPG nodes.
  std::string name;

  // Robot configuration.
  ompl::base::State* state;

  // End-effector pose.
  Eigen::Isometry3d poseEE;

  // For figuring weight of TPG nodes.
  double frechet;

  // For recovering the final motion plan.
  // HACK: Assumes Vertex to be unsigned long int.
  unsigned long int nnComponent;

}; // struct VProp

struct EProp
{
  // TPG Edge Weight.
  double length;

  // Evaluation marker for LazySP.
  bool evaluated;
}; // struct EProp

// Graph Description.
typedef boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, VProp, EProp> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

// Vertex Maps
typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type VertexIndexMap;
typedef boost::property_map<Graph, std::string VProp::*>::type VPNameMap;
typedef boost::property_map<Graph, ompl::base::State* VProp::*>::type VPStateMap;
typedef boost::property_map<Graph, Eigen::Isometry3d VProp::*>::type VPPoseEEMap;
typedef boost::property_map<Graph, double VProp::*>::type VPFrechetMap;
typedef boost::property_map<Graph, Vertex VProp::*>::type VPNNComponentMap;

// Edge Maps
typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;
typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;
typedef boost::property_map<Graph, bool EProp::*>::type EPEvaluatedMap;


std::vector<double> linDoubleSpace(double min, double max, size_t N);

std::vector<int> linIntSpace(double min, double max, size_t N);

std::vector<Vertex> flattenVertexList(
  std::vector< std::vector<Vertex> >& toFlatten,
  int startingIndex,
  int endingIndex
);

std::vector<Vertex> findKnnNodes(
  Vertex& queryVertex,
  std::vector<Vertex>& nodes,
  Graph& nnGraph,
  int numK,
  const ompl::base::StateSpacePtr stateSpace
);

std::vector<Vertex> getGraphVertices(Graph& graph);

std::vector<Vertex> getNeighbors(const Vertex& v, Graph& g);

std::vector<Vertex> getPredecessors(const Vertex& v, Graph& g);

#endif // UTIL_HPP_
