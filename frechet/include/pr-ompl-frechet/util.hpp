#ifndef UTIL_
#define UTIL_

#include <Eigen/Dense>

#include <boost/graph/adjacency_list.hpp>

#include <ompl/base/Planner.h>

/// Boost Graph definitions.
struct VProp {
  // Name of this node. Mainly used to identify TPG nodes.
  std::string name;

  // Robot configuration.
  ompl::base::State *state;

  // End-effector pose.
  Eigen::Isometry3d poseEE;

  // The "weight" of a tensor-product node. Used to compute the minimum Frechet
  // path. Please see the papers linked from the README.
  double frechet;

  // Only set for a tensor-product node, and represents the node's NN Graph
  // component. Used to recover the final motion plan.
  // HACK: Assumes Vertex to be unsigned long int.
  unsigned long int nnComponent;
}; // struct VProp

struct EProp {
  // TPG Edge Weight.
  double length;

  // Evaluation marker for LazySP.
  bool evaluated;
}; // struct EProp

// Graph Description.
typedef boost::adjacency_list<boost::hash_setS, boost::vecS,
                              boost::bidirectionalS, VProp, EProp>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

// Vertex Maps
typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type
    VertexIndexMap;
typedef boost::property_map<Graph, std::string VProp::*>::type VPNameMap;
typedef boost::property_map<Graph, ompl::base::State * VProp::*>::type
    VPStateMap;
typedef boost::property_map<Graph, Eigen::Isometry3d VProp::*>::type
    VPPoseEEMap;
typedef boost::property_map<Graph, double VProp::*>::type VPFrechetMap;
typedef boost::property_map<Graph, Vertex VProp::*>::type VPNNComponentMap;

// Edge Maps
typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type
    EdgeIndexMap;
typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;
typedef boost::property_map<Graph, bool EProp::*>::type EPEvaluatedMap;

/// Interpolate \c N continuous values between min and max, inclusive.
///
/// \param[in] min Interpolation start value.
/// \param[in] max Interpolation end value.
/// \param[in] N Number of values to interpolate.
std::vector<double> linDoubleSpace(double min, double max, size_t N);

/// Interpolate \c N *integer* values between min and max, inclusive. Rounds if
/// needed.
///
/// \param[in] min Interpolation start value.
/// \param[in] max Interpolation end value.
/// \param[in] N Number of values to interpolate.
std::vector<int> linIntSpace(double min, double max, size_t N);

/// Flatten a 2D vector of nodes into a new 1D vector, but only return the nodes
/// between certain indices of the 2D vector.
///
/// \param[in] toFlatten Input 2D node vector.
/// \param[in] startingIndex Index to start flattening from.
/// \param[in] endingIndex Index to stop flattening from.
std::vector<Vertex>
flattenVertexList(std::vector<std::vector<Vertex>> &toFlatten,
                  int startingIndex, int endingIndex);

/// Find the k-nearest neighbors of a query node in a given vector of other
/// nodes, according to distance in C-space.
///
/// \param[in] queryVertex Node to find the kNN of.
/// \param[in] nodes Vector of kNN candidates.
/// \param[in] nnGraph Graph all nodes belong to.
/// \param[in] numK Number of NN to find.
/// \param[in] stateSpace OMPL statespace that nodes correspond to.
std::vector<Vertex> findKnnNodes(Vertex &queryVertex,
                                 std::vector<Vertex> &nodes, Graph &nnGraph,
                                 int numK,
                                 const ompl::base::StateSpacePtr stateSpace);

/// Return a vector of all nodes a graph contains.
///
/// \param[in] graph Graph to return the nodes of.
std::vector<Vertex> getGraphVertices(Graph &graph);

/// Return a the neighbors (successors) of a given node in a graph.
///
/// \param[in] v Node to return the successors of.
/// \param[in] g Graph node exists in.
std::vector<Vertex> getNeighbors(const Vertex &v, Graph &g);

/// Return a the predecessors of a given node in a graph.
///
/// \param[in] v Node to return the predecessors of.
/// \param[in] g Graph node exists in.
std::vector<Vertex> getPredecessors(const Vertex &v, Graph &g);

#endif // UTIL_HPP_
