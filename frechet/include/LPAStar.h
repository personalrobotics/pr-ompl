#ifndef LPA_STAR_
#define LPA_STAR_

// TODO: Determine which of these are actually needed.
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <exception>
#include <queue>
#include <string>
#include <time.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/program_options.hpp>

#include "heap_indexed.h"
#include "util.hpp"

namespace po = boost::program_options;
using namespace pr_bgl;

typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

/// A class for *bottleneck* LPA*. NOTE that the graph *must* be a DAG for this
/// code to produce valid results.
class LPAStar {
  /// Min-heap that backs the priority queue.
  heap_indexed<double> mPQ;

  /// Start and target nodes of the search.
  Vertex mStartNode;
  Vertex mGoalNode;

  /// Distance map (see LPA* paper).
  std::unordered_map<Vertex, double> mDistance;
  /// Lookahead distance map (see LPA* paper).
  std::unordered_map<Vertex, double> mDistanceLookahead;
  /// Predecessor map (see LPA* paper).
  std::unordered_map<Vertex, Vertex> mPrev;

public:
  LPAStar() {}

  // NOTE: Methods related to PQ operations.

  /// Infinity value for marking collision edges.
  double mInfVal = std::numeric_limits<double>::max();

  /// Get the LPA* distance of a node.
  /// \param[in] v Node to get the distance of.
  double getDistance(Vertex &v);

  /// Get the LPA* lookahead distance of a node.
  /// \param[in] v Node to get the lookahead distance of.
  double getDistanceLookahead(Vertex &v);

  /// Update the priority of a node already on the PQ.
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to update the priority of.
  /// \param[in] newPriority New priority of the node.
  void updatePQ(Graph &g, Vertex &v, double newPriority);

  /// Insert a new node into the priority queue with given priority.
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to insert into the PQ.
  /// \param[in] priority Priority of the node.
  void insertPQ(Graph &g, Vertex &v, double priority);

  /// Delete a node from the priority queue.
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to delete from the PQ.
  void removePQ(Graph &g, Vertex &v);

  /// Check if the priority queue contains a given node.
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to check for in PQ.
  bool containsPQ(Graph &g, Vertex &v);

  /// Return true if the priority queue contains no nodes.
  bool isEmptyPQ();

  /// Get the samllest priority value in the queue.
  double peekPQ();

  /// Remove and return the node with the smallest priority in the PQ.
  /// \param[in] g Graph the search is on.
  Vertex popPQ(Graph &g);

  // NOTE: Core LPA* methods.

  /// Set up the core LPA* data structures.
  /// \param[in] g Graph the search is on.
  /// \param[in] start Start node of the search.
  /// \param[in] goal Target node of the search.
  void initLPA(Graph &g, Vertex &start, Vertex &goal);

  /// Compute the LPA* priority of a node.
  /// \param[in] node Node to compute LPA* priority of.
  double calculateKey(Vertex &node);

  /// This must be called called when u's dist and/or lookahead dist
  /// (and therefore consistency) may have been changed
  /// this ensures u is in the queue correctly
  /// \param[in] g Graph u belongs to.
  /// \param[in] u Node that must be updated.
  void updateVertex(Graph &g, Vertex &u);

  /// This is called to update vertex v due to either:
  /// - u_dist being updated OR
  /// - uv_weight being updated
  /// it will recalculate v's lookahead distance
  /// and return whether v's lookahead distance changed
  /// (if predecessor changed, but lookahead value didnt, return false)
  /// \param[in] g Graph u and v belong to.
  /// \param[in] u Predecessor of v that was updated.
  /// \param[in] v Node that may need to be updated.
  bool updatePredecessor(Graph &g, Vertex &u, Vertex &v);

  /// Return shortest path on graph.
  /// \param[in] g Graph the search is on.
  std::vector<Vertex> computeShortestPath(Graph &g);

  /// Helper method for the above. Follows predecessor map to recover the
  /// shortest path.
  std::vector<Vertex> followBackpointers();

}; // class LPAStar

#endif // LPA_STAR_
