#ifndef LPA_STAR_
#define LPA_STAR_

#include <unordered_map>
#include <vector>

#include "MinPriorityQueue.h"
#include "util.hpp"

using namespace pr_bgl;

typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

/// A class for *bottleneck* LPA*. NOTE that the graph *must* be a DAG for this
/// code to produce valid results.
class LPAStar {
  /// Our fringe data structure.
  MinPriorityQueue mPQ;

  /// Start and goal nodes of the search.
  Vertex mStartNode;
  Vertex mGoalNode;

  /// Distance map (see LPA* paper).
  std::unordered_map<Vertex, double> mDistanceMap;

  /// Lookahead distance map (see LPA* paper).
  std::unordered_map<Vertex, double> mDistanceLookaheadMap;

  /// Predecessor map (see LPA* paper).
  std::unordered_map<Vertex, Vertex> mPrevMap;

  // NOTE: Methods related to PQ operations.

  /// Get the LPA* distance of a node.
  ///
  /// \param[in] v Node to get the distance of.
  double getDistance(Vertex &v);

  /// Get the LPA* lookahead distance of a node.
  ///
  /// \param[in] v Node to get the lookahead distance of.
  double getDistanceLookahead(Vertex &v);

  // NOTE: Helpers used during search.

  /// Compute the LPA* priority of a node.
  ///
  /// \param[in] node Node to compute LPA* priority of.
  double calculateKey(Vertex &node);

  /// Helper method for the above. Follows predecessor map to recover the
  /// shortest path.
  std::vector<Vertex> followBackpointers();

public:
  LPAStar() {}

  /// Infinity value for marking collision edges.
  double mInfVal = std::numeric_limits<double>::max();

  // NOTE: Core LPA* methods.

  /// Set up the core LPA* data structures.
  ///
  /// \param[in] g Graph the search is on.
  /// \param[in] start Start node of the search.
  /// \param[in] goal Target node of the search.
  void initLPA(Graph &g, Vertex &start, Vertex &goal);

  /// This must be called when u's dist and/or lookahead dist (and therefore
  /// consistency) may have been changed. This ensures u is in the queue
  /// correctly.
  ///
  /// \param[in] g Graph u belongs to.
  /// \param[in] u Node that must be updated.
  void updateVertex(Graph &g, Vertex &u);

  /// This is called to update vertex v due to either:
  /// - u_dist being updated OR
  /// - uv_weight being updated
  /// it will recalculate v's lookahead distance
  /// and return whether v's lookahead distance changed
  /// (if predecessor changed, but lookahead value didnt, return false)
  ///
  /// \param[in] g Graph u and v belong to.
  /// \param[in] u Predecessor of v that was updated.
  /// \param[in] v Node that may need to be updated.
  bool updatePredecessor(Graph &g, Vertex &u, Vertex &v);

  /// Return shortest path on graph.
  ///
  /// \param[in] g Graph the search is on.
  std::vector<Vertex> computeShortestPath(Graph &g);
}; // class LPAStar

#endif // LPA_STAR_
