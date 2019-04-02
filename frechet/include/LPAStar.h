#ifndef LPA_STAR_
#define LPA_STAR_

//TODO: Determine which of these are actually needed.
#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <exception>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <time.h>

#include <boost/program_options.hpp>

#include "util.h"
#include "heap_indexed.h"

namespace po = boost::program_options;
using namespace pr_bgl;

// TODO: Need this?
// typedef boost::property_map<Graph, boost::vertex_index_t>::type vertexIndexMap;

/// A class for *bottleneck* LPA*. NOTE that the graph must be a DAG for this
/// code to produce valid results.
class LPAStar
{
  heap_indexed<double> mPQ;

  Vertex mStartNode;
  Vertex mGoalNode;

  std::unordered_map< Vertex, double > mDistance;
  std::unordered_map< Vertex, double > mDistanceLookahead;
  std::unordered_map< Vertex, Vertex > mPrev;

  public:
    // For marking collision edges.
    double mInfVal = std::numeric_limits<double>::max();

    double getDistance(Vertex& v);

    double getDistanceLookahead(Vertex& v);

    void updatePQ(Graph& g, Vertex& v, double newPriority);

    void insertPQ(Graph& g, Vertex& v, double priority);

    void removePQ(Graph& g, Vertex& v);

    bool containsPQ(Graph& g, Vertex& v);

    bool isEmptyPQ();

    // Get the samllest priority in the queue.
    double peekPQ();

    // We use a min heap, so this should be the Vertex with the smallest
    // priority.
    Vertex popPQ(Graph& g);

    // NOTE: Actual LPA* methods.

    // Set up the data structures with original nodes and edges.
    void initLPA(
      Graph& g,
      Vertex& start,
      Vertex& goal);

    // Add new nodes in the graph. Assume that client will call update methods
    // for those nodes on their own.
    void addNewNodes(std::vector<Vertex>& newNodes);

    // Get key of a node.
    double calculateKey(Vertex& node);

    // this must be called called when u's dist and/or lookahead dist
    // (and therefore consistency) may have been changed
    // this ensures u is in the queue correctly
    void updateVertex(Graph& g, Vertex& u);

    // this is called to update vertex v due to either:
    // - u_dist being updated OR
    // - uv_weight being updated
    // it will recalculate v's lookahead distance
    // and return whether v's lookahead distance changed
    // (if predecessor changed, but lookahead value didnt, return false)
    bool updatePredecessor(Graph& g, Vertex& u, Vertex& v);

    // Return shortest path.
    std::vector<Vertex> computeShortestPath(Graph& g);

    // Helper method for the above. Follows prev map to recover the shortest
    // path.
    std::vector<Vertex> followBackpointers();

}; // class LPAStar

#endif // LPA_STAR_
