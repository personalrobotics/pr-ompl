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
#include "BoostDefinitions.h"

#include "heap_indexed.h"

namespace po = boost::program_options;
using namespace BOOST_DEFINITIONS;
using namespace pr_bgl;

static std::pair<double, double> dummyPriorityOut;

typedef boost::property_map<Graph, boost::vertex_index_t>::type vertexIndexMap;

class LPAStar
{
  std::function<double(Edge& cpgEdge)> mEdgeWeightFunc;
  std::function<std::vector<Vertex>(Vertex& cpgNode)> mPredFunc;
  std::function<std::vector<Vertex>(Vertex& cpgNode)> mSuccFunc;

  heap_indexed< std::pair<double, double> > pq;

  Vertex startNode;
  Vertex goalNode;

  std::unordered_map< Vertex, std::pair<double, double> > distance;
  std::unordered_map< Vertex, std::pair<double, double> > distanceLookahead;
  std::unordered_map< Vertex, Vertex > prev;

  public:
    // Report the cost when an abort happens.
    double mAbortCost = 0.0;

    // For marking collision edges.
    double infVal = std::numeric_limits<double>::max();

    std::pair<double, double> getDistance(Vertex& v);

    std::pair<double, double> getDistanceLookahead(Vertex& v);

    void updatePQ(Graph& g, Vertex& v, std::pair<double, double> newPriority);

    void insertPQ(Graph& g, Vertex& v, std::pair<double, double> priority);

    void removePQ(Graph& g, Vertex& v);

    bool containsPQ(Graph& g, Vertex& v);

    bool isEmptyPQ();

    // Get the samllest priority in the queue.
    std::pair<double, double> peekPQ();

    // We use a min heap, so this should be the Vertex with the smallest
    // priority.
    Vertex popPQ(
      Graph& g,
      std::pair<double, double>& priorityOut = dummyPriorityOut);

    // NOTE: Actual LPA* methods.

    // Set up the data structures with original nodes and edges.
    void initLPA(
      Graph& g,
      Vertex& start,
      Vertex& goal,
      std::function<double(Edge& cpgEdge)> edgeWeightFunc = NULL,
      std::function<std::vector<Vertex>(Vertex& cpgNode)> predFunc = NULL,
      std::function<std::vector<Vertex>(Vertex& cpgNode)> succFunc = NULL);

    // Add new nodes in the graph. Assume that client will call update methods
    // for those nodes on their own.
    void addNewNodes(std::vector<Vertex>& newNodes);

    // Take the old priority of a node U, and get a new priority by combining it
    // with the edge leading from U to V.
    std::pair<double, double> combineCost(
      std::pair<double, double> priority,
      double uvEdgeCost,
      double vFrechetWeight);

    // Get key of a node.
    std::pair<double, double> calculateKey(Vertex& node);

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
    std::vector<Vertex> computeShortestPath(Graph& g, double bound = 0.0);

    // Helper method for the above. Follows prev map to recover the shortest
    // path.
    std::vector<Vertex> followBackpointers();

}; // class LPAStar

#endif // LPA_STAR_
