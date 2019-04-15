#include "util.hpp"

std::vector<double> linDoubleSpace(double min, double max, size_t N) {
  std::vector<double> range;
  double delta = (max - min) / double(N - 1);
  for (int i = 0; i < N; i++) {
    double curValue = min + i * delta;
    range.push_back(curValue);
  }
  return range;
}

std::vector<int> linIntSpace(double min, double max, size_t N) {
  std::vector<int> intRange;
  std::vector<double> range = linDoubleSpace(min, max, N);
  for (double d : range) {
    intRange.push_back((int)d);
  }

  return intRange;
}

std::vector<Vertex>
flattenVertexList(std::vector<std::vector<Vertex>> &toFlatten,
                  int startingIndex, int endingIndex) {
  std::vector<Vertex> flattenedList;
  for (int i = startingIndex; i <= endingIndex; i++)
    for (auto &singleNode : toFlatten.at(i))
      flattenedList.push_back(singleNode);

  return flattenedList;
}

std::vector<Vertex> findKnnNodes(Vertex &queryVertex,
                                 std::vector<Vertex> &nodes, Graph &nnGraph,
                                 int numK,
                                 const ompl::base::StateSpacePtr stateSpace) {
  VPStateMap stateMap = get(&VProp::state, nnGraph);
  auto queryState = stateMap[queryVertex];

  std::vector<std::pair<double, Vertex>> nodeDists;
  for (auto &singleNode : nodes) {
    auto nodeState = stateMap[singleNode];
    double nodeDistance = stateSpace->distance(queryState, nodeState);
    nodeDists.push_back(std::make_pair(nodeDistance, singleNode));
  }

  // Sort the pair vector by increasing first key.
  std::sort(nodeDists.begin(), nodeDists.end());

  std::vector<Vertex> nearestNeighbors;
  for (int i = 0; i < numK && i < nodeDists.size(); i++) {
    Vertex closeVertex = nodeDists.at(i).second;
    nearestNeighbors.push_back(closeVertex);
  }

  return nearestNeighbors;
}

std::vector<Vertex> getGraphVertices(Graph &graph) {
  std::vector<Vertex> graphVertices;

  std::pair<VertexIter, VertexIter> vp;
  for (vp = vertices(graph); vp.first != vp.second; ++vp.first) {
    Vertex curVertex = *vp.first;
    graphVertices.push_back(curVertex);
  }

  return graphVertices;
}

std::vector<Vertex> getNeighbors(const Vertex &v, Graph &g) {
  std::vector<Vertex> neighbors;
  NeighborIter ai;
  NeighborIter aiEnd;

  for (tie(ai, aiEnd) = adjacent_vertices(v, g); ai != aiEnd; ++ai) {
    Vertex curNeighbor = *ai;
    neighbors.push_back(curNeighbor);
  }

  return neighbors;
}

std::vector<Vertex> getPredecessors(const Vertex &v, Graph &g) {
  std::vector<Vertex> predecessors;
  boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;

  for (tie(ei, ei_end) = in_edges(v, g); ei != ei_end; ++ei) {
    Vertex curPred = source(*ei, g);
    predecessors.push_back(curPred);
  }

  return predecessors;
}
