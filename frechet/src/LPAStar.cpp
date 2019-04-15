#include "LPAStar.h"

double LPAStar::getDistance(Vertex &v) { return mDistance[v]; }

double LPAStar::getDistanceLookahead(Vertex &v) {
  return mDistanceLookahead[v];
}

void LPAStar::updatePQ(Graph &g, Vertex &v, double newPriority) {
  IndexMap index_map = get(boost::vertex_index, g);
  mPQ.update(index_map[v], newPriority);
}

void LPAStar::insertPQ(Graph &g, Vertex &v, double priority) {
  IndexMap index_map = get(boost::vertex_index, g);
  mPQ.insert(index_map[v], priority);
}

void LPAStar::removePQ(Graph &g, Vertex &v) {
  IndexMap index_map = get(boost::vertex_index, g);
  mPQ.remove(index_map[v]);
}

bool LPAStar::containsPQ(Graph &g, Vertex &v) {
  IndexMap index_map = get(boost::vertex_index, g);
  return mPQ.contains(index_map[v]);
}

bool LPAStar::isEmptyPQ() { return (mPQ.size() == 0); }

double LPAStar::peekPQ() { return mPQ.top_key(); }

Vertex LPAStar::popPQ(Graph &g) {
  Vertex top_vertex = vertex(mPQ.top_idx(), g);

  mPQ.remove_min();
  return top_vertex;
}

void LPAStar::initLPA(Graph &g, Vertex &start, Vertex &goal) {
  mStartNode = start;
  mGoalNode = goal;

  std::vector<Vertex> initNodes = getGraphVertices(g);
  for (auto &vertex : initNodes) {
    mDistance[vertex] = mInfVal;
    mDistanceLookahead[vertex] = mInfVal;
  }

  mDistanceLookahead[mStartNode] = 0.0;
  insertPQ(g, mStartNode, 0.0);
}

double LPAStar::calculateKey(Vertex &node) {
  return std::min(getDistance(node), getDistanceLookahead(node));
}

void LPAStar::updateVertex(Graph &g, Vertex &u) {
  EPLengthMap edgeLengthMap = get(&EProp::length, g);

  if (u != mStartNode) {
    std::vector<double> lookaheadValues;

    VPFrechetMap frechetMap = get(&VProp::frechet, g);
    double uFrechetWeight = frechetMap[u];

    auto preds = getPredecessors(u, g);
    for (auto &curPred : preds) {
      Edge curEdge = boost::edge(curPred, u, g).first;
      double edgeWeight = edgeLengthMap[curEdge];
      double curLookahead = std::max(getDistance(curPred), edgeWeight);

      lookaheadValues.push_back(curLookahead);
    }

    if (lookaheadValues.size() == 0) {
      mDistanceLookahead[u] = mInfVal;
    } else {
      std::vector<double>::iterator it = std::min_element(
          std::begin(lookaheadValues), std::end(lookaheadValues));

      double minLookahead = *it;
      mDistanceLookahead[u] = minLookahead;
    }
  }

  if (containsPQ(g, u))
    removePQ(g, u);

  if (getDistance(u) != getDistanceLookahead(u))
    insertPQ(g, u, calculateKey(u));
}

bool LPAStar::updatePredecessor(Graph &g, Vertex &u, Vertex &v) {
  // start vertex dist lookahead is always zero
  if (v == mStartNode)
    return false;

  bool u_relied_on = false;
  if (mPrev.count(v)) {
    Vertex v_pred = mPrev[v];
    u_relied_on = v_pred == u;
  }

  VPFrechetMap frechetMap = get(&VProp::frechet, g);
  double vFrechetWeight = frechetMap[v];

  double v_look_old = getDistanceLookahead(v);

  EPLengthMap edgeLengthMap = get(&EProp::length, g);
  Edge uv_edge = boost::edge(u, v, g).first;
  double uv_weight = edgeLengthMap[uv_edge];

  double v_look_u = std::max(getDistance(u), uv_weight);

  if (u_relied_on) // u was previously relied upon
  {
    // if dist through u decreased, then u is still best; just update value
    if (v_look_u == v_look_old) {
      return false;
    } else if (v_look_u < v_look_old) {
      mDistanceLookahead[v] = v_look_u;
      return true;
    } else // dist through u increased
    {
      // so we need to search for a potentially new best predecessessor
      double v_look_best = mInfVal;
      Vertex v_pred_best;

      std::vector<Vertex> v_preds = getPredecessors(v, g);
      for (auto curPred : v_preds) {
        Edge predEdge = boost::edge(curPred, v, g).first;
        double predEdgeWeight = edgeLengthMap[predEdge];
        double curLookahead = std::max(getDistance(curPred), predEdgeWeight);

        if (curLookahead < v_look_best) {
          v_look_best = curLookahead;
          v_pred_best = curPred;
        }
      }

      if (v_look_best != mInfVal)
        mPrev[v] = v_pred_best;

      if (v_look_best == v_look_old) {
        return false;
      } else {
        mDistanceLookahead[v] = v_look_best;
        return true;
      }
    }
  } else // some other (existing) predecessor was used by v
  {
    if (v_look_u < v_look_old) // dist through u is better
    {
      mPrev[v] = u;
      mDistanceLookahead[v] = v_look_u;
      return true;
    } else // u is not better
    {
      return false;
    }
  }
}

std::vector<Vertex> LPAStar::computeShortestPath(Graph &g) {
  // NOTE: Top element of PQ is now allowed to have *equal* cost to that of the
  // goal since we are doing bottleneck search. Otherwise, nothing will get
  // popped from the PQ after an edge change and update, causing nothing in the
  // prev map to get rewired.
  while (!isEmptyPQ() &&
         (peekPQ() <= calculateKey(mGoalNode) ||
          getDistanceLookahead(mGoalNode) != getDistance(mGoalNode))) {
    double curCost = peekPQ();
    Vertex u = popPQ(g);

    if (getDistance(u) > getDistanceLookahead(u)) {
      // Force it to be consistent.
      mDistance[u] = getDistanceLookahead(u);

      std::vector<Vertex> neighbors = getNeighbors(u, g);
      for (auto successor : neighbors) {
        if (updatePredecessor(g, u, successor))
          updateVertex(g, successor);
      }
    } else {
      mDistance[u] = mInfVal;
      updateVertex(g, u);

      std::vector<Vertex> neighbors = getNeighbors(u, g);
      for (auto successor : neighbors) {
        if (updatePredecessor(g, u, successor))
          updateVertex(g, successor);
      }
    }
  }

  // Actual path.
  return followBackpointers();
}

std::vector<Vertex> LPAStar::followBackpointers() {
  // Check if we actually reached the goal vertex. If we didn't, fail and
  // cry (by returning an empty vector).
  if (getDistance(mGoalNode) == mInfVal)
    return std::vector<Vertex>();

  std::vector<Vertex> finalPath;
  finalPath.push_back(mGoalNode);
  Vertex curBack = mPrev[mGoalNode];

  while (curBack != mStartNode) {
    finalPath.push_back(curBack);
    curBack = mPrev[curBack];
  }
  finalPath.push_back(mStartNode);

  std::reverse(finalPath.begin(), finalPath.end());
  return finalPath;
}
