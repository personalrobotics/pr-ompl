#include "LPAStar.h"

std::pair<double, double> LPAStar::getDistance(Vertex& v)
{
  // NOTE: We use this function since we might construct the CPG on demand. So
  // v might actually have been initialized in distance.
  if (!distance.count(v))
    distance[v] = std::make_pair(infVal, infVal);

  return distance[v];
}

std::pair<double, double> LPAStar::getDistanceLookahead(Vertex& v)
{
  // NOTE: We use this function since we might construct the CPG on demand. So
  // v might actually have been initialized in distanceLookahead.
  if (!distanceLookahead.count(v))
    distanceLookahead[v] = std::make_pair(infVal, infVal);

  return distanceLookahead[v];
}

void LPAStar::updatePQ(
  Graph& g,
  Vertex& v,
  std::pair<double, double> newPriority
) {
  vertexIndexMap index_map = get(boost::vertex_index, g);
  pq.update(index_map[v], newPriority);
}

void LPAStar::insertPQ(Graph& g, Vertex& v, std::pair<double, double> priority)
{
  vertexIndexMap index_map = get(boost::vertex_index, g);
  pq.insert(index_map[v], priority);
}

void LPAStar::removePQ(Graph& g, Vertex& v)
{
  vertexIndexMap index_map = get(boost::vertex_index, g);
  pq.remove(index_map[v]);
}

bool LPAStar::containsPQ(Graph& g, Vertex& v)
{
  vertexIndexMap index_map = get(boost::vertex_index, g);
  return pq.contains(index_map[v]);
}

bool LPAStar::isEmptyPQ()
{
  return (pq.size() == 0);
}

std::pair<double, double> LPAStar::peekPQ()
{
  return pq.top_key();
}

Vertex LPAStar::popPQ(Graph& g, std::pair<double, double>& priorityOut)
{
  priorityOut = pq.top_key();
  Vertex top_vertex = vertex(pq.top_idx(), g);

  pq.remove_min();
  return top_vertex;
}

void LPAStar::initLPA(
  Graph& g,
  Vertex& start,
  Vertex& goal,
  std::function<double(Edge& cpgEdge)> edgeWeightFunc,
  std::function<std::vector<Vertex>(Vertex& cpgNode)> predFunc,
  std::function<std::vector<Vertex>(Vertex& cpgNode)> succFunc
) {
  // NOTE: Calling this twice resets the LPA* structures.
  mEdgeWeightFunc = edgeWeightFunc;
  mPredFunc = predFunc;
  mSuccFunc = succFunc;

  pq.reset();
  distance.clear();
  distanceLookahead.clear();
  prev.clear();

  startNode = start;
  goalNode = goal;

  std::vector<Vertex> initNodes = getGraphVertices(g);
  for (auto& vertex : initNodes)
  {
    distance[vertex] = std::make_pair(infVal, infVal);
    distanceLookahead[vertex] = std::make_pair(infVal, infVal);
  }

  distanceLookahead[startNode] = std::make_pair(0, 0);
  insertPQ(g, startNode, std::make_pair(0, 0));
}

void LPAStar::addNewNodes(std::vector<Vertex>& newNodes)
{
  for (auto newVertex : newNodes)
  {
    distance[newVertex] = std::make_pair(infVal, infVal);
    distanceLookahead[newVertex] = std::make_pair(infVal, infVal);
  }
}

std::pair<double, double> LPAStar::combineCost(
  std::pair<double, double> priority,
  double uvEdgeCost,
  double vFrechetWeight
) {
  std::pair<double, double> newPriority;
  newPriority.first = std::max(priority.first, uvEdgeCost);

  // We always do bottleneck search, but we break ties by taking the path that
  // minimizes the SUM of deviations along the parameterization between the
  // Roadmap and Reference graphs. This means we need to accumulate the Frechet
  // Weight of each node V whenever we take an edge U ---> V in the CPG.
  newPriority.second = priority.second + vFrechetWeight;

  return newPriority;
}

std::pair<double, double> LPAStar::calculateKey(Vertex& node)
{
  return std::min(getDistance(node), getDistanceLookahead(node));
}

void LPAStar::updateVertex(Graph& g, Vertex& u)
{
  if (u != startNode)
  {
    std::vector< std::pair<double, double> > lookaheadValues;

    VPFrechetDistanceMap frechetMap = get(&VProp::frechetDistance, g);
    double uFrechetWeight = frechetMap[u];

    auto preds = mPredFunc(u);
    for (auto& curPred : preds)
    {
      Edge curEdge = boost::edge(curPred, u, g).first;
      double edgeWeight = mEdgeWeightFunc(curEdge);
      std::pair<double, double> curLookahead = combineCost(
        getDistance(curPred),
        edgeWeight,
        uFrechetWeight);

      lookaheadValues.push_back(curLookahead);
    }

    if (lookaheadValues.size() == 0)
    {
      distanceLookahead[u] = std::make_pair(infVal, infVal);
    } else {
      std::vector< std::pair<double, double> >::iterator it = std::min_element(
        std::begin(lookaheadValues),
        std::end(lookaheadValues));

      std::pair<double, double> minLookahead = *it;
      distanceLookahead[u] = minLookahead;
    }
  }

  if (containsPQ(g, u))
    removePQ(g, u);

  if (getDistance(u) != getDistanceLookahead(u))
    insertPQ(g, u, calculateKey(u));
}

bool LPAStar::updatePredecessor(Graph& g, Vertex& u, Vertex& v)
{
   // start vertex dist lookahead is always zero
   if (v == startNode)
      return false;

   bool u_relied_on = false;
   if (prev.count(v))
   {
     Vertex v_pred = prev[v];
     u_relied_on = v_pred == u;
   }

   VPFrechetDistanceMap frechetMap = get(&VProp::frechetDistance, g);
   double vFrechetWeight = frechetMap[v];

   std::pair<double, double> v_look_old = getDistanceLookahead(v);

   Edge uv_edge = boost::edge(u, v, g).first;
   double uv_weight = mEdgeWeightFunc(uv_edge);

   std::pair<double, double> v_look_u = combineCost(
     getDistance(u),
     uv_weight,
     vFrechetWeight);

   if (u_relied_on) // u was previously relied upon
   {
      // if dist through u decreased, then u is still best; just update value
      if (v_look_u == v_look_old)
      {
         return false;
      }
      else if (v_look_u < v_look_old)
      {
         distanceLookahead[v] = v_look_u;
         return true;
      }
      else // dist through u increased
      {
        // so we need to search for a potentially new best predecessessor
        std::pair<double, double> v_look_best = std::make_pair(infVal, infVal);
        Vertex v_pred_best;

        std::vector<Vertex> v_preds = mPredFunc(v);
        for (auto curPred : v_preds)
        {
          Edge predEdge = boost::edge(curPred, v, g).first;
          double predEdgeWeight = mEdgeWeightFunc(predEdge);
          std::pair<double, double> curLookahead = combineCost(
            getDistance(curPred),
            predEdgeWeight,
            vFrechetWeight);

          if (curLookahead < v_look_best)
          {
            v_look_best = curLookahead;
            v_pred_best = curPred;
          }
        }

        if (v_look_best != std::make_pair(infVal, infVal))
          prev[v] = v_pred_best;

        if (v_look_best == v_look_old)
        {
          return false;
        }
        else
        {
          distanceLookahead[v] = v_look_best;
          return true;
        }
      }
   }
   else // some other (existing) predecessor was used by v
   {
      if (v_look_u < v_look_old) // dist through u is better
      {
         prev[v] = u;
         distanceLookahead[v] = v_look_u;
         return true;
      }
      else // u is not better
      {
         return false;
      }
   }
}

std::vector<Vertex>  LPAStar::computeShortestPath(Graph& g, double bound)
{
  // NOTE: Needed for bounding due to wierd dual cost.
  std::pair<double, double> boundPair = std::make_pair(bound, infVal);

  // NOTE: Top element of PQ is now allowed to have *equal* cost to that of the
  // goal since we are doing bottleneck search. Otherwise, nothing will get
  // popped from the PQ after an edge change and update, causing nothing in the
  // prev map to get rewired.
  while (!isEmptyPQ() && (peekPQ() <= calculateKey(goalNode)
     || getDistanceLookahead(goalNode) != getDistance(goalNode)))
  {
    std::pair<double, double> curCost = peekPQ();
    // NOTE: LPA* star processes non-decreasing keys, so use a bound to abort
    // early if one is set.
    if (bound > 0.0 && curCost > boundPair)
    {
      Vertex dummy;
      std::vector<Vertex> dummyPath;
      dummyPath.push_back(dummy);
      dummyPath.push_back(dummy);

      mAbortCost = curCost.first;
      return dummyPath;
    }

     Vertex u = popPQ(g);

     if (getDistance(u) > getDistanceLookahead(u))
     {
        // Force it to be consistent.
        distance[u] = getDistanceLookahead(u);

        std::vector<Vertex> neighbors = mSuccFunc(u);
        for (auto successor : neighbors)
        {
          if (updatePredecessor(g, u, successor))
            updateVertex(g, successor);
        }
     }
     else
     {
      distance[u] = std::make_pair(infVal, infVal);
      updateVertex(g, u);

      std::vector<Vertex> neighbors = mSuccFunc(u);
      for (auto successor : neighbors)
      {
        if (updatePredecessor(g, u, successor))
          updateVertex(g, successor);
      }
     }
  }

  // Actual path.
  return followBackpointers();
}

std::vector<Vertex> LPAStar::followBackpointers()
{
  // Check if we actually reached the goal vertex. If we didn't, fail and
  // cry (by returning an empty vector).
  if (getDistance(goalNode).first == infVal)
    return std::vector<Vertex>();

  std::vector<Vertex> finalPath;
  finalPath.push_back(goalNode);
  Vertex curBack = prev[goalNode];

  while (curBack != startNode)
  {
    finalPath.push_back(curBack);
    curBack = prev[curBack];
  }
  finalPath.push_back(startNode);

  std::reverse(finalPath.begin(),finalPath.end());
  return finalPath;
}
