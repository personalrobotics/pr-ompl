#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <iostream>         // std::cerr
#include <queue>            // std::priority_queue
#include <set>              // std::set
#include <unordered_set>    // std::unordered_set
#include <fstream>          // Write to file
#include <assert.h>         // Debug
#include <chrono>           // record rewireTime

#include "NNFrechet.hpp"

namespace NNFrechet
{

NNFrechet::NNFrechet(
  const ompl::base::SpaceInformationPtr &si,
  std::vector<Eigen::Isometry3d>& referencePath,
  int numWaypoints,
  int ikMultiplier,
  int numNN,
  int discretization,
  int seed)
  : ompl::base::Planner(si, "NNFrechet")
  , mSpace(si->getStateSpace())
  , mNumWaypoints(numWaypoints)
  , mIKMultiplier(ikMultiplier)
  , mNumNN(numNN)
  , mDiscretization(discretization)
{
  mRandomGenerator.seed(seed);

  // Don't use the entire reference path. Subsample it.
  mReferencePath = subsampleRefPath(referencePath);
}

std::vector<Eigen::Isometry3d> NNFrechet::subsampleRefPath(
  std::vector<Eigen::Isometry3d>& referencePath
) {
  int numPts = ((mNumWaypoints - 1) * (mDiscretization + 1)) + 1;
  std::vector<int> sampledIDs = linIntSpace(0, mReferencePath.size() - 1, numPts);

  std::vector<Eigen::Isometry3d> subSampled;
  for (int i = 0; i < sampledIDs.size(); i++)
  {
    int sampledIndex = sampledIDs[i];
    Eigen::Isometry3d sampledPose = mReferencePath[sampledIndex];
    subSampled.push_back(sampledPose);
  }

  return subSampled;
}

void NNFrechet::setFKFunc(
  std::function<Eigen::Isometry3d(Eigen::VectorXd&)> fkFunc
) {
  mFkFunc = fkFunc;
}

void NNFrechet::setIKFunc(
  std::function<std::vector<Eigen::VectorXd>(Eigen::Isometry3d&, int)> ikFunc
) {
  mIkFunc = ikFunc;
}

void NNFrechet::setDistanceFunc(
  std::function<double(Eigen::Isometry3d&, Eigen::Isometry3d&)> distanceFunc
) {
  mDistanceFunc = distanceFunc;
}

void NNFrechet::buildReferenceGraph()
{
  VPNameMap nameMap = get(&VProp::name, mReferenceGraph);
  VPPoseEEMap poseMap = get(&VProp::poseEE, mReferenceGraph);

  Vertex prevVertex;
  for (int i = 0; i < mReferencePath.size(); i++)
  {
    Vertex newRefVertex = add_vertex(mReferenceGraph);

    Eigen::Isometry3d curPose = mReferencePath[i];
    poseMap[newRefVertex] = curPose;

    std::string refVertexName = "r" + std::to_string(i);
    nameMap[newRefVertex] = refVertexName;

    if (i > 0)
    {
      add_edge(prevVertex, newRefVertex, mReferenceGraph);
    }

    prevVertex = newRefVertex;

    // Record the first and last node on the reference graph.
    if (i == 0)
      mRefStartNode = newRefVertex;
    else if (i == mReferencePath.size() - 1)
      mRefGoalNode = newRefVertex;
  }
}

std::vector<Vertex> NNFrechet::sampleIKNodes(
  Eigen::Isometry3d& curWaypoint,
  int numSolutions
) {
  VPNameMap nameMap = get(&VProp::name, mNNGraph);
  VPStateMap stateMap = get(&VProp::state, mNNGraph);
  VPPoseEEMap poseMap = get(&VProp::poseEE, mNNGraph);

  std::vector<Vertex> sampledNodes;
  std::vector<Eigen::VectorXd> ikSolutions = mIkFunc(curWaypoint, numSolutions);

  for (auto curSol : ikSolutions)
  {
    Vertex newNNVertex = add_vertex(mNNGraph);

    Eigen::Isometry3d curPose = mFkFunc(curSol);
    poseMap[newNNVertex] = curPose;

    // TODO: Right now there's just a hack converting the Eigen::Vec to an OMPL
    // state. Is there a better way to do this?
    std::vector<double> jointStates;
    for (int i = 0; i < mSpace->getDimension(); i++)
      jointStates.push_back(curSol[i]);

    auto ikState = mSpace->allocState();
    mSpace->copyFromReals(ikState, jointStates);
    stateMap[newNNVertex] = ikState;

    // Node meta-data.
    std::string newNodeName = "n." + std::to_string(mNNIKID);
    nameMap[newNNVertex] = newNodeName;
    mNNIKID++;

    sampledNodes.push_back(newNNVertex);
  }

  return sampledNodes;
}

std::vector< std::vector<Vertex> > NNFrechet::sampleNNGraphNodes(
  int numSamples
) {
  // Record which new nodes were sampled, and which indices from the reference
  // path they were sampled from.
  std::vector< std::vector<Vertex> > newSampledNodes;
  newSampledNodes.resize(mReferencePath.size());

  // We go in the range [1, refIDs.size() - 2] so we ignore the endpoints. They
  // were handled.
  int beginRefIndex = 1;
  int endRefIndex = mReferencePath.size() - 2;

  // Sample indices on the reference path.
  std::vector<int> indexSampleCounts(mReferencePath.size(), 0);
  for (int i = 0; i < numSamples; i++)
  {
    std::uniform_int_distribution<int> unifrom(beginRefIndex, endRefIndex);
    int randIndex = unifrom(mRandomGenerator);
    indexSampleCounts.at(randIndex) = indexSampleCounts.at(randIndex) + 1;
  }

  // Sample IK solutions for each reference path point based on how many
  // times the above loop sampled it.
  for (int sampleIndex = beginRefIndex; sampleIndex <= endRefIndex; sampleIndex++)
  {
    int sampleCount = indexSampleCounts.at(sampleIndex);
    Eigen::Isometry3d& sampleWaypoint = mReferencePath.at(sampleIndex);

    auto sampledNodeVector = sampleIKNodes(sampleWaypoint, sampleCount);
    for (auto newNode : sampledNodeVector)
    {
      newSampledNodes.at(sampleIndex).push_back(newNode);
    }
  }

  return newSampledNodes;
}

void NNFrechet::addSubsampledEdge(
  Vertex& firstNNVertex,
  Vertex& secondNNVertex
) {
  VPNameMap nameMap = get(&VProp::name, mNNGraph);
  VPStateMap stateMap = get(&VProp::state, mNNGraph);
  VPPoseEEMap poseMap = get(&VProp::poseEE, mNNGraph);

  // Handle the cases where edge is not subsampled.
  if (firstNNVertex == mNNStartNode || secondNNVertex == mNNGoalNode)
  {
   add_edge(firstNNVertex, secondNNVertex, mNNGraph);
   return;
  }

  std::vector<ompl::base::State*> intermediateStates;
  std::vector<Eigen::Isometry3d> intermediatePoses;

  auto firstState = stateMap[firstNNVertex];
  auto secondState = stateMap[secondNNVertex];

  // Interpolate the intermediate configs.
  for (int i = 0; i < mDiscretization; i++)
  {
    auto curIntermediate = mSpace->allocState();
    mSpace->interpolate(firstState, secondState,
      (1.0 + i)/(mDiscretization+1), curIntermediate);
    intermediateStates.push_back(curIntermediate);
  }

  for (auto curIntermediate : intermediateStates)
  {
    // TODO: This exchange between OMPL and Eigen is a pain in the ass. How
    // should it be handled?
    std::vector<double> jointStates;
    mSpace->copyToReals(jointStates, curIntermediate);

    Eigen::VectorXd curConfig(mSpace->getDimension());
    for (int i = 0; i < mSpace->getDimension(); i++)
      curConfig[i] = jointStates[i];

    Eigen::Isometry3d curFK = mFkFunc(curConfig);
    intermediatePoses.push_back(curFK);
  }

  Vertex prevVertex = firstNNVertex;
  for (int i = 0; i < mDiscretization; i++)
  {
    // New node creation and data insertion.
    Vertex newSubVertex = add_vertex(mNNGraph);
    std::string newNodeName = "a" + std::to_string(mNNSubsampleID);

    nameMap[newSubVertex] = newNodeName;
    stateMap[newSubVertex] = intermediateStates[i];
    poseMap[newSubVertex] = intermediatePoses[i];

    // Add an "internal" edge resulting from subsampling an original edge.
    add_edge(prevVertex, newSubVertex, mNNGraph);
    // And if we reached the end...
    if (i == mDiscretization - 1)
    {
      add_edge(newSubVertex, secondNNVertex, mNNGraph);
    }

    prevVertex = newSubVertex;
    mNNSubsampleID++;
  }
}

void NNFrechet::buildNNGraph()
{
  VPNameMap nameMap = get(&VProp::name, mNNGraph);
  VPPoseEEMap poseMap = get(&VProp::poseEE, mNNGraph);

  // Set up the variable that keeps track of nodes sampled for the NN Graph.
  std::vector< std::vector<Vertex> > sampledNNGraphNodes;
  sampledNNGraphNodes.resize(mReferencePath.size());

  // NOTE: Add dummy start vertex to the NN Graph.
  mNNStartNode = add_vertex(mNNGraph);
  nameMap[mNNStartNode] = "c0";
  poseMap[mNNStartNode] = mReferencePath.at(0);

  // NOTE: Add dummy end vertex to the NN Graph.
  mNNGoalNode = add_vertex(mNNGraph);
  nameMap[mNNGoalNode] = "c1";
  poseMap[mNNGoalNode] = mReferencePath.back();

  // NOTE: We always sample the start and end vertices since we want the
  // path to reach those. We also sample multiple solutions for each.
  Eigen::Isometry3d& firstWaypoint = mReferencePath.at(0);
  std::vector<Vertex> firstWaypointVertices = sampleIKNodes(firstWaypoint, mIKMultiplier);

  for (Vertex firstWaypointNode : firstWaypointVertices)
  {
    add_edge(mNNStartNode, firstWaypointNode, mNNGraph);
    sampledNNGraphNodes.at(0).push_back(firstWaypointNode);
  }

  // NOTE: Now repeat eveything we did, but for the last waypoint on the
  // reference path.
  Eigen::Isometry3d& lastWaypoint = mReferencePath.back();
  std::vector<Vertex> lastWaypointVertices = sampleIKNodes(lastWaypoint, mIKMultiplier);

  for (Vertex lastWaypointNode : lastWaypointVertices)
  {
    sampledNNGraphNodes.back().push_back(lastWaypointNode);
    add_edge(lastWaypointNode, mNNGoalNode, mNNGraph);
  }

  int remainingIKBudget = mNumWaypoints * mIKMultiplier;
  // We already used some of this for the end points.
  remainingIKBudget -= 2*mIKMultiplier;

  // Sample nodes for waypoints besides the endpoints of the reference path.
  auto sampledNodes = sampleNNGraphNodes(remainingIKBudget);

  for (int refIndex = 0; refIndex < sampledNodes.size(); refIndex++)
  {
    for (auto indexNewNode : sampledNodes.at(refIndex))
      sampledNNGraphNodes.at(refIndex).push_back(indexNewNode);
  }

  // Connect each raodmap node to its kNN in C-Space that are sampled *further*
  // down the reference path (to ensure monotonicity).
  for (int refIndex = 0; refIndex < sampledNNGraphNodes.size(); refIndex++)
  {
    for (auto& nodeSampledAtWaypoint : sampledNNGraphNodes.at(refIndex))
    {
      std::vector<Vertex> deeperNodes = flattenVertexList(
        sampledNNGraphNodes,
        refIndex + 1,
        sampledNNGraphNodes.size() - 1);

      // Get neighbors sampled from waypoints further along the reference path.
      std::vector<Vertex> cSpaceNeighbors = findKnnNodes(
        nodeSampledAtWaypoint,
        deeperNodes,
        mNNGraph,
        mNumNN,
        mSpace);

      for (auto& closeNode : cSpaceNeighbors)
        addSubsampledEdge(nodeSampledAtWaypoint, closeNode);
    }
  }
}

void NNFrechet::addTensorProductNodes(
  std::vector<Vertex>& refNodes,
  std::vector<Vertex>& nnNodes
) {
  VPNameMap refNameMap = get(&VProp::name, mReferenceGraph);
  VPPoseEEMap refPoseMap = get(&VProp::poseEE, mReferenceGraph);

  VPNameMap nnNameMap = get(&VProp::name, mNNGraph);
  VPPoseEEMap nnPoseMap = get(&VProp::poseEE, mNNGraph);
  VPStateMap nnStateMap = get(&VProp::state, mNNGraph);

  VPNameMap tensorNameMap = get(&VProp::name, mTensorProductGraph);
  VPStateMap tensorStateMap = get(&VProp::state, mTensorProductGraph);
  VPNNComponentMap nnComponentMap = get(&VProp::nnComponent, mTensorProductGraph);
  VPFrechetMap frechetMap = get(&VProp::frechet, mTensorProductGraph);

  int numTPGNodes = 0;
  for (const auto& curRefNode : refNodes)
  {
    std::string refNodeName = refNameMap[curRefNode];

    for (const auto& curNNNode : nnNodes)
    {
      std::string nnNodeName = nnNameMap[curNNNode];

      Vertex newTensorNode = add_vertex(mTensorProductGraph);
      std::string tensorVertexName = refNodeName + "_" + nnNodeName;

      // Compute the Frechet weight of this node.
      Eigen::Isometry3d refPose = refPoseMap[curRefNode];
      Eigen::Isometry3d nnPose = nnPoseMap[curNNNode];
      double frechetWeight = mDistanceFunc(refPose, nnPose);

      tensorNameMap[newTensorNode] = tensorVertexName;
      frechetMap[newTensorNode] = frechetWeight;
      mNameToVertex[tensorVertexName] = newTensorNode;

      // Also store nn information to recover plan later.
      tensorStateMap[newTensorNode] = nnStateMap[curNNNode];
      nnComponentMap[newTensorNode] = curNNNode;

      // Save the start and goal nodes of the TPG.
      if (curRefNode == mRefStartNode && curNNNode == mNNStartNode)
        mTensorStartNode = newTensorNode;

      if (curRefNode == mRefGoalNode && curNNNode == mNNGoalNode)
        mTensorGoalNode = newTensorNode;

      numTPGNodes++;
    }
  }

  std::cout << "[INFO]: TPG has " << numTPGNodes << " nodes." << std::endl;
}

void NNFrechet::addTensorProductEdge(
  Vertex& v1,
  Vertex& v2
) {
  VPFrechetMap frechetMap = get(&VProp::frechet, mTensorProductGraph);
  EPLengthMap edgeLengthMap = get(&EProp::length, mTensorProductGraph);
  VPNNComponentMap nnComponentMap = get(&VProp::nnComponent, mTensorProductGraph);

  std::pair<Edge, bool> edgePair = add_edge(v1, v2, mTensorProductGraph);
  bool edgeAdded = edgePair.second;

  if (edgeAdded)
  {
    Edge newEdge = edgePair.first;
    edgeLengthMap[newEdge] = std::max(frechetMap[v1], frechetMap[v2]);

    Vertex nnU = nnComponentMap[v1];
    Vertex nnV = nnComponentMap[v2];

    // This actually represents an edge in the roadmap.
    if (nnU != nnV)
    {
      VPNameMap nnNameMap = get(&VProp::name, mNNGraph);
      std::string nnEdgeString = nnNameMap[nnU] + "_" + nnNameMap[nnV];

      // Optimization to quickly kill lots of nodes in the TPG using LazySP.
      if (!mNNToTPGEdges.count(nnEdgeString))
        mNNToTPGEdges[nnEdgeString] = std::vector<Edge>();

      mNNToTPGEdges[nnEdgeString].push_back(newEdge);
    }
  }
}

void NNFrechet::connectTensorProductNodes(
  std::vector<Vertex>& refNodes,
  std::vector<Vertex>& nnNodes
) {
  VPNameMap refNameMap = get(&VProp::name, mReferenceGraph);
  VPNameMap nnNameMap = get(&VProp::name, mNNGraph);
  VPNameMap tensorNameMap = get(&VProp::name, mTensorProductGraph);

  for (const auto& curRefNode : refNodes)
  {
    std::string refNodeName = refNameMap[curRefNode];
    std::vector<Vertex> refNodeNeighbors = getNeighbors(curRefNode, mReferenceGraph);

    for (const auto& curNNNode : nnNodes)
    {
      std::string nnNodeName = nnNameMap[curNNNode];
      std::vector<Vertex> nnNodeNeighbors = getNeighbors(curNNNode, mNNGraph);

      std::string tensorNodeName = refNodeName + "_" + nnNodeName;
      Vertex curTensorNode = mNameToVertex[tensorNodeName];

      // Option A: Take a step on the reference graph.
      for (Vertex refNeighbor : refNodeNeighbors)
      {
        std::string refNeighborName = refNameMap[refNeighbor];
        std::string tensorNeighborName = refNeighborName + "_" + nnNodeName;

        Vertex tensorNeighborNode = mNameToVertex[tensorNeighborName];
        addTensorProductEdge(curTensorNode, tensorNeighborNode);
      }

      // Option B: Take a step on the NN graph.
      for (Vertex nnNeighbor : nnNodeNeighbors)
      {
        std::string nnNeighborName = nnNameMap[nnNeighbor];
        std::string tensorNeighborName = refNodeName + "_" + nnNeighborName;

        Vertex tensorNeighborNode = mNameToVertex[tensorNeighborName];
        addTensorProductEdge(curTensorNode, tensorNeighborNode);
      }

      // Option C: Take a step on *both*.
      for (Vertex refNeighbor : refNodeNeighbors)
      {
        for (Vertex nnNeighbor : nnNodeNeighbors)
        {
          std::string refNeighborName = refNameMap[refNeighbor];
          std::string nnNeighborName = nnNameMap[nnNeighbor];
          std::string tensorNeighborName = refNeighborName + "_" + nnNeighborName;

          Vertex tensorNeighborNode = mNameToVertex[tensorNeighborName];
          addTensorProductEdge(curTensorNode, tensorNeighborNode);
        }
      }
    }
  }
}

void NNFrechet::buildTensorProductGraph()
{
  std::vector<Vertex> refNodes = getGraphVertices(mReferenceGraph);
  std::vector<Vertex> nnNodes = getGraphVertices(mNNGraph);

  std::cout << "[INFO]: Ref Graph has " << refNodes.size() << " nodes."
    << std::endl;
  std::cout << "[INFO]: NN Graph has " << nnNodes.size() << " nodes."
    << std::endl;

  // Set up the tensor product graph with the correct nodes.
  addTensorProductNodes(refNodes, nnNodes);

  // And connect them up to eachother.
  connectTensorProductNodes(refNodes, nnNodes);
}

void NNFrechet::initStructures()
{
  // Build all three graphs.
  buildReferenceGraph();
  buildNNGraph();
  buildTensorProductGraph();

  std::cout << "[INFO]: Tensor Product Graph has been built." << std::endl;

  mLPAStar.initLPA(mTensorProductGraph, mTensorStartNode, mTensorGoalNode);
  std::cout << "[INFO]: LPA* structure initialized." << std::endl;
}

std::vector<Vertex> NNFrechet::extractNNPath(
  std::vector<Vertex>& tensorProductPath
) {
  VPNNComponentMap nnComponentMap =
    get(&VProp::nnComponent, mTensorProductGraph);

  std::vector<Vertex> nnPath;
  for (int i = 0; i < tensorProductPath.size(); i++)
  {
    Vertex tensorVertex = tensorProductPath[i];
    Vertex curNNVertex = nnComponentMap[tensorVertex];

    // NOTE: The start/goal are dummies. Ignore for now.
    // TODO: Should they *not* be dummies?
    if (curNNVertex == mNNStartNode || curNNVertex == mNNGoalNode)
      continue;

    // This represents an actual edge in the NN Graph.
    if (nnPath.size() == 0 || nnPath.back() != curNNVertex)
    {
      nnPath.push_back(curNNVertex);
    }
  }

  return nnPath;
}

bool NNFrechet::checkEdgeEvaluation(Vertex& source, Vertex& target)
{
  EPEvaluatedMap evalMap = get(&EProp::evaluated, mNNGraph);
  Edge edgeUV = boost::edge(source, target, mNNGraph).first;

  // TODO: Do we have to manually set things to false?
  return evalMap[edgeUV];
}

bool NNFrechet::evaluateEdge(
  Vertex& source,
  Vertex& target
) {
  // First, mark the edge as evlauated.
  EPEvaluatedMap evalMap = get(&EProp::evaluated, mNNGraph);
  Edge edgeUV = boost::edge(source, target, mNNGraph).first;
  evalMap[edgeUV] = true;

  VPStateMap stateMap = get(&VProp::state, mNNGraph);
  ompl::base::State* startState  = stateMap[source];
  ompl::base::State* endState = stateMap[target];

  auto validityChecker = si_->getStateValidityChecker();
  auto checkState = mSpace->allocState();

  double length = mSpace->distance(startState, endState);
  int numQ = ceil(length/mCheckResolution);

  // Interpolate and coll-check the edge.
  double step = 1.0/numQ;
  for (double alpha = 0.0; alpha <= 1.0; alpha += step)
  {
    mSpace->interpolate(startState, endState, alpha, checkState);
    if (!validityChecker->isValid(checkState))
    {
      mSpace->freeState(checkState);
      return true;
    }
  }

  mSpace->freeState(checkState);
  return false;
}

void NNFrechet::markEdgeInCollision(Vertex& nnU, Vertex& nnV)
{
  EPLengthMap edgeLengthMap = get(&EProp::length, mTensorProductGraph);
  VPNameMap nnNameMap = get(&VProp::name, mNNGraph);

  std::string nnEdgeString = nnNameMap[nnU] + "_" + nnNameMap[nnV];
  std::vector<Edge>& mappedTPGEdges = mNNToTPGEdges[nnEdgeString];

  for (const auto& curEdge : mappedTPGEdges)
    edgeLengthMap[curEdge] = mLPAStar.mInfVal;

  for (const auto& updateEdge : mappedTPGEdges)
  {
    Vertex tensorU = source(updateEdge, mTensorProductGraph);
    Vertex tensorV = target(updateEdge, mTensorProductGraph);

    if (mLPAStar.updatePredecessor(mTensorProductGraph, tensorU, tensorV))
      mLPAStar.updateVertex(mTensorProductGraph, tensorV);
  }
}

std::vector<ompl::base::State*> NNFrechet::lazySP()
{
  VPStateMap stateMap = get(&VProp::state, mNNGraph);

  // Lazy SP style. Just keep searching until you find a collision free
  // path that works.
  while (true)
  {
    std::vector<Vertex> shortestPath =
      mLPAStar.computeShortestPath(mTensorProductGraph);

    // Shortest path was all in collision.
    if (shortestPath.size() == 0)
      return std::vector<ompl::base::State*>();

    std::vector<Vertex> nnPath = extractNNPath(shortestPath);

    std::vector<ompl::base::State*> finalStates;
    bool collisionFree = true;
    // NOTE: We check the current node and the next one, so stop one node
    // early on the path.
    for (int i = 0; i < nnPath.size() - 1; i++)
    {
      Vertex curVertex = nnPath[i];
      Vertex nextVertex = nnPath[i + 1];

      bool alreadyEvaluated = checkEdgeEvaluation(
        curVertex,
        nextVertex);

      if (!alreadyEvaluated)
      {
        // TODO: Forward collision checking?
        bool inCollision = evaluateEdge(curVertex, nextVertex);

        // Edge is in collision. Path will not be used.
        if (inCollision)
        {
          markEdgeInCollision(curVertex, nextVertex);

          collisionFree = false;
          break;
        }
      }

      // Edge is free, add it to the path.
      ompl::base::State* startState  = stateMap[curVertex];
      ompl::base::State* endState = stateMap[nextVertex];

      if (finalStates.size() == 0)
      {
        finalStates.push_back(startState);
        finalStates.push_back(endState);
      } else {
        finalStates.push_back(endState);
      }
    }

    // Fully valid path.
    if (collisionFree)
      return finalStates;
  }
}



}
