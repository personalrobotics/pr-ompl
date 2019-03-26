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

void NNFrechet::buildTensorProductGraph()
{
  std::vector<Vertex> referenceVertices = getGraphVertices(mReferenceGraph);
  std::vector<Vertex> layerVertices = getGraphVertices(mNNGraph);

  std::cout << "[INFO]: Ref Graph has " << referenceVertices.size() << " nodes."
    << std::endl;
  std::cout << "[INFO]: NN Graph has " << layerVertices.size() << " nodes."
    << std::endl;

  // // Set up the cross product graph with the correct nodes.
  // addCrossProductNodes(
  //   referenceGraph,
  //   layeredGraph,
  //   referenceVertices,
  //   layerVertices,
  //   pathId,
  //   crossStartVertexName,
  //   crossEndVertexName,
  //   refPoseMap,
  //   layerPoseMap);
  //
  // // And connect them up to eachother.
  // connectCrossProductNodes(
  //   referenceGraph,
  //   layeredGraph,
  //   referenceVertices,
  //   layerVertices,
  //   pathId);
}



}