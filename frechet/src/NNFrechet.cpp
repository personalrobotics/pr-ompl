#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <iostream>         // std::cerr
#include <queue>            // std::priority_queue
#include <set>              // std::set
#include <unordered_set>    // std::unordered_set
#include <fstream>          // Write to file
#include <assert.h>         // Debug
#include <chrono>           // record rewireTime

#include "util.hpp"
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
    auto ikState = mSpace->allocState();
    for (int i = 0; i < mSpace->getDimension(); i++)
    {
      jointStates.push_back(curSol[i]);
    }
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
  // TODO: Rest of this.
  // std::vector<Vertex> firstWaypointVertices = addNewIkNodesStrict(
  //   diskGraph,
  //   firstWaypoint,
  //   numEndpointSamples,
  //   0, // When sampling IK in disk graph, waypoint id/layer index are the same.
  //   0,
  //   diskGraphPoseMap,
  //   mUseLazyIK);
  //
  // // NOTE: In this case, there's no point going on. Just escape.
  // if (firstWaypointVertices.size() == 0)
  // {
  //   return;
  // }
  //
  // for (Vertex firstWaypointNode : firstWaypointVertices)
  // {
  //   add_edge(dummyStartVertex, firstWaypointNode, diskGraph);
  //   mSampledNNGraphNodes.at(0).push_back(firstWaypointNode);
  // }
  //
  // // NOTE: Now repeat eveything we did, but for the last waypoint on the
  // // reference path.
  // PoseType lastWaypoint = referencePoses.at(referencePoses.size() - 1);
  // std::vector<Vertex> lastWaypointVertices = addNewIkNodesStrict(
  //   diskGraph,
  //   lastWaypoint,
  //   numEndpointSamples,
  //   mSampledNNGraphNodes.size() - 1,
  //   mSampledNNGraphNodes.size() - 1,
  //   diskGraphPoseMap,
  //   mUseLazyIK);
  //
  // for (Vertex lastWaypointNode : lastWaypointVertices)
  // {
  //   mSampledNNGraphNodes.at(mSampledNNGraphNodes.size() - 1).push_back(lastWaypointNode);
  //
  //   add_edge(lastWaypointNode, dummyEndVertex, diskGraph);
  // }
  //
  // int remainingIKBudget = numWaypoints * numIK;
  // // We already used some of this for the end points.
  // remainingIKBudget = remainingIKBudget - 2*numEndpointSamples;
  //
  // // Sample nodes for waypoints besides the endpoints of the reference path.
  // auto sampledNodes = sampleNNGraphNodes(
  //   remainingIKBudget,
  //   diskGraph,
  //   referencePoses,
  //   diskGraphPoseMap);
  //
  // // Update mSampledNNGraphNodes with the new nodes that were sampled.
  // for (int refIndex = 0; refIndex < sampledNodes.size(); refIndex++)
  // {
  //   for (auto indexNewNode : sampledNodes.at(refIndex))
  //     mSampledNNGraphNodes.at(refIndex).push_back(indexNewNode);
  // }
  //
  // // Connect each raodmap node to its kNN in C-Space that are sampled *further*
  // // down the reference path (to ensure monotonicity).
  // for (int refIndex = 0; refIndex < mSampledNNGraphNodes.size(); refIndex++)
  // {
  //   for (auto& nodeSampledAtWaypoint : mSampledNNGraphNodes.at(refIndex))
  //   {
  //     std::vector<Vertex> deeperNodes = flattenVertexList(
  //       mSampledNNGraphNodes,
  //       refIndex + 1,
  //       mSampledNNGraphNodes.size() - 1);
  //
  //     // Get neighbors sampled from waypoints further along the reference path.
  //     std::vector<Vertex> cSpaceNeighbors = findKnnNodes(
  //       nodeSampledAtWaypoint,
  //       deeperNodes,
  //       diskGraph,
  //       mCSpaceDistFunc,
  //       mNumNearestNeighbors);
  //
  //     for (auto& closeNode : cSpaceNeighbors)
  //     {
  //       addSubsampledEdge(
  //         diskGraph,
  //         nodeSampledAtWaypoint,
  //         closeNode,
  //         diskGraphPoseMap,
  //         // NOTE: This isn't perfect, but trying to match the discretization
  //         // of the reference path and roadmap paths exactly can prove so
  //         // expensive that it's really not worth it.
  //         discretization,
  //         // Remember, ref path indices and layer indices are the same now!
  //         refIndex,
  //         mUseLazyIK);
  //     }
  //   }
  // }
}



}
