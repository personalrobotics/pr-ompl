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
  , mReferencePath(referencePath)
  , mNumWaypoints(numWaypoints)
  , mIKMultiplier(ikMultiplier)
  , mNumNN(numNN)
  , mDiscretization(discretization)
{
  mRandomGenerator.seed(seed);
}

void NNFrechet::setFKFunc(
  std::function<Eigen::Isometry3d(Eigen::VectorXd&)> fkFunc
) {
  mFkFunc = fkFunc;
}

void NNFrechet::setIKFunc(
  std::function<std::vector<Eigen::VectorXd>(Eigen::Isometry3d&, std::vector<double>&)> ikFunc
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

  // Don't use the entire reference path. Subsample it.
  int numPts = ((mNumWaypoints - 1) * (mDiscretization + 1)) + 1;
  std::vector<int> sampledIDs = linIntSpace(0, mReferencePath.size() - 1, numPts);

  Vertex prevVertex;
  for (int i = 0; i < sampledIDs.size(); i++)
  {
    Vertex newRefVertex = add_vertex(mReferenceGraph);

    int sampledIndex = sampledIDs[i];
    Eigen::Isometry3d sampledPose = mReferencePath[sampledIndex];
    poseMap[newRefVertex] = sampledPose;

    std::string refVertexName = "r" + std::to_string(sampledIndex);
    nameMap[newRefVertex] = refVertexName;

    if (i > 0)
    {
      add_edge(prevVertex, newRefVertex, mReferenceGraph);
    }

    prevVertex = newRefVertex;

    // Record the first and last node on the reference graph.
    if (i == 0)
      mRefStartNode = newRefVertex;
    else if (i == sampledIDs.size() - 1)
      mRefGoalNode = newRefVertex;
  }
}



}
