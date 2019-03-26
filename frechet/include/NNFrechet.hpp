#ifndef NN_FRECHET_HPP_
#define NN_FRECHET_HPP_

#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <exception>
#include <iostream>
#include <unordered_map>

#include "util.hpp"

namespace NNFrechet {

  /// The OMPL Planner class that implements the algorithm
  class NNFrechet: public ompl::base::Planner
  {
    std::default_random_engine mRandomGenerator;

    /// The pointer to the OMPL state space
    const ompl::base::StateSpacePtr mSpace;

    Graph mReferenceGraph;
    Graph mNNGraph;
    Graph mTensorProductGraph;

    // Map name of TPG Node -> Vertex.
    std::unordered_map<std::string, Vertex> mNameToVertex;

    // Reference path we were given.
    std::vector<Eigen::Isometry3d> mReferencePath;

    Vertex mRefStartNode;
    Vertex mRefGoalNode;

    Vertex mNNStartNode;
    Vertex mNNGoalNode;

    Vertex mTensorStartNode;
    Vertex mTensorGoalNode;

    int mNNIKID = 0;
    int mNNSubsampleID = 0;

    // Tensor-product bottleneck vertex.
    Vertex mBottleneckVertex;
    // Cost of found tensor-product path.
    double mBottleneckCost;

    // Parameter Related
    int mNumWaypoints;
    int mIKMultiplier;
    int mNumNN;
    int mDiscretization;

    // Function pointers that are set during creation for FK/IK.
    std::function<Eigen::Isometry3d(Eigen::VectorXd&)> mFkFunc;
    std::function<std::vector<Eigen::VectorXd>(Eigen::Isometry3d&, int)> mIkFunc;
    // Custom task space distance function used to calculate frechet distance.
    std::function<double(Eigen::Isometry3d&, Eigen::Isometry3d&)> mDistanceFunc;

    NNFrechet(
      const ompl::base::SpaceInformationPtr &si,
      std::vector<Eigen::Isometry3d>& referencePath,
      int numWaypoints,
      int ikMultiplier,
      int numNN,
      int discretization,
      int seed = 1);

    // Setters and getters.
    std::vector<Eigen::Isometry3d> subsampleRefPath(std::vector<Eigen::Isometry3d>& referencePath);
    void setFKFunc(std::function<Eigen::Isometry3d(Eigen::VectorXd&)> fkFunc);
    void setIKFunc(std::function<std::vector<Eigen::VectorXd>(Eigen::Isometry3d&, int)> ikFunc);
    void setDistanceFunc(std::function<double(Eigen::Isometry3d&, Eigen::Isometry3d&)> distanceFunc);

    // Graph construction methods.
    void buildReferenceGraph();

    std::vector<Vertex> sampleIKNodes(
      Eigen::Isometry3d& curWaypoint,
      int numSolutions);
    std::vector< std::vector<Vertex> > sampleNNGraphNodes(int numSamples);
    std::vector<Vertex> addSubsampledEdge(
      Vertex& firstNNVertex,
      Vertex& secondNNVertex);
    void buildNNGraph();

  };

} // namespace LRAstar
#endif // NN_FRECHET_HPP_
