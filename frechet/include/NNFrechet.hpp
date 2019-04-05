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
#include "LPAStar.h"

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

    // Keep around EACH path's corresponding set of edges in the tensor product
    // graph. By making this a map of [U_NN, V_NN] -> {E_TPG}, we can
    // "knock out" edges quickly in the TPG that correspond to U -> V in the
    // NN Graph. This is an *optimization* when handling a collision.
    std::unordered_map< std::string, std::vector<Edge> > mNNToTPGEdges;

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
    // TODO: Set this intelligently.
    int mCheckResolution = 0.05;

    LPAStar mLPAStar;

    // Function pointers that are set externally for FK/IK.
    std::function<Eigen::Isometry3d(Eigen::VectorXd&)> mFkFunc;
    std::function<std::vector<Eigen::VectorXd>(Eigen::Isometry3d&, int)> mIkFunc;
    // Custom task space distance function used to calculate frechet distance.
    std::function<double(Eigen::Isometry3d&, Eigen::Isometry3d&)> mDistanceFunc;

    NNFrechet(const ompl::base::SpaceInformationPtr &si);

    NNFrechet(
      const ompl::base::SpaceInformationPtr &si,
      int numWaypoints,
      int ikMultiplier,
      int numNN,
      int discretization,
      int seed = 1);

    // Setters and getters.
    std::vector<Eigen::Isometry3d> subsampleRefPath(std::vector<Eigen::Isometry3d>& referencePath);
    void setRefPath(std::vector<Eigen::Isometry3d>& referencePath);
    void setFKFunc(std::function<Eigen::Isometry3d(Eigen::VectorXd&)> fkFunc);
    void setIKFunc(std::function<std::vector<Eigen::VectorXd>(Eigen::Isometry3d&, int)> ikFunc);
    void setDistanceFunc(std::function<double(Eigen::Isometry3d&, Eigen::Isometry3d&)> distanceFunc);

    // Graph construction methods.
    void buildReferenceGraph();

    std::vector<Vertex> sampleIKNodes(
      Eigen::Isometry3d& curWaypoint,
      int numSolutions);
    std::vector< std::vector<Vertex> > sampleNNGraphNodes(int numSamples);
    void addSubsampledEdge(
      Vertex& firstNNVertex,
      Vertex& secondNNVertex);
    void buildNNGraph();

    void addTensorProductNodes(
      std::vector<Vertex>& refNodes,
      std::vector<Vertex>& nnNodes);
    void addTensorProductEdge(Vertex& v1, Vertex& v2);
    void connectTensorProductNodes(
      std::vector<Vertex>& refNodes,
      std::vector<Vertex>& nnNodes);
    void buildTensorProductGraph();

    // Extract the corresponding NN Graph path from a path in the TPG.
    std::vector<Vertex> extractNNPath(std::vector<Vertex>& tensorProductPath);
    // Check if lazy SP has already evaluated an NN Graph edge.
    bool checkEdgeEvaluation(Vertex& source, Vertex& target);
    // Core collision check.
    bool evaluateEdge(Vertex& source, Vertex& target);
    void markEdgeInCollision(Vertex& nnU, Vertex& nnV);

    // OMPL required methods
    void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef);
    ompl::base::PathPtr constructSolution(std::vector<Vertex>& nnPath);
    // Use LazySP to search for a collision free path that minimizes Frechet.
    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc);
    ompl::base::PlannerStatus solve(double solveTime);
    // Inits all three graphs, as well as LPA* structures.
    void setup();
  };

} // namespace LRAstar
#endif // NN_FRECHET_HPP_
