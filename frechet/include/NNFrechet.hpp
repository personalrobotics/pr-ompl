#ifndef NN_FRECHET_HPP_
#define NN_FRECHET_HPP_

#include <exception>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "LPAStar.h"
#include "util.hpp"

namespace NNFrechet {

/// The OMPL Planner class that implements the NNF algorithm.
class NNFrechet : public ompl::base::Planner {
  /// Used to sample random points on the reference path to IK.
  std::default_random_engine mRandomGenerator;

  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// Super-sampled reference path encoded as a one-dimensional graph.
  Graph mReferenceGraph;

  /// Sampled nearest-neighbor graph in C-space. Composed of IK solutions for
  /// task-space points on the reference path.
  Graph mNNGraph;

  /// Tensor-product graph between the above two graphs. Used to compute the
  /// minimum-Frechet path on the NN Graph.
  Graph mTensorProductGraph;

  /// Map of TPG node's name -> TPG node.
  std::unordered_map<std::string, Vertex> mNameToVertex;

  /// Keep around EACH path's corresponding set of edges in the tensor product
  /// graph. By making this a map of [U_NN, V_NN] -> {E_TPG}, we can
  /// "knock out" edges quickly in the TPG that correspond to U -> V in the
  /// NN Graph. This is an *optimization* when handling a collision.
  std::unordered_map<std::string, std::vector<Edge>> mNNToTPGEdges;

  /// (Super-sampled version of) given reference path.
  std::vector<Eigen::Isometry3d> mReferencePath;

  /// Represent first and last waypoint on reference path.
  Vertex mRefStartNode;
  Vertex mRefGoalNode;

  /// Dummy start and target nodes a path on the NN Graph must start and end at.
  /// These are later stripped from the final motion plan.
  Vertex mNNStartNode;
  Vertex mNNGoalNode;

  /// Start node for our bottleneck search on the TPG. Product of
  /// \c mRefStartNode and \c mNNStartNode.
  Vertex mTensorStartNode;

  /// Target node for our bottleneck search on the TPG. Product of
  /// \c mRefGoalNode and \c mNNGoalNode.
  Vertex mTensorGoalNode;

  /// Increments so each sampled IK node in \c mNNGraph has a unique name.
  int mNNIKID = 0;

  /// Increments so each node in \c mNNGraph resulting from interpolation has a
  /// unique name.
  int mNNSubsampleID = 0;

  /// NOTE: NNF parameters.

  /// How many task-space waypoints to super-sample from the given reference
  /// path.
  int mNumWaypoints;

  /// Multiplied by \c mNumWaypoints to get the IK budget for building
  /// \c mNNGraph.
  int mIKMultiplier;

  /// How many nearest-neighbors to connect to in \c mNNGraph.
  int mNumNN;

  /// How finely edges in \c mNNGraph should be sub-sampled to increase accuracy
  /// of Frechet computation.
  int mDiscretization;

  /// Default resolution to collision-check in C-space.
  double mCheckResolution = 0.05;

  /// NOTE: Fields to record planning stats.

  /// Frechet cost of found tensor-product path.
  double mFinalError;

  /// Build times for specific graphs.
  double mBuildNNTime = 0;
  double mBuildTesnorTime = 0;

  /// Total time spent on \c setup() (all 3 graphs + LPA*).
  double mInitStructuresTime = 0;

  /// Total time spent on \c solve() (i.e. LazySP).
  double mSearchTime = 0;

  /// LPA* object used to quickly rewire path in TPG after edge is marked in
  /// collision. Searche for a minimum-bottleneck path on the TPG.
  std::shared_ptr<LPAStar> mLPAStar;

  /// NOTE: Function pointers that are set externally for FK, IK, and distance
  /// in task-space.

  /// FK function that returns the end-effector pose that a configuration
  /// would induce.
  std::function<Eigen::Isometry3d(ompl::base::State *)> mFkFunc;

  /// IK function that returns n inverse kinematics solutions for the given
  /// waypoint in task-space.
  std::function<std::vector<ompl::base::State *>(Eigen::Isometry3d &, int)>
      mIkFunc;

  /// Computes the distance between two poses in task-space. Used to compute
  /// the Frechet error of a path in \c mNNGraph via the tensor-product graph.
  std::function<double(Eigen::Isometry3d &, Eigen::Isometry3d &)> mDistanceFunc;

public:
  NNFrechet(const ompl::base::SpaceInformationPtr &si);

  NNFrechet(const ompl::base::SpaceInformationPtr &si, int numWaypoints,
            int ikMultiplier, int numNN, int discretization, int seed = 1);

  // Setters and getters. These *must* be called before setup().
  std::vector<Eigen::Isometry3d>
  subsampleRefPath(std::vector<Eigen::Isometry3d> &referencePath);
  void setRefPath(std::vector<Eigen::Isometry3d> &referencePath);
  void setFKFunc(std::function<Eigen::Isometry3d(ompl::base::State *)> fkFunc);
  void setIKFunc(
      std::function<std::vector<ompl::base::State *>(Eigen::Isometry3d &, int)>
          ikFunc);
  void setDistanceFunc(
      std::function<double(Eigen::Isometry3d &, Eigen::Isometry3d &)>
          distanceFunc);

  // Setters that caller is not required to use, but that can be used to
  // alter NNF params before setup() constructs the three graphs.
  void setNumWaypoints(int numWaypoints);
  void setIKMultiplier(int ikMultiplier);
  void setNumNN(int numNN);
  void setDiscretization(int discretization);
  void setRandomSeed(int seed);

  // Graph construction methods.
  void buildReferenceGraph();

  std::vector<Vertex> sampleIKNodes(Eigen::Isometry3d &curWaypoint,
                                    int numSolutions);
  std::vector<std::vector<Vertex>> sampleNNGraphNodes(int numSamples);
  void addSubsampledEdge(Vertex &firstNNVertex, Vertex &secondNNVertex);
  void buildNNGraph();

  void addTensorProductNodes(std::vector<Vertex> &refNodes,
                             std::vector<Vertex> &nnNodes);
  void addTensorProductEdge(Vertex &v1, Vertex &v2);
  void connectTensorProductNodes(std::vector<Vertex> &refNodes,
                                 std::vector<Vertex> &nnNodes);
  void buildTensorProductGraph();

  // Extract the corresponding NN Graph path from a path in the TPG.
  std::vector<Vertex> extractNNPath(std::vector<Vertex> &tensorProductPath);
  // Check if lazy SP has already evaluated an NN Graph edge.
  bool checkEdgeEvaluation(Vertex &source, Vertex &target);
  // Core collision check.
  bool evaluateEdge(Vertex &source, Vertex &target);
  void markEdgeInCollision(Vertex &nnU, Vertex &nnV);

  // OMPL required methods
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);
  ompl::base::PathPtr constructSolution(std::vector<Vertex> &nnPath);
  // Use LazySP to search for a collision free path that minimizes Frechet.
  ompl::base::PlannerStatus
  solve(const ompl::base::PlannerTerminationCondition &ptc);
  ompl::base::PlannerStatus solve(double solveTime);
  // Inits all three graphs, as well as LPA* structures.
  void setup();
};

} // namespace LRAstar
#endif // NN_FRECHET_HPP_
