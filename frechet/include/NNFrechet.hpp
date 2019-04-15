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
  /// Construct the NNF planner with default params.
  ///
  /// \param[in] si OMPL state-space struct.
  NNFrechet(const ompl::base::SpaceInformationPtr &si);

  /// Construct the NNF planner with custom params.
  ///
  /// \param[in] si OMPL state-space struct.
  /// \param[in] numWaypoints Sets \c mNumWaypoints.
  /// \param[in] ikMultiplier Sets \c mIKMultiplier.
  /// \param[in] numNN Sets \c mNumNN.
  /// \param[in] discretization Sets \c mDiscretization.
  /// \param[in] seed Set random seed for \c mRandomGenerator.
  NNFrechet(const ompl::base::SpaceInformationPtr &si, int numWaypoints,
            int ikMultiplier, int numNN, int discretization, int seed = 1);

  /// Take given reference path and super-sample according to \c mNumWaypoints
  /// and \c mDiscretization.
  ///
  /// \param[in] referencePath Input reference path.
  std::vector<Eigen::Isometry3d>
  subsampleRefPath(std::vector<Eigen::Isometry3d> &referencePath);

  /// NOTE: Setters and getter that *must* be used before \c setup() is called.

  /// Sets the reference path to follow with end-effector.
  ///
  /// \param[in] referencePath Input reference path.
  void setRefPath(std::vector<Eigen::Isometry3d> &referencePath);

  /// Sets the function used to compute forward kinematics, and thus edge
  /// weights of \c mTensorProductGraph.
  ///
  /// \param[in] fkFunc Set FK function.
  void setFKFunc(std::function<Eigen::Isometry3d(ompl::base::State *)> fkFunc);

  /// Sets the function used to compute inverse kinematics, and thus sample
  /// nodes of \c mNNGraph.
  ///
  /// \param[in] ikFunc Set IK function.
  void setIKFunc(
      std::function<std::vector<ompl::base::State *>(Eigen::Isometry3d &, int)>
          ikFunc);

  /// Sets the function used to compute distance between two poses in
  /// task-space, and thus the edge weights of \c mTensorProductGraph.
  ///
  /// \param[in] distanceFunc Set task-space distance function.
  void setDistanceFunc(
      std::function<double(Eigen::Isometry3d &, Eigen::Isometry3d &)>
          distanceFunc);

  /// NOTE: Setters that caller is not required to use, but that can be used to
  /// alter NNF params before setup() constructs the three graphs.

  /// Set \c mNumWaypoints before \c setup().
  ///
  /// \param[in] numWaypoints Sets \c mNumWaypoints.
  void setNumWaypoints(int numWaypoints);

  /// Set \c mIKMultiplier before \c setup().
  ///
  /// \param[in] ikMultiplier Sets \c mIKMultiplier.
  void setIKMultiplier(int ikMultiplier);

  /// Set \c mNumNN before \c setup().
  ///
  /// \param[in] numNN Sets \c mNumNN.
  void setNumNN(int numNN);

  /// Set \c mDiscretization before \c setup().
  ///
  /// \param[in] discretization Sets \c mDiscretization.
  void setDiscretization(int discretization);

  /// Sets random seed for \c mRandomGenerator before \c setup().
  ///
  /// \param[in] seed Sets random seed for \c mRandomGenerator.
  void setRandomSeed(int seed);

  /// NOTE: Graph construction methods.

  /// Builds \c mReferenceGraph from \c mReferencePath.
  void buildReferenceGraph();

  /// Sample \c numSolutions IK solutions using \c mIkFunc for the point
  /// \c curWaypoint and add the corresponding nodes to \mNNGraph.
  ///
  /// \param[in] curWaypoint Task-space point to sample IK for.
  /// \param[in] numSolutions Number of solutions to sample.
  std::vector<Vertex> sampleIKNodes(Eigen::Isometry3d &curWaypoint,
                                    int numSolutions);

  /// Unifromly sample \c numSamples IK solutions for points on
  /// \c mReferencePath using \c sampleIKNodes.
  ///
  /// \param[in] numSamples Number of solutions to sample.
  std::vector<std::vector<Vertex>> sampleNNGraphNodes(int numSamples);

  /// Add sub-sampled edge between two nodes in /c mNNGraph.
  ///
  /// \param[in] firstNNVertex Edge source in /c mNNGraph.
  /// \param[in] secondNNVertex Edge target in /c mNNGraph.
  void addSubsampledEdge(Vertex &firstNNVertex, Vertex &secondNNVertex);

  /// Use the above helpers to sample \c mNNGraph from \c mReferencePath.
  void buildNNGraph();

  /// Initialize node of \c mTensorProductGraph.
  ///
  /// \param[in] refNodes Passed nodes of \c mReferenceGraph.
  /// \param[in] nnNodes Passed nodes of \c mNNGraph.
  void addTensorProductNodes(std::vector<Vertex> &refNodes,
                             std::vector<Vertex> &nnNodes);

  /// Add edge between two nodes of \c mTensorProductGraph.
  ///
  /// \param[in] v1 Edge source in /c mTensorProductGraph.
  /// \param[in] v2 Edge target in /c mTensorProductGraph.
  void addTensorProductEdge(Vertex &v1, Vertex &v2);

  /// Initialize edges of \c mTensorProductGraph.
  ///
  /// \param[in] refNodes Passed nodes of \c mReferenceGraph.
  /// \param[in] nnNodes Passed nodes of \c mNNGraph.
  void connectTensorProductNodes(std::vector<Vertex> &refNodes,
                                 std::vector<Vertex> &nnNodes);

  /// Use the above helpers to build \c mTensorProductGraph out of
  /// \c mReferenceGraph and \c mNNGraph.
  void buildTensorProductGraph();

  /// NOTE: Helpers related to LazySP and graph search.

  /// Extract the corresponding path in \c mNNGraph from a path in
  /// \c mTensorProductGraph.
  ///
  /// \param[in] tensorProductPath Found path on \c mTensorProductGraph.
  std::vector<Vertex> extractNNPath(std::vector<Vertex> &tensorProductPath);

  /// Check if lazySP has already evaluated an edge in \c mNNGraph.
  ///
  /// \param[in] source Source of edge to check in /c mNNGraph.
  /// \param[in] target Target of edge to check in /c mNNGraph.
  bool checkEdgeEvaluation(Vertex &source, Vertex &target);

  /// Collision-check an edge of \c mNNGraph.
  ///
  /// \param[in] source Source of edge to check in /c mNNGraph.
  /// \param[in] target Target of edge to check in /c mNNGraph.
  bool evaluateEdge(Vertex &source, Vertex &target);

  /// Mark an edge of \c mNNGraph as being in collision. Also sets all
  /// *corresponding* nodes of \c mTensorProductGraph to have infinite weight,
  /// and update \c mLPAStar accordingly.
  ///
  /// \param[in] nnU Source of edge to mark in /c mNNGraph.
  /// \param[in] nnV Target of edge to mark in /c mNNGraph.
  void markEdgeInCollision(Vertex &nnU, Vertex &nnV);

  /// NOTE: OMPL required methods.

  /// Mark an edge of \c mNNGraph as being in collision. Also sets all
  /// *corresponding* nodes of \c mTensorProductGraph to have infinite weight,
  /// and update \c mLPAStar accordingly.
  ///
  /// \param[in] nnU Source of edge to mark in /c mNNGraph.
  /// \param[in] nnV Target of edge to mark in /c mNNGraph.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);

  /// Helper that converts a Boost graph path on \c mNNGraph into an OMPL path
  /// to return out.
  ///
  /// \param[in] nnPath Found path on \c mNNGraph.
  ompl::base::PathPtr constructSolution(std::vector<Vertex> &nnPath);

  /// Use LazySP to search for a collision free path that minimizes Frechet.
  ///
  /// \param[in] ptc OMPL termination condition.
  ompl::base::PlannerStatus
  solve(const ompl::base::PlannerTerminationCondition &ptc);

  /// Use LazySP to search for a collision free path that minimizes Frechet with
  /// a given timeout. Converts \c solveTime and calls the above method.
  ///
  /// \param[in] solveTime Time converted into an OMPL termination condition.
  ompl::base::PlannerStatus solve(double solveTime);

  /// Inits all three graphs, as well as LPA* structures.
  void setup();
};

} // namespace LRAstar
#endif // NN_FRECHET_HPP_
