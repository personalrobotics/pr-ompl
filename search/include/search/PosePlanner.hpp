/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef SEARCH_POSE_PLANNER_HPP_
#define SEARCH_POSE_PLANNER_HPP_

// STL headers
#include <exception>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>

namespace search {

/// The OMPL Planner class that implements the algorithm.
class PosePlanner : public ::ompl::base::Planner
{
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit PosePlanner(const ::ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~PosePlanner(void);

  /// Setup the planner.
  void setup() override;

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(
      const ::ompl::base::ProblemDefinitionPtr& pdef) override;

  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ::ompl::base::PlannerStatus solve(
      const ompl::base::PlannerTerminationCondition& ptc);

  /// Clear the planner setup.
  void clear() override;

  /// Set the collision checking resolution along the edge.
  void setCollisionCheckResolution(double resolution);

  /// Get the connection radius of the graph.
  double getCollisionCheckResolution() const;

private:
  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// The underlying grid.
  const datastructures::GridPtr mGrid;

  /// Collision checking resolution for the edge.
  double mCollisionCheckResolution;

}; // PosePlanner

} // search

#endif // SEARCH_POSE_PLANNER_HPP_