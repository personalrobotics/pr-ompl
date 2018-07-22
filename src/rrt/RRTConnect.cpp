/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

/* Modifications to support:
 * - different extension types (e.g. EXTEND and CONNECT)
 *
 * Author: Chris Dellin <cdellin@gmail.com>
 * Copyright: 2015 Carnegie Mellon University
 * License: BSD
 */

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>

#include "rrt/RRTConnect.h"

using namespace ompl;
using namespace ompl::geometric;

pr_ompl::RRTConnect::RRTConnect(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTConnect")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    maxDistance_ = 0.0;
    ext_types_ = std::make_pair(EXTTYPE_EXTEND, EXTTYPE_CONNECT);

    Planner::declareParam<double>("range", this,
        &RRTConnect::setRange,
        &RRTConnect::getRange,
        "0.:1.:10000."
    );
    Planner::declareParam<std::string>("extension_type", this,
        &RRTConnect::setExtensionTypesString,
        &RRTConnect::getExtensionTypesString,
        "con-con,con-ext,ext-con,ext-ext"
    );
    connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);
}

pr_ompl::RRTConnect::~RRTConnect(void)
{
    freeMemory();
}

void pr_ompl::RRTConnect::setExtensionTypesString(std::string const &str)
{
    std::pair<enum ExtensionType,
              enum ExtensionType> extension_types;

    if (str == "con-con")
    {
        ext_types_ = std::make_pair(EXTTYPE_CONNECT, EXTTYPE_CONNECT);
    }
    else if (str == "con-ext")
    {
        ext_types_ = std::make_pair(EXTTYPE_CONNECT, EXTTYPE_EXTEND);
    }
    else if (str == "ext-con")
    {
        ext_types_ = std::make_pair(EXTTYPE_EXTEND, EXTTYPE_CONNECT);
    } else if (str == "ext-ext") {
        ext_types_ = std::make_pair(EXTTYPE_EXTEND, EXTTYPE_EXTEND);
    } else {
        throw std::runtime_error("Invalid extension type string.");
    }
}

std::string pr_ompl::RRTConnect::getExtensionTypesString() const
{
    std::stringstream ss;
    ss << getExtensionTypeString(ext_types_.first)
       << "-"
       << getExtensionTypeString(ext_types_.second);
    return ss.str();
}

std::string pr_ompl::RRTConnect::getExtensionTypeString(
    enum ExtensionType extension_type) const
{
    switch (extension_type) {
    case EXTTYPE_CONNECT:
        return "con";

    case EXTTYPE_EXTEND:
        return "ext";

    default:
        throw std::runtime_error("Unknown extension type.");
    }
}

void pr_ompl::RRTConnect::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    tStart_->setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, this, _1, _2));
    tGoal_->setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, this, _1, _2));
}

void pr_ompl::RRTConnect::freeMemory(void)
{
    std::vector<Motion*> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void pr_ompl::RRTConnect::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State*, base::State*>(NULL, NULL);
}

pr_ompl::RRTConnect::GrowState pr_ompl::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
        dstate = tgi.xstate;
        reach = false;
    }
    // if we are in the start tree, we just check the motion like we normally do;
    // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it receives as argument is valid,
    // so we check that one first
    bool validMotion = tgi.start ? si_->checkMotion(nmotion->state, dstate) : si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nmotion->state);

    if (validMotion)
    {
        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->root = nmotion->root;
        tgi.xmotion = motion;

        tree->add(motion);
        if (reach)
            return REACHED;
        else
            return ADVANCED;
    }
    else
        return TRAPPED;
}

ompl::base::PlannerStatus pr_ompl::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("Unknown type of goal (or goal undefined)");
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("Motion planning start tree could not be initialized!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("Insufficient states in sampleable goal region");
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("Starting with %d states", (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion   *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree      = true;
    bool solved         = false;

    while (ptc == false)
    {
        TreeData &tree      = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st)
            {
                Motion* motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("Unable to sample any valid states for goal tree");
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        /* used to track growth state */
        GrowState gs;

        /* first extension */
        tgi.xmotion = 0;
        do
        {
            gs = growTree(tree, tgi, rmotion);
        }
        while (gs == ADVANCED && ext_types_.first == EXTTYPE_CONNECT);

        /* perhaps we added nothing (TRAPPED immediately?) */
        if (!tgi.xmotion)
           continue;

        /* remember which motion was most recently added */
        Motion *addedMotion = tgi.xmotion;

        /* attempt to connect trees */

        /* if reached, it means we used rstate directly, no need top copy again */
        tgi.xmotion = 0;
        if (gs != REACHED)
            si_->copyState(rstate, addedMotion->state);

        tgi.start = startTree;
        do
        {
            gs = growTree(otherTree, tgi, rmotion);
        }
        while (gs == ADVANCED && ext_types_.second == EXTTYPE_CONNECT);

        Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
        Motion *goalMotion  = startTree ? addedMotion : tgi.xmotion;

        /* if we connected the trees in a valid way (start and goal pair is valid)*/
        if (gs == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {
            // it must be the case that either the start tree or the goal tree has made some progress
            // so one of the parents is not NULL. We go one step 'back' to avoid having a duplicate state
            // on the solution path
            if (startMotion->parent)
                startMotion = startMotion->parent;
            else
                goalMotion = goalMotion->parent;

            connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

            /* construct the solution path */
            Motion *solution = startMotion;
            std::vector<Motion*> mpath1;
            while (solution != NULL)
            {
                mpath1.push_back(solution);
                solution = solution->parent;
            }

            solution = goalMotion;
            std::vector<Motion*> mpath2;
            while (solution != NULL)
            {
                mpath2.push_back(solution);
                solution = solution->parent;
            }

            PathGeometric *path = new PathGeometric(si_);
            path->getStates().reserve(mpath1.size() + mpath2.size());
            for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
                path->append(mpath1[i]->state);
            for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
                path->append(mpath2[i]->state);

            pdef_->addSolutionPath(base::PathPtr(path), false, 0.0);
            solved = true;
            break;
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("Created %u states (%u start + %u goal)", tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void pr_ompl::RRTConnect::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (tStart_)
        tStart_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
                         base::PlannerDataVertex(motions[i]->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
                         base::PlannerDataVertex(motions[i]->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}
