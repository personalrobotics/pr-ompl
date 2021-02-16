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

#ifndef PR_OMPL_RRT_CONNECT_H_
#define PR_OMPL_RRT_CONNECT_H_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
 #include "ompl/util/Exception.h"

namespace pr_ompl {

/**
   @anchor gRRTC
   @par Short description
   The basic idea is to grow to RRTs, one from the start and
   one from the goal, and attempt to connect them.
   @par External documentation
   J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844730">10.1109/ROBOT.2000.844730</a><br>
   <a href="http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246">[PDF]</a>
   <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
*/

/** \brief RRT-Connect (RRTConnect) */
class RRTConnect : public ompl::base::Planner
{
public:
    enum ExtensionType
    {
        EXTTYPE_EXTEND,
        EXTTYPE_CONNECT
    };

    /** \brief Constructor */
    RRTConnect(const ompl::base::SpaceInformationPtr &si);

    virtual ~RRTConnect(void);

    virtual void getPlannerData(ompl::base::PlannerData &data) const;

    virtual ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

    virtual void clear(void);

    /** \brief Set the range the planner is supposed to use.

        This parameter greatly influences the runtime of the
        algorithm. It represents the maximum length of a
        motion to be added in the tree of motions. */
    void setRange(double distance)
    {
        maxDistance_ = distance;
    }

    /** \brief Get the range the planner is using */
    double getRange(void) const
    {
        return maxDistance_;
    }

    void setExtensionTypes(
        std::pair<enum ExtensionType, enum ExtensionType> const &extension_types)
    {
        ext_types_ = extension_types;
    }

    std::pair<enum ExtensionType, enum ExtensionType> getExtensionTypes()
    {
        return ext_types_;
    }

    void setGoalSamplingProbability(double prob)
    {
        if( prob < 0.0 || prob > 1.0)
        {
            throw ompl::Exception("Goal sampling probability must be between [0, 1]");
        }

        goalSamplingProbability_ = prob;
    }

    double getGoalSamplingProbability(void) const
    {
        return goalSamplingProbability_;
    } 

    /** \brief Set a different nearest neighbors datastructure */
    template<template<typename T> class NN>
    void setNearestNeighbors(void)
    {
        tStart_.reset(new NN<Motion*>());
        tGoal_.reset(new NN<Motion*>());
    }

    virtual void setup(void);

protected:
    void setExtensionTypesString(std::string const &str);

    std::string getExtensionTypesString() const;
    std::string getExtensionTypeString(enum ExtensionType extension_type) const;

    /** \brief Representation of a motion */
    class Motion
    {
    public:

        Motion(void) : root(NULL), state(NULL), parent(NULL)
        {
            parent = NULL;
            state  = NULL;
        }

        Motion(const ompl::base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL)
        {
        }

        ~Motion(void)
        {
        }

        const ompl::base::State *root;
        ompl::base::State       *state;
        Motion            *parent;

    };

    /** \brief A nearest-neighbor datastructure representing a tree of motions */
    typedef boost::shared_ptr< ompl::NearestNeighbors<Motion*> > TreeData;

    /** \brief Information attached to growing a tree of motions (used internally) */
    struct TreeGrowingInfo
    {
        ompl::base::State         *xstate;
        Motion              *xmotion;
        bool                 start;
    };

    /** \brief The state of the tree after an attempt to extend it */
    enum GrowState
        {
            /// no progress has been made
            TRAPPED,
            /// progress has been made towards the randomly sampled state
            ADVANCED,
            /// the randomly sampled state was reached
            REACHED
        };

    /** \brief Free the memory allocated by this planner */
    void freeMemory(void);

    /** \brief Compute distance between motions (actually distance between contained states) */
    double distanceFunction(const Motion* a, const Motion* b) const
    {
        return si_->distance(a->state, b->state);
    }

    /** \brief Grow a tree towards a random state */
    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

    /** \brief State sampler */
    ompl::base::StateSamplerPtr         sampler_;

    /** \brief The start tree */
    TreeData                      tStart_;

    /** \brief The goal tree */
    TreeData                      tGoal_;

    /** \brief The maximum length of a motion to be added to a tree */
    double                        maxDistance_;

    /** \brief The goal sampling probability value */
    double                        goalSamplingProbability_;
    
    std::pair<enum ExtensionType,enum ExtensionType> ext_types_;

    /** \brief The random number generator */
    ompl::RNG                           rng_;

    /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
    std::pair<ompl::base::State*, ompl::base::State*>      connectionPoint_;
};

}

#endif
