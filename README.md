# pr-ompl

A collection of planner implementations for the
[Open Motion Planning Library][ompl] written by the
[Personal Robotics Lab][pr], as well as [OpenRAVE][openrave] bindings
thereof.

Currently, this consists of a variant of the RRTConnect planner which
supports both the EXTEND and CONNECT extension types for each tree (rooted
at the start(s) and goal(s)) which was written by Chris Dellin
(<cdellin@gmail.com>).

Once these planners are well-tested, we intend to submit them upstream.

In general, we try to keep the `pr_ompl` package free of dependencies, so it
is low-cost to pull it in to use one of its simple planners.  Planners with
dependencies should have their own packages.

[pr]: https://personalrobotics.ri.cmu.edu/
[ompl]: http://ompl.kavrakilab.org/
[openrave]: http://openrave.org/
