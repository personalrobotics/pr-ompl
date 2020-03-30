# pr-ompl

A collection of planner implementations for the
[Open Motion Planning Library][ompl] written by the
[Personal Robotics Lab][pr].

Currently, this consists of:

* A variant of the RRTConnect planner that
supports both the EXTEND and CONNECT extension types for each tree (rooted
at the start(s) and goal(s)), which was written by Chris Dellin
(<cdellin@gmail.com>). There are also corresponding [OpenRAVE][openrave] 
bindings.

* An implementation of the NNF (Nearest-Neighbor Fréchet) motion planner,
which is used to follow end-effector paths (particularly in 
highly-constrained environments). Please see [this][nnf_paper] paper for more info.

Note that each planner is treated as a separate Catkin package. Additional planners 
will be added in the future following this convention.

## License

`pr-ompl` is licensed under a BSD license. See [LICENSE](./LICENSE) for more
information.

[pr]: https://personalrobotics.cs.washington.edu/
[ompl]: http://ompl.kavrakilab.org/
[openrave]: http://openrave.org/
[nnf_paper]: https://personalrobotics.cs.washington.edu/publications/niyaz2018surgicalpath.pdf
