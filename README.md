# graphopt
A wrapper around a modified version of Michael Kaess's Incremental Smoothing and Mapping (ISAM) **graph opt**imization library. The original ISAM library can be found [here](http://people.csail.mit.edu/kaess/isam/), and the modified version used in this wrapper can be found [here](https://github.com/Humhu/isam).

# Dependencies
* libsuitesparse: package for sparse linear algebra
* [argus_utils](https://github.com/Humhu/argus_utils): utilities for all argus packages

# Concepts
## Graph optimization
ISAM is a graph optimization framework, meaning that it organizes an optimization into a graphical structure that represents the relations between observed data and unknown quantities. Graphs are typically composed of *nodes* and *edges*, but the graph optimization uses slightly different vernacular. **Nodes** represent variables, either known or estimated, in the optimization. **Factors** connect variables like edges, but also encode or contain observed data. When we optimize a graph, we find the values of the nodes that maximize the likelihood (specifically, the log likelihood) of all of the factors in the graph.

# Extensions to ISAM
## Integration with argus PoseSE3
Graphopt wraps the PoseSE3 object in argus_utils (which is itself a wrapper around [sophus](https://github.com/strasdat/Sophus)) in its SE3 slam representation, simplifying use of SE3 SLAM types in the argus ecosystem. 

*Reference: slamse3.h*

## Fiducial Simultaneous Calibration Localization and Mapping (SCLAM)
We also extend the ISAM monocular types to enable generic sparse (point representation) fiducial Simultaneous Calibration Localization and Mapping (SCLAM), where fiducial point observations can be used to estimate both camera and fiducial intrinsics and extrinsics relative to a moving reference frame, as well as the pose of the reference frame.

*Reference: sclam_fiducial.h*

# Classes
## GraphOptimizer
Wraps an instance of isam::Slam, providing methods for reading parameters from ROS as well as interacting with std::shared_ptr smart pointers instead of ISAM's raw pointer interfaces. 

*Reference: GraphOptimizer.h*
