Evaluating pcl/registration
---------------------------

This is a collection of ideas on how to build an evaluation framework of pcl/registration.

Data generation
===============
- synthetic data
- real word data (how to get ground truth?)
  - Kinect
  - PR2 laser scanner
  - SICK laser data
  - small range 3D scanner
  - mid range 3D scanner (Faro)
  - high end 3D scanner (Riegl, Velodyne)
- Point Types
  - 2D(?)
  - 3D
  - RGB
- dynamics
  - static scans
  - scanning while driving (e.g. robots)
- size
  - room
  - building
  - outdoor (street)

Architecture
============
- some lib for polygonal data
- modeling different sensors
- modeling noise
- add a trajectory file
- output a pile of .pcd files
- integrate command line tools from PCL grandfather

Evaluating different algorithms
===============================

ICP
^^^
- how does the algorithm cope with outliers
- how are the point pairs evaluated:

  - does it use normal or RGB information
  - does it weight the pairs differently
  - which kind of point pairs are used:

    - one-to-one
    - one-to-many
    - many-to-many

Similar Projects
================
- `GICP <http://stanford.edu/~avsegal/resources/papers/Generalized_ICP.pdf>`_
- Gazebo
- `slam benchmarking <http://kaspar.informatik.uni-freiburg.de/~slamEvaluation/index.php>`_
- `Automated SLAM Evaluation <http://slameval.willowgarage.com/workshop/>`_
