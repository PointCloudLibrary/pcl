PCL/registration
================

Participants
------------
- Michael Dixon
- Radu Rusu
- Nicola Fioraio
- Jochen Sprickerhof

Existing Frameworks
-------------------
- SLAM6D
- Toro
- Hogman
- G2O
- MegaSLAM/MegaICP

Mission
-------
Provide a common interface/architecture for all of these and future SLAM ideas.

Ideas
-----
- Separate algorithms from data structures.
- strip down everything to it's basics and define an interface.
- modify data structure in algorithms (you can copy them before if you need to).
- point clouds are not transformed, only the translation and rotation is updated.

Data structures
---------------
.. note::
  These ideas are independent of actual data structures in the PCL for now. We can see later how to integrate them best.

Pose
^^^^
.. code-block:: c++

   struct Pose
   {
     Eigen::Vector3 translation;
     Eigen::Quaternion rotation;
   }

PointCloud
^^^^^^^^^^
.. code-block:: c++

   typedef vector<vector <float> > Points;

PosedPointCloud
^^^^^^^^^^^^^^^^^
.. code-block:: c++

   typedef pair<Pose*, PointCloud*> PosedPointCloud;

PointCloud* can be 0.

Graph
^^^^^
This should hold the SLAM graph. I would propose to use Boost::Graph for it, as it allows us to access a lot of algorithms.

.. note::

   define abstract structure.

CovarianceMatrix
^^^^^^^^^^^^^^^^
.. code-block:: c++

   typedef Eigen::Matrix4f CovarianceMatrix;

Measurement
^^^^^^^^^^^
.. code-block:: c++

   struct Measurement
   {
     Pose pose;
     CovarianceMatrix covariance;
   }

Idea: change the CovarianceMatrix into a function pointer.

Interfaces
----------

GlobalRegistration
^^^^^^^^^^^^^^^^^^
.. code-block:: c++

   class GlobalRegistration
   {
     public:
       /**
         * \param history how many poses should be cached (0 means all)
         */
       GlobalRegistration (int history = 0) : history_(history) {}

       /**
         * \param pc a new point cloud for GlobalRegistration
         * \param pose the initial pose of the pc, could be 0 (unknown)
         */
       void addPointCloud (PointCloud &pc, Pose &pose = 0)
       {
         new_clouds_.push_back (std::make_pair (pc, pose));
       }

       /**
         * returns the current estimate of the transformation from point cloud from to point cloud to
           throws an exception if the transformation is unknown
         */
       Pose getTF (PointCloud &from, PointCloud &to);

       /**
         * run the optimization process
         * \param lod the level of detail (optional). Roughly how long it should run (TODO: better name/parametrization?)
         */
       virtual void compute (int lod = 0) {}

     private:
       int history_;
       map<PointCloud*, Pose*> poses_;
       PosedPointCloud new_clouds_;
   };

This will be the base class interface for every SLAM algorithm. At any point you can add point clouds and they will be processed.
The poses can be either in a global or in a local coordinate system (meaning that they are incremental regarding the last one).
Idea: Do we need the compute? Could it be included into the addPointCloud or getTF?

GraphOptimizer
^^^^^^^^^^^^^^
.. code-block:: c++

   class GraphOptimizer
   {
     public:
       virtual void optimize (Graph &gr) = 0;
   }

LoopDetection
^^^^^^^^^^^^^
.. code-block:: c++

   class LoopDetection
   {
     public:
       virtual ~LoopDetection() = default;
       virtual list<std::pair<PointCloud*, PointCloud*> > detectLoop(list<PosedPointCloud*> poses, list<PosedPointCloud*> query) {} = 0;
   }

GraphHandler
^^^^^^^^^^^^
.. code-block:: c++

   class GraphHandler
   {
     void addPose (Graph &gr, PointCloud &pc);
     void addConstraint (Graph &gr, PointCloud &from, PointCloud &to, Pose &pose);
   }

.. note::

   I'm not sure about this one.


Example Implementations
-----------------------

PairwiseGlobalRegistration
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: c++

   class PairwiseGlobalRegistration : public GlobalRegistration
   {
     public:
       PairwiseGlobalRegistration(Registration &reg) : reg_(reg) {}
       virtual void compute (int lod = 0) {}
       {
         list<PosedPointCloud >::iterator cloud_it;
         for (cloud_it = new_clouds_.begin(); cloud_it != new_clouds_.end(); cloud_it++)
         {
           if(!old_) {
             old = *cloud_it;
             continue;
           }
           reg_.align(old_, *cloud_it, transformation);
           poses[*cloud_it] = transformation;
           old_ = *cloud_it;
         }
         new_clouds_.clear();
       }

     private:
       Registration &reg_;
       PointCloud &old_;
   }

DistanceLoopDetection
^^^^^^^^^^^^^^^^^^^^^
.. code-block:: c++

   class DistanceLoopDetection : LoopDetection
   {
     public:
       virtual list<std::pair<PointCloud*, PointCloud*> > detectLoop(list<PosedPointCloud*> poses, list<PosedPointCloud*> query)
       {
         //I want a map reduce here ;)
         list<PosedPointCloud >::iterator poses_it;
         for (poses_it = poses.begin(); poses_it != poses.end(); poses_it++)
         {
           list<PosedPointCloud >::iterator query_it;
           for (query_it = query.begin(); query_it != query.end(); query_it++)
           {
             if (dist (*poses_it, *query_it) < min_dist_)
             {
               //..
             }
         }

       }

   }

ELCH
^^^^
.. code-block:: c++

   class ELCH : public GlobalRegistration
   {
     public:
       ELCH(GlobalRegistration &initial_optimizer = PairwiseGlobalRegistration(), LoopDetection &loop_detection, GraphOptimizer &loop_optimizer, GraphOptimizer &graph_optimizer = LUM())
   }

LUM
^^^
.. code-block:: c++

   class ELCH : public GlobalRegistration
   {
     public:
       ELCH(GlobalRegistration &initial_optimizer = PairwiseGlobalRegistration(), LoopDetection &loop_detection, GraphOptimizer &loop_optimizer, GraphOptimizer &graph_optimizer)
   }

Lu and Milios style scan matching (as in SLAM6D)
