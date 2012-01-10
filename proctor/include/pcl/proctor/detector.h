#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <pcl/features/feature.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "proctor/config.h"
#include "proctor/timer.h"
#include "proctor/database_entry.h"

using std::vector;
using std::stringstream;
using std::auto_ptr;
using std::exception;
using std::numeric_limits;

namespace pcl {
  namespace proctor {
    struct Scene;

    class Proposer;

    class Detector
    {
      public:
        typedef FPFHSignature33 Signature; // TODO Get rid of this
        typedef boost::shared_ptr<Proposer> ProposerPtr;
        typedef boost::shared_ptr<const Proposer> ProposerConstPtr;

        typedef boost::shared_ptr<std::map<std::string, Entry> > DatabasePtr;
        typedef boost::shared_ptr<const std::map<std::string, Entry> > ConstDatabasePtr;

        Detector() {
          database_.reset(new std::map<std::string, Entry>);
        }

        /** density of keypoints, used as a voxel size */
        static const double keypoint_separation;

        /** each feature point can vote for up to this many models */
        static const int max_votes;

        /** run the registration on this many models */
        static const int num_registration;

        enum TimerBin {
          KEYPOINTS_TRAINING,
          OBTAIN_FEATURES_TRAINING,
          BUILD_TREE,
          KEYPOINTS_TESTING,
          COMPUTE_FEATURES_TESTING,
          VOTING_CLASSIFIER,
          IA_RANSAC,
          ICP,
          NUM_BINS
        };


        //Entry database[Config::num_models];

        /**
         * do any offline processing
         * models[i] is a registered point cloud of the ith model
         */
        void train(Scene &scene);

        /**
         * do any online processing
         * scene is a range scan
         * return guessed model number
         * populate classifier[candidate model] with a similarity score (higher is better)
         * populate registration[candidate model] with a distance score (lower is better, but 0 means didn't try)
         */
        std::string query(Scene &scene, float *classifier, double *registration);

        /** start a visualizer; if called, must be called before training/querying */
        void enableVisualization();

        /** print the timing data */
        void printTimer();

        /** the timer */
        Timer<NUM_BINS> timer;

        /** get a dense sampling of points as keypoints and return their indices */
        IndicesPtr computeKeypoints(PointCloud<PointNormal>::Ptr cloud);

        /** run the feature */
        PointCloud<Signature>::Ptr computeFeatures(PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices);

        /** try to load the features from disk, or do it from scratch. for training only */
        PointCloud<Signature>::Ptr obtainFeatures(Scene &scene, IndicesPtr indices, bool is_test_phase);

        void
        setProposer(const ProposerPtr proposer) {
          proposer_ = proposer;
        }

      private:

        auto_ptr<visualization::CloudViewer> vis;

        KdTree<Signature>::Ptr tree;

        ProposerPtr proposer_;

        DatabasePtr database_;
    };
  }
}

#endif
