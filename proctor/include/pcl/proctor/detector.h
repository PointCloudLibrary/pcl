#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>

#include "proctor/config.h"
#include "proctor/timer.h"

using namespace std;
namespace pcl {
  namespace proctor {
    class Detector {
      public:

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

        typedef FPFHSignature33 Signature;

        /** a cloud and its features */
        typedef struct {
          PointCloud<PointNormal>::Ptr cloud;
          IndicesPtr indices;
          PointCloud<Signature>::Ptr features;
        } Entry;

        Entry database[Config::num_models];

        /**
         * do any offline processing
         * models[i] is a registered point cloud of the ith model
         */
        void train(PointCloud<PointNormal>::Ptr *models);

        /**
         * do any online processing
         * scene is a range scan
         * return guessed model number
         * populate classifier[candidate model] with a similarity score (higher is better)
         * populate registration[candidate model] with a distance score (lower is better, but 0 means didn't try)
         */
        int query(PointCloud<PointNormal>::Ptr scene, float *classifier, double *registration);

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
        PointCloud<Signature>::Ptr obtainFeatures(int mi, PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices);

        /** run IA_RANSAC and ICP to judge similarity */
        double computeRegistration(Entry &source, int mi, int ci);

      private:

        auto_ptr<visualization::CloudViewer> vis;

        KdTree<Signature>::Ptr tree;

    };
  }
}

#endif //#ifndef DETECTOR_H_
