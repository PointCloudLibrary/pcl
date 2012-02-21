#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <pcl/features/feature.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include "proctor/timer.h"
#include "proctor/database_entry.h"
#include "proctor/keypoint_wrapper.h"
#include "proctor/feature_wrapper.h"
#include "proctor/proposer.h"
#include <pcl/features/fpfh.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

using std::vector;
using std::stringstream;
using std::auto_ptr;
using std::exception;
using std::numeric_limits;

class DetectorVisualizer; // TODO Move into the pcl::proctor namespace

namespace pcl
{
  namespace proctor
  {

    struct Scene;

    class Proposer;

    class Detector
    {
      public:
        typedef boost::shared_ptr<Proposer> ProposerPtr;
        typedef boost::shared_ptr<const Proposer> ProposerConstPtr;

        typedef boost::shared_ptr<std::map<std::string, Entry> > DatabasePtr;
        typedef boost::shared_ptr<const std::map<std::string, Entry> > ConstDatabasePtr;

        typedef boost::shared_ptr<KeypointWrapper> KeypointWrapperPtr;
        typedef boost::shared_ptr<const KeypointWrapper> ConstKeypointWrapperPtr;

        typedef boost::shared_ptr<FeatureWrapper> FeatureWrapperPtr;
        //typedef boost::shared_ptr<const KeypointWrapper> ConstKeypointWrapperPtr;

        Detector() {
          database_.reset(new std::map<std::string, Entry>);
          detector_vis_ = NULL;
        }

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

        /**
         * do any offline processing
         * models[i] is a registered point cloud of the ith model
         */
        void
        train(Scene &scene);

        /**
         * do any online processing
         * scene is a range scan
         * return guessed model id
         */
        std::string
        query(Scene &scene);

        /** start a visualizer; if called, must be called before training/querying */
        void
        enableVisualization(DetectorVisualizer *vis);

        /** print the timing data */
        void
        printTimer();

        /** get a dense sampling of points as keypoints and return their indices */
        PointCloud<PointNormal>::Ptr
        computeKeypoints(PointCloud<PointNormal>::Ptr cloud);

        /** run the feature */
        PointCloud<Signature>::Ptr
        computeFeatures(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints);

        /** try to load the features from disk, or do it from scratch. for training only */
        PointCloud<Signature>::Ptr
        obtainFeatures(Scene &scene, PointCloud<PointNormal>::Ptr keypoints, bool is_test_phase, bool cache = false);

        void
        setProposers(const std::vector<ProposerPtr>& proposers)
        {
          proposers_ = proposers;
        }

        void
        setKeypointWrapper(const KeypointWrapperPtr wrapper)
        {
          keypoint_wrap_ = wrapper;
        }

        void
        setFeatureEstimator(FeatureWrapperPtr &feature_est)
        {
          feature_est_ = feature_est;
        }


        DetectorVisualizer *detector_vis_;

      //private:

        /** the timer */
        Timer<NUM_BINS> timer;

        std::vector<ProposerPtr> proposers_;

        DatabasePtr database_;

        KeypointWrapperPtr keypoint_wrap_;

        FeatureWrapperPtr feature_est_;

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
          ar & feature_est_;
          ar & keypoint_wrap_;
          ar & proposers_;
        }
    };

    inline std::ostream &operator<<(std::ostream & out, Detector const & v) {
      out << "Detector:\n  - Feature: " << v.feature_est_->name_ << "\n  - Keypoint: " << v.keypoint_wrap_->name_;
      return out;
    }
  }
}

#endif
