#include <sstream>
#include <stdlib.h>

#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>

#include "proctor/detector.h"
#include "proctor/proctor.h"
#include "proctor/proposer.h"
#include "proctor/registration_proposer.h"
#include "proctor/detector_visualizer.h"

namespace pcl
{
  namespace proctor
  {
    /** Detector */
    const int Detector::num_registration = 3;

    /* Trains a single model */
    void
    Detector::train(Scene &scene)
    {
      srand(0);

      PointCloud<PointNormal>::Ptr keypoints = computeKeypoints(scene.cloud);

      if (detector_vis_) {
        detector_vis_->addCloud(scene.id + "keypoints", keypoints);
        detector_vis_->addCloud(scene.id + "original", scene.cloud);
      }
      PointCloud<Signature>::Ptr features = obtainFeatures(scene, keypoints, false);

      Entry e;
      e.cloud = scene.cloud;
      e.keypoints = keypoints;
      e.features = features;
      e.tree = KdTreeFLANN<Signature>::Ptr(new KdTreeFLANN<Signature>());
      e.tree->setInputCloud(e.features);

      (*database_)[scene.id] = e;
    }

    std::string
    Detector::query(Scene &scene, float *classifier, double *registration)
    {
      static int i = 0;
      if (detector_vis_) {
        std::stringstream ss;
        std::string s;
        ss << i;
        s = ss.str();
        detector_vis_->addCloud(scene.id + "_test_" + s, scene.cloud);
        i++;
      }

      cout << "detector testing " << scene.id << endl;

      Entry e;
      e.cloud = scene.cloud;
      timer.start();
      e.keypoints = computeKeypoints(e.cloud);
      timer.stop(KEYPOINTS_TESTING);
      timer.start();
      e.features = obtainFeatures(scene, e.keypoints, true);
      timer.stop(COMPUTE_FEATURES_TESTING);

      memset(classifier, 0, Config::num_models * sizeof(*classifier));

      StopWatch s;

      std::vector<std::string> all_ids;
      for (std::map<std::string, Entry>::iterator db_it = database_->begin(); db_it != database_->end(); db_it++)
      {
        all_ids.push_back((*db_it).first);
      }

      std::vector<std::string> current_ids = all_ids;

      for (size_t i = 0; i < proposers_.size(); i++)
      {
        Proposer::Ptr current_proposer = proposers_[i];
        current_proposer->setDatabase(database_);
        current_proposer->getProposed(num_registration, e, current_ids, current_ids);

        cout << "Proposed after stage " << i << ":";
        for (size_t i = 0; i < current_ids.size(); i++) {
          cout << current_ids[i] << " ";
        }
        cout << endl;
      }

      return current_ids[0];
    }

    void
    Detector::enableVisualization(DetectorVisualizer *vis)
    {
      detector_vis_ = vis;
    }

    void
    Detector::printTimer()
    {
      printf(
        "select training keypoints: %10.3f sec\n"
        "obtain training features:  %10.3f sec\n"
        "build feature tree:        %10.3f sec\n"
        "select testing keypoints:  %10.3f sec\n"
        "compute testing features:  %10.3f sec\n"
        "voting classifier:         %10.3f sec\n"
        "initial alignment:         %10.3f sec\n"
        "ICP:                       %10.3f sec\n",
        timer[KEYPOINTS_TRAINING],
        timer[OBTAIN_FEATURES_TRAINING],
        timer[BUILD_TREE],
        timer[KEYPOINTS_TESTING],
        timer[COMPUTE_FEATURES_TESTING],
        timer[VOTING_CLASSIFIER],
        timer[IA_RANSAC],
        timer[ICP]
      );
    }

    PointCloud<PointNormal>::Ptr
    Detector::computeKeypoints(PointCloud<PointNormal>::Ptr cloud)
    {
      //PointNormal min;
      //PointNormal max;

      //getMinMax3D(*cloud, min, max);
      //cout << "Max: " << max << "\tMin: " << min << endl;
      //float vol = 1;
      //vol *= max.x - min.x;
      //vol *= max.y - min.y;
      //vol *= max.z - min.z;

      //float keypoint_separation = pow(vol / 10000, 0.3);
      //cout << "Sep: " << vol << " " << keypoint_separation << endl;
      //keypoint_separation = 0.05;
      //
      PointCloud<PointNormal>::Ptr keypoints (new PointCloud<PointNormal>());

      if (keypoint_wrap_)
      {
        keypoint_wrap_->compute(cloud, *keypoints);
      }
      else
      {
        // TODO Figure out how to handle this error
        cerr << "No keypoint estimator given" << endl;
      }

      return keypoints;
    }

    PointCloud<Signature>::Ptr
    Detector::computeFeatures(PointCloud<PointNormal>::Ptr cloud, PointCloud<PointNormal>::Ptr keypoints)
    {
      cout << "computing features on " << keypoints->size() << " points. search surface size: " << cloud->size() << endl;
      PointCloud<Signature>::Ptr features (new PointCloud<Signature>());

      feature_est_->compute(cloud, keypoints, *features);
      cout << "done computing features" << endl;
      return features;
    }

    // TODO Enum for is_test_phase
    PointCloud<Signature>::Ptr
    Detector::obtainFeatures(Scene &scene, PointCloud<PointNormal>::Ptr keypoints, bool is_test_phase, bool cache)
    {
      if (cache == false)
      {
        PointCloud<Signature>::Ptr features = computeFeatures(scene.cloud, keypoints);
        return features;
      }
      else
      {
        std::string name_str = std::string(feature_est_->name_) + scene.id;

        if (is_test_phase) {
          name_str += "_test";
        }
        else {
          name_str += "_train";
        }

        name_str += ".pcd";

        const char *name = name_str.c_str();

        if (ifstream(name)) {
          PointCloud<Signature>::Ptr features (new PointCloud<Signature>());
          //io::loadPCDFile(name, *features);
          //if (features->points.size() != indices->size())
          //cout << "got " << features->points.size() << " features from " << indices->size() << " points" << endl;
          return features;
        } else {
          PointCloud<Signature>::Ptr features = computeFeatures(scene.cloud, keypoints);
          //io::savePCDFileBinary(name, *features);
          return features;
        }
      }
    }
  }
}
