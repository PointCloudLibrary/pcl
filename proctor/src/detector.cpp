#include <sstream>

#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/time.h>

#include "proctor/detector.h"
#include "proctor/proctor.h"
#include "proctor/proposer.h"
#include "proctor/registration_proposer.h"
#include "proctor/detector_visualizer.h"

namespace pcl {
  namespace proctor {
    /** Detector */
    const double Detector::keypoint_separation = 0.1;
    const int Detector::max_votes = 100;
    const int Detector::num_registration = 4;

    /* Trains a single model */
    void Detector::train(Scene &scene) {
      srand(0);
      if (detector_vis_) {
        detector_vis_->addCloud(scene.id, scene.cloud);
      }

      IndicesPtr keypoints = computeKeypoints(scene.cloud);
      PointCloud<Detector::Signature>::Ptr features = obtainFeatures(scene, keypoints, false);

      Entry e;
      e.cloud = scene.cloud;
      e.indices = keypoints;
      e.features = features;
      e.tree = KdTree<Signature>::Ptr(new KdTreeFLANN<Signature>());
      e.tree->setInputCloud(e.features);

      (*database_)[scene.id] = e;
    }

    std::string Detector::query(Scene &scene, float *classifier, double *registration) {
      cout << "detector testing " << scene.id << endl;

      Entry e;
      e.cloud = scene.cloud;
      timer.start();
      e.indices = computeKeypoints(e.cloud);
      timer.stop(KEYPOINTS_TESTING);
      timer.start();
      e.features = obtainFeatures(scene, e.indices, true);
      timer.stop(COMPUTE_FEATURES_TESTING);

      memset(classifier, 0, Config::num_models * sizeof(*classifier));
      clock_t totalTime = 0;
      float time = 0;

      StopWatch s;

      std::vector<std::string> all_ids;
      for (std::map<std::string, Entry>::iterator db_it = database_->begin(); db_it != database_->end(); db_it++)
      {
        all_ids.push_back((*db_it).first);
      }

      std::vector<std::string> proposed;
      proposer_->setDatabase(database_);
      proposer_->getProposed(num_registration, e, all_ids, proposed);

      RegistrationProposer reg_proposer;
      std::vector<std::string> picked;
      reg_proposer.setDatabase(database_);
      reg_proposer.getProposed(1, e, proposed, picked);

      return picked[0];
    }

    void Detector::enableVisualization(DetectorVisualizer *vis) {
      detector_vis_ = vis;
    }

    void Detector::printTimer() {
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

    IndicesPtr Detector::computeKeypoints(PointCloud<PointNormal>::Ptr cloud) {
      IndicesPtr indices (new vector<int>());
      PointCloud<int> leaves;
      UniformSampling<PointNormal> us;
      us.setRadiusSearch(keypoint_separation);
      us.setInputCloud(cloud);
      us.compute(leaves);
      indices->assign(leaves.points.begin(), leaves.points.end()); // can't use operator=, probably because of different allocators

      return indices;
    }

    PointCloud<Detector::Signature>::Ptr Detector::computeFeatures(PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices) {
      cout << "computing features on " << indices->size() << " points" << endl;
      PointCloud<Signature>::Ptr features (new PointCloud<Signature>());
      FPFHEstimation<PointNormal, PointNormal, Signature> fpfh;
      fpfh.setRadiusSearch(0.1);
      fpfh.setInputCloud(cloud);
      fpfh.setIndices(indices);
      search::KdTree<PointNormal>::Ptr kdt (new search::KdTree<PointNormal>());
      fpfh.setSearchMethod(kdt);
      fpfh.setInputNormals(cloud);
      fpfh.compute(*features);
      if (features->points.size() != indices->size())
        cout << "got " << features->points.size() << " features from " << indices->size() << " points" << endl;
      return features;
    }

    // TODO Enum for is_test_phase
    PointCloud<Detector::Signature>::Ptr Detector::obtainFeatures(Scene &scene, IndicesPtr indices, bool is_test_phase, bool cache) {
      if (cache == false)
      {
        PointCloud<Signature>::Ptr features = computeFeatures(scene.cloud, indices);
        return features;
      }
      else
      {
        std::string name_str = std::string("feature_") + scene.id;

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
          io::loadPCDFile(name, *features);
          //if (features->points.size() != indices->size())
          //cout << "got " << features->points.size() << " features from " << indices->size() << " points" << endl;
          return features;
        } else {
          PointCloud<Signature>::Ptr features = computeFeatures(scene.cloud, indices);
          io::savePCDFileBinary(name, *features);
          return features;
        }
      }
    }
  }
}
