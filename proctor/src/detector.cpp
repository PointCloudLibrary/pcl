#include <sstream>

#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/common/time.h>

#include "proctor/detector.h"
#include "proctor/proctor.h"
#include "proctor/ia_ransac_sub.h"


namespace pcl {
  namespace proctor {
    /** create the viewports to be used by top candidates */
    class create_viewports {
    public:
      create_viewports(PointCloud<PointNormal>::ConstPtr sentinel) : sentinel(sentinel) {}
      void operator()(visualization::PCLVisualizer &v) {
        for (int ci = 0; ci < Detector::num_registration; ci++) {
          int vp;
          v.createViewPort(double(ci) / Detector::num_registration, 0, double(ci + 1) / Detector::num_registration, 1, vp);
          {
            stringstream ss;
            ss << "candidate_" << ci;
            v.addPointCloud(sentinel, visualization::PointCloudColorHandlerCustom<PointNormal>(sentinel, 0xc0, 0x00, 0x40), ss.str(), vp);
          }
          {
            stringstream ss;
            ss << "aligned_" << ci;
            v.addPointCloud(sentinel, visualization::PointCloudColorHandlerCustom<PointNormal>(sentinel, 0xc0, 0x00, 0x40), ss.str(), vp);
          }
        }
      }
    private:
      PointCloud<PointNormal>::ConstPtr sentinel;
    };

    /** show a candidate model as the specified candidate */
    class show_candidate {
    public:
      show_candidate(int ci, PointCloud<PointNormal>::ConstPtr candidate) : ci(ci), candidate(candidate) {}
      void operator()(visualization::PCLVisualizer &v) {
        {
          stringstream ss;
          ss << "candidate_" << ci;
          v.updatePointCloud(candidate, visualization::PointCloudColorHandlerCustom<PointNormal>(candidate, 0xc0, 0x00, 0x40), ss.str());
        }
        {
          stringstream ss;
          ss << "aligned_" << ci;
          v.updatePointCloud(candidate, visualization::PointCloudColorHandlerCustom<PointNormal>(candidate, 0xc0, 0x00, 0x40), ss.str());
        }
      }
    private:
      int ci;
      PointCloud<PointNormal>::ConstPtr candidate;
    };

    /** show an aligned point cloud in the specified viewport */
    class show_aligned {
    public:
      show_aligned(int ci, PointCloud<PointNormal>::ConstPtr aligned) : ci(ci), aligned(aligned) {}
      void operator()(visualization::PCLVisualizer &v) {
        stringstream ss;
        ss << "aligned_" << ci;
        v.updatePointCloud(aligned, visualization::PointCloudColorHandlerCustom<PointNormal>(aligned, 0xff, 0xff, 0xff), ss.str());
      }
    private:
      int ci;
      PointCloud<PointNormal>::ConstPtr aligned;
    };

    class visualization_callback {
    public:
      visualization_callback(int ci, visualization::CloudViewer *vis) : ci(ci), vis(vis) {}
      void operator()(const PointCloud<PointNormal> &cloud_src,
                      const vector<int> &indices_src,
                      const PointCloud<PointNormal> &cloud_tgt,
                      const vector<int> &indices_tgt) {
        // make a copy of the cloud. expensive.
        PointCloud<PointNormal>::ConstPtr aligned (new PointCloud<PointNormal>(cloud_src));
        vis->runOnVisualizationThreadOnce(show_aligned(ci, aligned));
      }
    private:
      int ci;
      visualization::CloudViewer *vis;
    };

    /** Detector */
    const double Detector::keypoint_separation = 0.1;
    const int Detector::max_votes = 100;
    const int Detector::num_registration = 4;

    /* Trains a single model */
    void Detector::train(Scene &scene) {
      srand(0);
      IndicesPtr keypoints = computeKeypoints(scene.cloud);
      PointCloud<Detector::Signature>::Ptr features = obtainFeatures(scene, keypoints, false);

      Entry e;
      e.cloud = scene.cloud;
      e.indices = keypoints;
      e.features = features;
      e.tree = KdTree<Signature>::Ptr(new KdTreeFLANN<Signature>());
      e.tree->setInputCloud(e.features);

      database[scene.id] = e;
    }

    typedef struct {
      std::string id;
      float votes;
    } Candidate;

    bool operator<(const Candidate &a, const Candidate &b) {
      return a.votes > b.votes; // sort descending
    }

    double Detector::get_votes(Entry &query, Entry &match) {
      double votes = 0;
      for (unsigned int pi = 0; pi < query.indices->size(); pi++) {
        vector<int> indices;
        vector<float> distances;

        clock_t start = clock();
        StopWatch s;
        int num_found = match.tree->nearestKSearch(*query.features, pi, max_votes, indices, distances);

        for (int ri = 0; ri < num_found; ri++) {
          votes += 1. / (distances[ri] + numeric_limits<float>::epsilon());
          //votes -= distances[ri];
        }
      }

      return votes;
    }

    // TODO scene as a reference
    std::string Detector::query(Scene scene, float *classifier, double *registration) {
      cout << "Running a test on model " << scene.id << endl;

      Entry e;
      e.cloud = scene.cloud;
      timer.start();
      e.indices = computeKeypoints(e.cloud);
      timer.stop(KEYPOINTS_TESTING);
      timer.start();
      e.features = obtainFeatures(scene, e.indices, true);
      timer.stop(COMPUTE_FEATURES_TESTING);
      // let each point cast votes
      timer.start();
      memset(classifier, 0, Config::num_models * sizeof(*classifier));
      clock_t totalTime = 0;
      float time = 0;

      StopWatch s;

      std::map<std::string, Entry>::iterator it;

      // get top candidates
      vector<Candidate> ballot;
      for ( it = database.begin() ; it != database.end(); it++ ) {
        std::string target_id = (*it).first;
        Entry target = (*it).second;

        Candidate* candidate = new Candidate;
        candidate->id = target_id;
        candidate->votes = get_votes(e, target);
        ballot.push_back(*candidate);
      }

      sort(ballot.begin(), ballot.end());

      time += s.getTimeSeconds();
      //cout << "Total K-Nearest Search Time for Query: " << time << endl;

      //timer.stop(VOTING_CLASSIFIER);
      //if (vis.get()) {
        //for (int ci = 0; ci < num_registration; ci++) {
          //vis->runOnVisualizationThreadOnce(show_candidate(ci, database[ballot[ci].mi].cloud));
        //}
      //}
       //run registration on top candidates
      //memset(registration, 0, Config::num_models * sizeof(*registration));
      std::map<std::string, double> reg_scores;
      double best = numeric_limits<double>::infinity();
      std::string guessed_id = "";
      for (int ci = 0; ci < num_registration; ci++) {
        std::string id = ballot[ci].id;
        flush(cout << id << ": " << ballot[ci].votes);
        reg_scores[id] = computeRegistration(e, id, ci);
        cout << " / " << reg_scores[id] << endl;
        if (reg_scores[id] < best) {
          guessed_id = id;
          best = reg_scores[id];
        }
      }
      return guessed_id;
    }

    void Detector::enableVisualization() {
      vis.reset(new visualization::CloudViewer("Detector Visualization"));
      PointCloud<PointNormal>::Ptr sentinel (new PointCloud<PointNormal>());
      sentinel->points.push_back(PointNormal());
      vis->runOnVisualizationThreadOnce(create_viewports(sentinel));
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

      srand(0);
      int r = 200;
      IndicesPtr subset (new vector<int>());
      for (int i = 0; i < r; i++) {
        if (indices->size() == 0) {
          break;
        }
        int pick = rand() % indices->size();
        subset->push_back(indices->at(pick));
        indices->erase(indices->begin() + pick);
      }
      cout << "Subset Size: " << subset->size() << endl;
      return subset;
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
    PointCloud<Detector::Signature>::Ptr Detector::obtainFeatures(Scene &scene, IndicesPtr indices, bool is_test_phase) {
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

    double Detector::computeRegistration(Entry &source, std::string id, int ci) {
      Entry& target = database[id];
      typedef boost::function<void(const PointCloud<PointNormal> &cloud_src,
                                  const vector<int> &indices_src,
                                  const PointCloud<PointNormal> &cloud_tgt,
                                  const vector<int> &indices_tgt)> f;

      //cout << "Source: " << id << "\tSize: " << source.cloud->size() << "\tIndices Size: " << source.indices->size() << "\tFeature Size: " << source.features->size() << endl;
      //cout << "Target: " << id << "\tSize: " << target.cloud->size() << "\tIndices Size: " << target.indices->size() << "\tFeature Size: " << target.features->size() << endl;
      // Copy the source/target clouds with only the subset of points that
      // also features calculated for them.
      PointCloud<PointNormal>::Ptr source_subset(
          new PointCloud<PointNormal>(*(source.cloud), *(source.indices)));
      PointCloud<PointNormal>::Ptr target_subset(
          new PointCloud<PointNormal>(*(target.cloud), *(target.indices)));

      timer.start();


      // TODO Filtering the source cloud makes it much faster when computing the
      // error metric, but may not be as good
      SampleConsensusInitialAlignment<PointNormal, PointNormal, Signature> ia_ransac;
      ia_ransac.setMinSampleDistance(0.05);
      ia_ransac.setMaxCorrespondenceDistance(0.5);
      ia_ransac.setMaximumIterations(256);
      ia_ransac.setInputCloud(source_subset);
      ia_ransac.setSourceFeatures(source.features);
      //ia_ransac.setSourceIndices(source.indices);
      ia_ransac.setInputTarget(target_subset);
      ia_ransac.setTargetFeatures(target.features);
      //ia_ransac.setTargetIndices(target.indices);

      if (vis.get()) {
        f updater (visualization_callback(ci, vis.get()));
        ia_ransac.registerVisualizationCallback(updater);
      }


      PointCloud<PointNormal>::Ptr aligned (new PointCloud<PointNormal>());
      ia_ransac.align(*aligned);
      timer.stop(IA_RANSAC);

      timer.start();
      IterativeClosestPoint<PointNormal, PointNormal> icp;
      PointCloud<PointNormal>::Ptr aligned2 (new PointCloud<PointNormal>());
      icp.setInputCloud(aligned);
      icp.setInputTarget(target.cloud);
      icp.setMaxCorrespondenceDistance(0.1);
      icp.setMaximumIterations(64);
      if (vis.get()) {
        f updater (visualization_callback(ci, vis.get()));
        icp.registerVisualizationCallback(updater);
      }
      icp.align(*aligned2);
      timer.stop(ICP);
      return icp.getFitnessScore();
    }
  }
}
