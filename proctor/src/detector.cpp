#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/registration/icp.h>

#include "proctor/detector.h"
#include "proctor/ia_ransac_sub.h"
#include "proctor/proctor.h"

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

void Detector::train(PointCloud<PointNormal>::Ptr *models) {
  srand(time(NULL));
  PointCloud<Signature>::Ptr features (new PointCloud<Signature>);
  for (int mi = 0; mi < Config::num_models; mi++) {
    Entry &e = database[mi];
    e.cloud = models[mi];
    timer.start();
    e.indices = computeKeypoints(e.cloud);
    timer.stop(KEYPOINTS_TRAINING);
    timer.start();
    e.features = obtainFeatures(mi, e.cloud, e.indices);
    timer.stop(OBTAIN_FEATURES_TRAINING);
    *features += *e.features;
    cout << "finished model " << mi << endl;
  }
  timer.start();
  tree = KdTree<Signature>::Ptr(new KdTreeFLANN<Signature>());
  tree->setInputCloud(features);
  timer.stop(BUILD_TREE);
}

typedef struct {
  int mi;
  float votes;
} Candidate;

bool operator<(const Candidate &a, const Candidate &b) {
  return a.votes > b.votes; // sort descending
}

int Detector::query(PointCloud<PointNormal>::Ptr scene, float *classifier, double *registration) {
  Entry e;
  e.cloud = scene;
  timer.start();
  e.indices = computeKeypoints(e.cloud);
  timer.stop(KEYPOINTS_TESTING);
  timer.start();
  e.features = computeFeatures(e.cloud, e.indices);
  timer.stop(COMPUTE_FEATURES_TESTING);
  // let each point cast votes
  timer.start();
  memset(classifier, 0, Config::num_models * sizeof(*classifier));
  for (int pi = 0; pi < e.indices->size(); pi++) {
    vector<int> indices;
    vector<float> distances;
    tree->nearestKSearch(*e.features, pi, max_votes, indices, distances);
    for (int ri = 0; ri < max_votes; ri++) {
      // do a linear search to determine which model
      // this will make sense when we switch to dense
      int index = indices[ri];
      int mi;
      for (mi = 0; mi < Config::num_models; mi++) {
        if (index < database[mi].indices->size()) break;
        index -= database[mi].indices->size();
      }
      // TODO: is this appropriate weighting?
      classifier[mi] += 1. / (distances[ri] + numeric_limits<float>::epsilon());
    }
  }
  // get top candidates
  vector<Candidate> ballot(Config::num_models);
  for (int mi = 0; mi < Config::num_models; mi++) {
    ballot[mi].mi = mi;
    ballot[mi].votes = classifier[mi];
  }
  sort(ballot.begin(), ballot.end());
  timer.stop(VOTING_CLASSIFIER);
  if (vis.get()) {
    for (int ci = 0; ci < num_registration; ci++) {
      vis->runOnVisualizationThreadOnce(show_candidate(ci, database[ballot[ci].mi].cloud));
    }
  }
  // run registration on top candidates
  memset(registration, 0, Config::num_models * sizeof(*registration));
  double best = numeric_limits<double>::infinity();
  int guess = -1;
  for (int ci = 0; ci < num_registration; ci++) {
    int mi = ballot[ci].mi;
    flush(cout << mi << ": " << ballot[ci].votes);
    registration[mi] = computeRegistration(e, mi, ci);
    cout << " / " << registration[mi] << endl;
    if (registration[mi] < best) {
      guess = mi;
      best = registration[mi];
    }
  }
  for (int ci = num_registration; ci < Config::num_models; ci++) {
    cout << ballot[ci].mi << ": " << ballot[ci].votes << endl;
  }
  return guess;
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
}

PointCloud<Detector::Signature>::Ptr Detector::computeFeatures(PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices) {
  cout << "computing features on " << indices->size() << " points" << endl;
  PointCloud<Signature>::Ptr features (new PointCloud<Signature>());
  FPFHEstimation<PointNormal, PointNormal, Signature> fpfh;
  fpfh.setRadiusSearch(0.1);
  fpfh.setInputCloud(cloud);
  fpfh.setIndices(indices);
  KdTree<PointNormal>::Ptr kdt (new KdTreeFLANN<PointNormal>());
  fpfh.setSearchMethod(kdt);
  fpfh.setInputNormals(cloud);
  fpfh.compute(*features);
  if (features->points.size() != indices->size())
    cout << "got " << features->points.size() << " features from " << indices->size() << " points" << endl;
  return features;
}

PointCloud<Detector::Signature>::Ptr Detector::obtainFeatures(int mi, PointCloud<PointNormal>::Ptr cloud, IndicesPtr indices) {
  char name[17];
  sprintf(name, "feature_%04d.pcd", Proctor::models[mi].id);
  if (ifstream(name)) {
    PointCloud<Signature>::Ptr features (new PointCloud<Signature>());
    io::loadPCDFile(name, *features);
    if (features->points.size() != indices->size())
      cout << "got " << features->points.size() << " features from " << indices->size() << " points" << endl;
    return features;
  } else {
    PointCloud<Signature>::Ptr features = computeFeatures(cloud, indices);
    io::savePCDFileBinary(name, *features);
    return features;
  }
}

double Detector::computeRegistration(Entry &source, int mi, int ci) {
  Entry &target = database[mi];
  typedef boost::function<void(const PointCloud<PointNormal> &cloud_src,
                               const vector<int> &indices_src,
                               const PointCloud<PointNormal> &cloud_tgt,
                               const vector<int> &indices_tgt)> f;

  timer.start();
  SubsetSAC_IA<PointNormal, PointNormal, Signature> ia_ransac_sub;
  PointCloud<PointNormal>::Ptr aligned (new PointCloud<PointNormal>());
  ia_ransac_sub.setSourceIndices(source.indices);
  ia_ransac_sub.setTargetIndices(target.indices);
  ia_ransac_sub.setMinSampleDistance(0.05);
  ia_ransac_sub.setMaxCorrespondenceDistance(0.5);
  ia_ransac_sub.setMaximumIterations(256);
  ia_ransac_sub.setInputCloud(source.cloud);
  ia_ransac_sub.setSourceFeatures(source.features);
  ia_ransac_sub.setInputTarget(target.cloud);
  ia_ransac_sub.setTargetFeatures(target.features);
  if (vis.get()) {
    f updater (visualization_callback(ci, vis.get()));
    ia_ransac_sub.registerVisualizationCallback(updater);
  }
  ia_ransac_sub.align(*aligned);
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
