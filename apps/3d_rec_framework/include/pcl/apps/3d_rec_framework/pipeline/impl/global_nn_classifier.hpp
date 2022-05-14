/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNPipeline<Distance, PointInT, FeatureT>::
    loadFeaturesAndCreateFLANN()
{
  auto models = source_->getModels();
  for (std::size_t i = 0; i < models->size(); i++) {
    std::string path =
        source_->getModelDescriptorDir(models->at(i), training_dir_, descr_name_);

    for (const auto& dir_entry : bf::directory_iterator(path)) {
      std::string file_name = (dir_entry.path().filename()).string();

      std::vector<std::string> strs;
      boost::split(strs, file_name, boost::is_any_of("_"));

      if (strs[0] == "descriptor") {
        std::string full_file_name = dir_entry.path().string();
        std::vector<std::string> strs;
        boost::split(strs, full_file_name, boost::is_any_of("/"));

        typename pcl::PointCloud<FeatureT>::Ptr signature(
            new pcl::PointCloud<FeatureT>);
        pcl::io::loadPCDFile(full_file_name, *signature);

        flann_model descr_model;
        descr_model.first = models->at(i);
        int size_feat = sizeof((*signature)[0].histogram) / sizeof(float);
        descr_model.second.resize(size_feat);
        memcpy(&descr_model.second[0],
               &(*signature)[0].histogram[0],
               size_feat * sizeof(float));

        flann_models_.push_back(descr_model);
      }
    }
  }

  convertToFLANN(flann_models_, flann_data_);
  flann_index_ = new flann::Index<DistT>(flann_data_, flann::LinearIndexParams());
  flann_index_->buildIndex();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNPipeline<Distance, PointInT, FeatureT>::nearestKSearch(
    flann::Index<DistT>* index,
    const flann_model& model,
    int k,
    flann::Matrix<int>& indices,
    flann::Matrix<float>& distances)
{
  flann::Matrix<float> p =
      flann::Matrix<float>(new float[model.second.size()], 1, model.second.size());
  memcpy(&p.ptr()[0], &model.second[0], p.cols * p.rows * sizeof(float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index->knnSearch(p, indices, distances, k, flann::SearchParams(512));
  delete[] p.ptr();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNPipeline<Distance, PointInT, FeatureT>::classify()
{

  categories_.clear();
  confidences_.clear();

  first_nn_category_ = std::string("");

  PointInTPtr processed(new pcl::PointCloud<PointInT>);
  PointInTPtr in(new pcl::PointCloud<PointInT>);

  typename pcl::PointCloud<FeatureT>::CloudVectorType signatures;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> centroids;

  if (!indices_.empty()) {
    pcl::copyPointCloud(*input_, indices_, *in);
  }
  else {
    in = input_;
  }

  estimator_->estimate(in, processed, signatures, centroids);
  std::vector<index_score> indices_scores;

  if (!signatures.empty()) {
    for (std::size_t idx = 0; idx < signatures.size(); idx++) {
      float* hist = signatures[idx][0].histogram;
      int size_feat = sizeof(signatures[idx][0].histogram) / sizeof(float);
      std::vector<float> std_hist(hist, hist + size_feat);
      ModelT empty;

      flann_model histogram(empty, std_hist);
      flann::Matrix<int> indices;
      flann::Matrix<float> distances;
      nearestKSearch(flann_index_, histogram, NN_, indices, distances);

      // gather NN-search results
      for (int i = 0; i < NN_; ++i) {
        index_score is;
        is.idx_models_ = indices[0][i];
        is.idx_input_ = static_cast<int>(idx);
        is.score_ = distances[0][i];
        indices_scores.push_back(is);
      }
    }

    std::sort(indices_scores.begin(), indices_scores.end(), sortIndexScoresOp);
    first_nn_category_ = flann_models_[indices_scores[0].idx_models_].first.class_;

    std::map<std::string, int> category_map;
    int num_n = std::min(NN_, static_cast<int>(indices_scores.size()));

    for (int i = 0; i < num_n; ++i) {
      std::string cat = flann_models_[indices_scores[i].idx_models_].first.class_;
      auto it = category_map.find(cat);
      if (it == category_map.end()) {
        category_map[cat] = 1;
      }
      else {
        it->second++;
      }
    }

    for (const auto& category : category_map) {
      float prob = static_cast<float>(category.second) / static_cast<float>(num_n);
      categories_.push_back(category.first);
      confidences_.push_back(prob);
    }
  }
  else {
    first_nn_category_ = std::string("error");
    categories_.push_back(first_nn_category_);
  }
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNPipeline<Distance, PointInT, FeatureT>::initialize(
    bool force_retrain)
{

  // use the source to know what has to be trained and what not, checking if the
  // descr_name directory exists unless force_retrain is true, then train everything
  auto models = source_->getModels();
  std::cout << "Models size:" << models->size() << std::endl;

  if (force_retrain) {
    for (std::size_t i = 0; i < models->size(); i++) {
      source_->removeDescDirectory(models->at(i), training_dir_, descr_name_);
    }
  }

  for (std::size_t i = 0; i < models->size(); i++) {
    if (!source_->modelAlreadyTrained(models->at(i), training_dir_, descr_name_)) {
      for (std::size_t v = 0; v < models->at(i).views_->size(); v++) {
        PointInTPtr processed(new pcl::PointCloud<PointInT>);
        // pro view, compute signatures
        typename pcl::PointCloud<FeatureT>::CloudVectorType signatures;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
            centroids;
        estimator_->estimate(
            models->at(i).views_->at(v), processed, signatures, centroids);

        std::string path =
            source_->getModelDescriptorDir(models->at(i), training_dir_, descr_name_);

        bf::path desc_dir = path;
        if (!bf::exists(desc_dir))
          bf::create_directory(desc_dir);

        const std::string path_view = path + "/view_" + std::to_string(v) + ".pcd";
        pcl::io::savePCDFileBinary(path_view, *processed);

        const std::string path_pose = path + "/pose_" + std::to_string(v) + ".txt";
        PersistenceUtils::writeMatrixToFile(path_pose, models->at(i).poses_->at(v));

        const std::string path_entropy =
            path + "/entropy_" + std::to_string(v) + ".txt";
        PersistenceUtils::writeFloatToFile(path_entropy,
                                           models->at(i).self_occlusions_->at(v));

        // save signatures and centroids to disk
        for (std::size_t j = 0; j < signatures.size(); j++) {
          const std::string path_centroid = path + "/centroid_" + std::to_string(v) +
                                            "_" + std::to_string(j) + ".txt";
          Eigen::Vector3f centroid(centroids[j][0], centroids[j][1], centroids[j][2]);
          PersistenceUtils::writeCentroidToFile(path_centroid, centroid);

          const std::string path_descriptor = path + "/descriptor_" +
                                              std::to_string(v) + "_" +
                                              std::to_string(j) + ".pcd";
          pcl::io::savePCDFileBinary(path_descriptor, signatures[j]);
        }
      }
    }
    else {
      // else skip model
      std::cout << "The model has already been trained..." << std::endl;
    }
  }

  // load features from disk
  // initialize FLANN structure
  loadFeaturesAndCreateFLANN();
}
