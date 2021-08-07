#include <pcl/apps/3d_rec_framework/pipeline/local_recognizer.h>
#include <pcl/apps/3d_rec_framework/utils/vtk_model_sampling.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <flann/flann.hpp>

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::
    loadFeaturesAndCreateFLANN()
{
  auto models = source_->getModels();
  std::cout << "Models size:" << models->size() << std::endl;

  for (std::size_t i = 0; i < models->size(); i++) {
    std::string path =
        source_->getModelDescriptorDir(models->at(i), training_dir_, descr_name_);

    for (const auto& dir_entry : bf::directory_iterator(path)) {
      std::string file_name = (dir_entry.path().filename()).string();

      std::vector<std::string> strs;
      boost::split(strs, file_name, boost::is_any_of("_"));

      if (strs[0] == "descriptor") {
        std::string full_file_name = dir_entry.path().string();
        std::string name = file_name.substr(0, file_name.length() - 4);
        std::vector<std::string> strs;
        boost::split(strs, name, boost::is_any_of("_"));

        flann_model descr_model;
        descr_model.model = models->at(i);
        descr_model.view_id = atoi(strs[1].c_str());

        if (use_cache_) {
          std::string path =
              source_->getModelDescriptorDir(models->at(i), training_dir_, descr_name_);
          const std::string dir_keypoints = path + "/keypoint_indices_" +
                                            std::to_string(descr_model.view_id) +
                                            ".pcd";

          const std::string dir_pose =
              path + "/pose_" + std::to_string(descr_model.view_id) + ".txt";

          Eigen::Matrix4f pose_matrix;
          PersistenceUtils::readMatrixFromFile(dir_pose, pose_matrix);

          std::pair<std::string, int> pair_model_view =
              std::make_pair(models->at(i).id_, descr_model.view_id);
          poses_cache_[pair_model_view] = pose_matrix;

          // load keypoints and save them to cache
          typename pcl::PointCloud<PointInT>::Ptr keypoints(
              new pcl::PointCloud<PointInT>());
          pcl::io::loadPCDFile(dir_keypoints, *keypoints);
          keypoints_cache_[pair_model_view] = keypoints;
        }

        typename pcl::PointCloud<FeatureT>::Ptr signature(
            new pcl::PointCloud<FeatureT>());
        pcl::io::loadPCDFile(full_file_name, *signature);

        int size_feat = sizeof((*signature)[0].histogram) / sizeof(float);

        for (std::size_t dd = 0; dd < signature->size(); dd++) {
          descr_model.keypoint_id = static_cast<int>(dd);
          descr_model.descr.resize(size_feat);

          memcpy(&descr_model.descr[0],
                 &(*signature)[dd].histogram[0],
                 size_feat * sizeof(float));

          flann_models_.push_back(descr_model);
        }
      }
    }
  }

  convertToFLANN(flann_models_, flann_data_);

  flann_index_ = new flann::Index<DistT>(flann_data_, flann::KDTreeIndexParams(4));
  flann_index_->buildIndex();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::
    nearestKSearch(flann::Index<DistT>* index,
                   const flann_model& model,
                   int k,
                   flann::Matrix<int>& indices,
                   flann::Matrix<float>& distances)
{
  flann::Matrix<float> p =
      flann::Matrix<float>(new float[model.descr.size()], 1, model.descr.size());
  memcpy(&p.ptr()[0], &model.descr[0], p.cols * p.rows * sizeof(float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index->knnSearch(p, indices, distances, k, flann::SearchParams(kdtree_splits_));
  delete[] p.ptr();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::
    initialize(bool force_retrain)
{
  std::shared_ptr<std::vector<ModelT>> models;

  if (search_model_.empty()) {
    models = source_->getModels();
  }
  else {
    models = source_->getModels(search_model_);
    // reset cache and flann structures
    delete flann_index_;

    flann_models_.clear();
    poses_cache_.clear();
    keypoints_cache_.clear();
  }

  std::cout << "Models size:" << models->size() << std::endl;

  if (force_retrain) {
    for (std::size_t i = 0; i < models->size(); i++) {
      source_->removeDescDirectory(models->at(i), training_dir_, descr_name_);
    }
  }

  for (std::size_t i = 0; i < models->size(); i++) {
    std::cout << models->at(i).class_ << " " << models->at(i).id_ << std::endl;

    if (!source_->modelAlreadyTrained(models->at(i), training_dir_, descr_name_)) {
      for (std::size_t v = 0; v < models->at(i).views_->size(); v++) {
        PointInTPtr processed(new pcl::PointCloud<PointInT>);
        typename pcl::PointCloud<FeatureT>::Ptr signatures(
            new pcl::PointCloud<FeatureT>());
        PointInTPtr keypoints_pointcloud;

        bool success = estimator_->estimate(
            models->at(i).views_->at(v), processed, keypoints_pointcloud, signatures);

        if (success) {
          std::string path =
              source_->getModelDescriptorDir(models->at(i), training_dir_, descr_name_);

          bf::path desc_dir = path;
          if (!bf::exists(desc_dir))
            bf::create_directory(desc_dir);

          const std::string path_view = path = "/view_" + std::to_string(v) + ".pcd";
          pcl::io::savePCDFileBinary(path_view, *processed);

          const std::string path_pose = path + "/pose_" + std::to_string(v) + ".txt";
          PersistenceUtils::writeMatrixToFile(path_pose, models->at(i).poses_->at(v));

          if (v < models->at(i).self_occlusions_->size()) {
            const std::string path_entropy =
                path + "/entropy_" + std::to_string(v) + ".txt";
            PersistenceUtils::writeFloatToFile(path_entropy,
                                               models->at(i).self_occlusions_->at(v));
          }

          // save keypoints and signatures to disk
          const std::string keypoints_sstr =
              path + "/keypoint_indices_" + std::to_string(v) + ".pcd";
          pcl::io::savePCDFileBinary(keypoints_sstr, *keypoints_pointcloud);

          const std::string path_descriptor =
              path + "/descriptor_" + std::to_string(v) + ".pcd";
          pcl::io::savePCDFileBinary(path_descriptor, *signatures);
        }
      }
    }
    else {
      std::cout << "Model already trained..." << std::endl;
      // there is no need to keep the views in memory once the model has been trained
      models->at(i).views_->clear();
    }
  }

  loadFeaturesAndCreateFLANN();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::
    recognize()
{

  models_.reset(new std::vector<ModelT>);
  transforms_.reset(
      new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);

  PointInTPtr processed;
  typename pcl::PointCloud<FeatureT>::Ptr signatures(new pcl::PointCloud<FeatureT>());
  PointInTPtr keypoints_pointcloud;

  if (signatures_ != nullptr && processed_ != nullptr &&
      (signatures_->size() == keypoints_pointcloud->size())) {
    keypoints_pointcloud = keypoints_input_;
    signatures = signatures_;
    processed = processed_;
    std::cout << "Using the ISPK ..." << std::endl;
  }
  else {
    processed.reset((new pcl::PointCloud<PointInT>));
    if (!indices_.empty()) {
      PointInTPtr sub_input(new pcl::PointCloud<PointInT>);
      pcl::copyPointCloud(*input_, indices_, *sub_input);
      estimator_->estimate(sub_input, processed, keypoints_pointcloud, signatures);
    }
    else {
      estimator_->estimate(input_, processed, keypoints_pointcloud, signatures);
    }

    processed_ = processed;
  }

  std::cout << "Number of keypoints:" << keypoints_pointcloud->size() << std::endl;

  int size_feat = sizeof((*signatures)[0].histogram) / sizeof(float);

  // feature matching and object hypotheses
  std::map<std::string, ObjectHypothesis> object_hypotheses;
  {
    for (std::size_t idx = 0; idx < signatures->size(); idx++) {
      float* hist = (*signatures)[idx].histogram;
      std::vector<float> std_hist(hist, hist + size_feat);
      flann_model histogram;
      histogram.descr = std_hist;
      flann::Matrix<int> indices;
      flann::Matrix<float> distances;
      nearestKSearch(flann_index_, histogram, 1, indices, distances);

      // read view pose and keypoint coordinates, transform keypoint coordinates to
      // model coordinates
      Eigen::Matrix4f homMatrixPose;
      getPose(flann_models_.at(indices[0][0]).model,
              flann_models_.at(indices[0][0]).view_id,
              homMatrixPose);

      typename pcl::PointCloud<PointInT>::Ptr keypoints(
          new pcl::PointCloud<PointInT>());
      getKeypoints(flann_models_.at(indices[0][0]).model,
                   flann_models_.at(indices[0][0]).view_id,
                   keypoints);

      PointInT view_keypoint =
          (*keypoints)[flann_models_.at(indices[0][0]).keypoint_id];
      PointInT model_keypoint;
      model_keypoint.getVector4fMap() =
          homMatrixPose.inverse() * view_keypoint.getVector4fMap();

      typename std::map<std::string, ObjectHypothesis>::iterator it_map;
      if ((it_map = object_hypotheses.find(
               flann_models_.at(indices[0][0]).model.id_)) != object_hypotheses.end()) {
        // if the object hypothesis already exists, then add information
        ObjectHypothesis oh = (*it_map).second;
        oh.correspondences_pointcloud->points.push_back(model_keypoint);
        oh.correspondences_to_inputcloud->push_back(pcl::Correspondence(
            static_cast<int>(oh.correspondences_pointcloud->size() - 1),
            static_cast<int>(idx),
            distances[0][0]));
        oh.feature_distances_->push_back(distances[0][0]);
      }
      else {
        // create object hypothesis
        ObjectHypothesis oh;

        typename pcl::PointCloud<PointInT>::Ptr correspondences_pointcloud(
            new pcl::PointCloud<PointInT>());
        correspondences_pointcloud->points.push_back(model_keypoint);

        oh.model_ = flann_models_.at(indices[0][0]).model;
        oh.correspondences_pointcloud = correspondences_pointcloud;
        // last keypoint for this model is a correspondence the current scene keypoint

        pcl::CorrespondencesPtr corr(new pcl::Correspondences());
        oh.correspondences_to_inputcloud = corr;
        oh.correspondences_to_inputcloud->push_back(
            pcl::Correspondence(0, static_cast<int>(idx), distances[0][0]));

        std::shared_ptr<std::vector<float>> feat_dist(new std::vector<float>);
        feat_dist->push_back(distances[0][0]);

        oh.feature_distances_ = feat_dist;
        object_hypotheses[oh.model_.id_] = oh;
      }
    }
  }

  {
    for (auto it_map = object_hypotheses.cbegin(); it_map != object_hypotheses.cend();
         it_map++) {
      std::vector<pcl::Correspondences> corresp_clusters;
      cg_algorithm_->setSceneCloud(keypoints_pointcloud);
      cg_algorithm_->setInputCloud((*it_map).second.correspondences_pointcloud);
      cg_algorithm_->setModelSceneCorrespondences(
          (*it_map).second.correspondences_to_inputcloud);
      cg_algorithm_->cluster(corresp_clusters);

      std::cout << "Instances:" << corresp_clusters.size() << " Total correspondences:"
                << (*it_map).second.correspondences_to_inputcloud->size() << " "
                << it_map->first << std::endl;
      std::vector<bool> good_indices_for_hypothesis(corresp_clusters.size(), true);

      if (threshold_accept_model_hypothesis_ < 1.f) {
        // sort the hypotheses for each model according to their correspondences and
        // take those that are threshold_accept_model_hypothesis_ over the max
        // cardinality
        int max_cardinality = -1;
        for (const auto& corresp_cluster : corresp_clusters) {
          if (max_cardinality < static_cast<int>(corresp_cluster.size())) {
            max_cardinality = static_cast<int>(corresp_cluster.size());
          }
        }

        for (std::size_t i = 0; i < corresp_clusters.size(); i++) {
          if (static_cast<float>((corresp_clusters[i]).size()) <
              (threshold_accept_model_hypothesis_ *
               static_cast<float>(max_cardinality))) {
            good_indices_for_hypothesis[i] = false;
          }
        }
      }

      for (std::size_t i = 0; i < corresp_clusters.size(); i++) {

        if (!good_indices_for_hypothesis[i])
          continue;

        Eigen::Matrix4f best_trans;
        typename pcl::registration::TransformationEstimationSVD<PointInT, PointInT>
            t_est;
        t_est.estimateRigidTransformation(*(*it_map).second.correspondences_pointcloud,
                                          *keypoints_pointcloud,
                                          corresp_clusters[i],
                                          best_trans);

        models_->push_back((*it_map).second.model_);
        transforms_->push_back(best_trans);
      }
    }
  }

  std::cout << "Number of hypotheses:" << models_->size() << std::endl;

  /**
   * POSE REFINEMENT
   **/

  if (ICP_iterations_ > 0) {
    pcl::ScopeTime ticp("ICP ");

    // Prepare scene and model clouds for the pose refinement step
    PointInTPtr cloud_voxelized_icp(new pcl::PointCloud<PointInT>());
    pcl::VoxelGrid<PointInT> voxel_grid_icp;
    voxel_grid_icp.setInputCloud(processed);
    voxel_grid_icp.setLeafSize(VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
    voxel_grid_icp.filter(*cloud_voxelized_icp);
    source_->voxelizeAllModels(VOXEL_SIZE_ICP_);

    // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(cloud_voxelized_icp) \
  schedule(dynamic,1) \
  num_threads(omp_get_num_procs())
    // clang-format on
    for (int i = 0; i < static_cast<int>(models_->size()); i++) {

      ConstPointInTPtr model_cloud;
      PointInTPtr model_aligned(new pcl::PointCloud<PointInT>);
      model_cloud = models_->at(i).getAssembled(VOXEL_SIZE_ICP_);
      pcl::transformPointCloud(*model_cloud, *model_aligned, transforms_->at(i));

      typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT>::Ptr
          rej(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT>());

      rej->setInputTarget(cloud_voxelized_icp);
      rej->setMaximumIterations(1000);
      rej->setInlierThreshold(0.005f);
      rej->setInputSource(model_aligned);

      pcl::IterativeClosestPoint<PointInT, PointInT> reg;
      reg.addCorrespondenceRejector(rej);
      reg.setInputTarget(cloud_voxelized_icp); // scene
      reg.setInputSource(model_aligned);       // model
      reg.setMaximumIterations(ICP_iterations_);
      reg.setMaxCorrespondenceDistance(VOXEL_SIZE_ICP_ * 4.f);

      typename pcl::PointCloud<PointInT>::Ptr output_(new pcl::PointCloud<PointInT>());
      reg.align(*output_);

      Eigen::Matrix4f icp_trans = reg.getFinalTransformation();
      transforms_->at(i) = icp_trans * transforms_->at(i);
    }
  }

  /**
   * HYPOTHESES VERIFICATION
   **/

  if (hv_algorithm_) {

    pcl::ScopeTime thv("HV verification");

    std::vector<typename pcl::PointCloud<PointInT>::ConstPtr> aligned_models;
    aligned_models.resize(models_->size());
    for (std::size_t i = 0; i < models_->size(); i++) {
      ConstPointInTPtr model_cloud = models_->at(i).getAssembled(0.0025f);
      PointInTPtr model_aligned(new pcl::PointCloud<PointInT>);
      pcl::transformPointCloud(*model_cloud, *model_aligned, transforms_->at(i));
      aligned_models[i] = model_aligned;
    }

    std::vector<bool> mask_hv;
    hv_algorithm_->setSceneCloud(input_);
    hv_algorithm_->addModels(aligned_models, true);
    hv_algorithm_->verify();
    hv_algorithm_->getMask(mask_hv);

    std::shared_ptr<std::vector<ModelT>> models_temp;
    std::shared_ptr<
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>
        transforms_temp;

    models_temp.reset(new std::vector<ModelT>);
    transforms_temp.reset(
        new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);

    for (std::size_t i = 0; i < models_->size(); i++) {
      if (!mask_hv[i])
        continue;

      models_temp->push_back(models_->at(i));
      transforms_temp->push_back(transforms_->at(i));
    }

    models_ = models_temp;
    transforms_ = transforms_temp;
  }
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getPose(
    ModelT& model, int view_id, Eigen::Matrix4f& pose_matrix)
{

  if (use_cache_) {
    using mv_pair = std::pair<std::string, int>;
    mv_pair pair_model_view = std::make_pair(model.id_, view_id);

    std::map<mv_pair,
             Eigen::Matrix4f,
             std::less<>,
             Eigen::aligned_allocator<std::pair<const mv_pair, Eigen::Matrix4f>>>::
        iterator it = poses_cache_.find(pair_model_view);

    if (it != poses_cache_.end()) {
      pose_matrix = it->second;
      return;
    }
  }

  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir = path + "/pose_" + std::to_string(view_id) + ".txt";

  PersistenceUtils::readMatrixFromFile(dir, pose_matrix);
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::
    getKeypoints(ModelT& model,
                 int view_id,
                 typename pcl::PointCloud<PointInT>::Ptr& keypoints_cloud)
{

  if (use_cache_) {
    std::pair<std::string, int> pair_model_view = std::make_pair(model.id_, view_id);
    typename std::map<std::pair<std::string, int>, PointInTPtr>::iterator it =
        keypoints_cache_.find(pair_model_view);

    if (it != keypoints_cache_.end()) {
      keypoints_cloud = it->second;
      return;
    }
  }

  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir =
      path + "/keypoint_indices_" + std::to_string(view_id) + ".pcd";

  pcl::io::loadPCDFile(dir, *keypoints_cloud);
}
