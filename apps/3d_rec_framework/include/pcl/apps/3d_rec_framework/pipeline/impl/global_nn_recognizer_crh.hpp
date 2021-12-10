/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/global_nn_recognizer_crh.h>
#include <pcl/common/time.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl/registration/icp.h>

#include <random>

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getPose(
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
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getCRH(
    ModelT& model, int view_id, int d_id, CRHPointCloud::Ptr& hist)
{

  hist.reset(new CRHPointCloud);
  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir = path + "/centroid_" + std::to_string(view_id) + '_' +
                          std::to_string(d_id) + ".pcd";

  pcl::io::loadPCDFile(dir, *hist);
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getCentroid(
    ModelT& model, int view_id, int d_id, Eigen::Vector3f& centroid)
{
  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir = path + "/centroid_" + std::to_string(view_id) + '_' +
                          std::to_string(d_id) + ".txt";

  PersistenceUtils::getCentroidFromFile(dir, centroid);
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getView(
    ModelT& model, int view_id, PointInTPtr& view)
{
  view.reset(new pcl::PointCloud<PointInT>);
  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir = path + "/view_" + std::to_string(view_id) + ".pcd";
  pcl::io::loadPCDFile(dir, *view);
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::
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

        int view_id = atoi(strs[1].c_str());
        std::vector<std::string> strs1;
        boost::split(strs1, strs[2], boost::is_any_of("."));
        int descriptor_id = atoi(strs1[0].c_str());

        std::string full_file_name = dir_entry.path().string();
        typename pcl::PointCloud<FeatureT>::Ptr signature(
            new pcl::PointCloud<FeatureT>);
        pcl::io::loadPCDFile(full_file_name, *signature);

        flann_model descr_model;
        descr_model.model = models->at(i);
        descr_model.view_id = view_id;
        descr_model.descriptor_id = descriptor_id;

        int size_feat = sizeof((*signature)[0].histogram) / sizeof(float);
        descr_model.descr.resize(size_feat);
        memcpy(&descr_model.descr[0],
               &(*signature)[0].histogram[0],
               size_feat * sizeof(float));

        flann_models_.push_back(descr_model);

        if (use_cache_) {

          const std::string dir_pose =
              path + "/pose_" + std::to_string(descr_model.view_id) + ".txt";

          Eigen::Matrix4f pose_matrix;
          PersistenceUtils::readMatrixFromFile(dir_pose, pose_matrix);

          std::pair<std::string, int> pair_model_view =
              std::make_pair(models->at(i).id_, descr_model.view_id);
          poses_cache_[pair_model_view] = pose_matrix;
        }
      }
    }
  }

  convertToFLANN(flann_models_, flann_data_);
  flann_index_ = new flann::Index<DistT>(flann_data_, flann::LinearIndexParams());
  flann_index_->buildIndex();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::
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
  index->knnSearch(p, indices, distances, k, flann::SearchParams(512));
  delete[] p.ptr();
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::recognize()
{

  models_.reset(new std::vector<ModelT>);
  transforms_.reset(
      new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);

  PointInTPtr processed(new pcl::PointCloud<PointInT>);
  PointInTPtr in(new pcl::PointCloud<PointInT>);

  std::vector<pcl::PointCloud<FeatureT>,
              Eigen::aligned_allocator<pcl::PointCloud<FeatureT>>>
      signatures;
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> centroids;

  if (!indices_.empty())
    pcl::copyPointCloud(*input_, indices_, *in);
  else
    in = input_;

  {
    pcl::ScopeTime t("Estimate feature");
    crh_estimator_->estimate(in, processed, signatures, centroids);
  }

  std::vector<CRHPointCloud::Ptr> crh_histograms;
  crh_estimator_->getCRHHistograms(crh_histograms);

  std::vector<index_score> indices_scores;
  if (!signatures.empty()) {

    {
      pcl::ScopeTime t_matching("Matching and roll...");
      for (std::size_t idx = 0; idx < signatures.size(); idx++) {

        float* hist = signatures[idx][0].histogram;
        int size_feat = sizeof(signatures[idx][0].histogram) / sizeof(float);
        std::vector<float> std_hist(hist, hist + size_feat);
        ModelT empty;

        flann_model histogram;
        histogram.descr = std_hist;

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

      int num_n = std::min(NN_, static_cast<int>(indices_scores.size()));

      if (do_CRH_) {
        /*
         * Once we have the models, we need to find a 6DOF pose using the roll histogram
         * pass to pcl_recognition::CRHAlignment both views, centroids and CRH
         */

        pcl::CRHAlignment<PointInT, 90> crha;

        for (int i = 0; i < num_n; ++i) {
          ModelT m = flann_models_[indices_scores[i].idx_models_].model;
          int view_id = flann_models_[indices_scores[i].idx_models_].view_id;
          int desc_id = flann_models_[indices_scores[i].idx_models_].descriptor_id;

          std::cout << m.id_ << " " << view_id << " " << desc_id << std::endl;

          // get crhs
          CRHPointCloud::Ptr input_crh = crh_histograms[indices_scores[i].idx_input_];
          CRHPointCloud::Ptr view_crh;
          getCRH(m, view_id, desc_id, view_crh);

          // get centroids
          Eigen::Vector3f input_centroid = centroids[indices_scores[i].idx_input_];
          Eigen::Vector3f view_centroid;
          getCentroid(m, view_id, desc_id, view_centroid);

          // crha.setModelAndInputView (view, processed);
          crha.setInputAndTargetCentroids(view_centroid, input_centroid);
          crha.align(*view_crh, *input_crh);

          Eigen::Matrix4f model_view_pose;
          getPose(m, view_id, model_view_pose);

          std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
              roll_transforms;
          crha.getTransforms(roll_transforms);

          // create object hypothesis
          for (const auto& roll_transform : roll_transforms) {
            Eigen::Matrix4f final_roll_trans(roll_transform * model_view_pose);
            models_->push_back(m);
            transforms_->push_back(final_roll_trans);
          }
        }
      }
      else {
        for (int i = 0; i < num_n; ++i) {
          ModelT m = flann_models_[indices_scores[i].idx_models_].model;
          models_->push_back(m);
        }
      }
    }

    std::cout << "Number of object hypotheses:" << models_->size() << std::endl;

    /**
     * POSE REFINEMENT
     **/

    if (ICP_iterations_ > 0) {
      pcl::ScopeTime t("Pose refinement");

      // Prepare scene and model clouds for the pose refinement step
      float VOXEL_SIZE_ICP_ = 0.005f;
      PointInTPtr cloud_voxelized_icp(new pcl::PointCloud<PointInT>());
      pcl::VoxelGrid<PointInT> voxel_grid_icp;
      voxel_grid_icp.setInputCloud(processed);
      voxel_grid_icp.setLeafSize(VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
      voxel_grid_icp.filter(*cloud_voxelized_icp);
      source_->voxelizeAllModels(VOXEL_SIZE_ICP_);

      // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(VOXEL_SIZE_ICP_, cloud_voxelized_icp) \
  num_threads(omp_get_num_procs())
      // clang-format on
      for (int i = 0; i < static_cast<int>(models_->size()); i++) {

        ConstPointInTPtr model_cloud = models_->at(i).getAssembled(VOXEL_SIZE_ICP_);
        PointInTPtr model_aligned(new pcl::PointCloud<PointInT>);
        pcl::transformPointCloud(*model_cloud, *model_aligned, transforms_->at(i));

        pcl::IterativeClosestPoint<PointInT, PointInT> reg;
        reg.setInputSource(model_aligned);       // model
        reg.setInputTarget(cloud_voxelized_icp); // scene
        reg.setMaximumIterations(ICP_iterations_);
        reg.setMaxCorrespondenceDistance(VOXEL_SIZE_ICP_ * 3.f);
        reg.setTransformationEpsilon(1e-5);

        typename pcl::PointCloud<PointInT>::Ptr output_(
            new pcl::PointCloud<PointInT>());
        reg.align(*output_);

        Eigen::Matrix4f icp_trans = reg.getFinalTransformation();
        transforms_->at(i) = icp_trans * transforms_->at(i);
      }
    }

    /**
     * HYPOTHESES VERIFICATION
     **/

    if (hv_algorithm_) {

      pcl::ScopeTime t("HYPOTHESES VERIFICATION");

      std::vector<typename pcl::PointCloud<PointInT>::ConstPtr> aligned_models;
      aligned_models.resize(models_->size());

      for (std::size_t i = 0; i < models_->size(); i++) {
        ConstPointInTPtr model_cloud = models_->at(i).getAssembled(0.005f);
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
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::initialize(
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
        PointInTPtr view = models->at(i).views_->at(v);

        if (noisify_) {
          std::random_device rd;
          std::mt19937 rng(rd());
          std::normal_distribution<float> nd(0.0f, noise_);
          // Noisify each point in the dataset
          for (std::size_t cp = 0; cp < view->size(); ++cp)
            (*view)[cp].z += nd(rng);
        }

        // pro view, compute signatures and CRH
        std::vector<pcl::PointCloud<FeatureT>,
                    Eigen::aligned_allocator<pcl::PointCloud<FeatureT>>>
            signatures;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
            centroids;
        crh_estimator_->estimate(view, processed, signatures, centroids);

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

        std::vector<CRHPointCloud::Ptr> crh_histograms;
        crh_estimator_->getCRHHistograms(crh_histograms);

        // save signatures and centroids to disk
        for (std::size_t j = 0; j < signatures.size(); j++) {
          const std::string path_centroid = path + "/centroid_" + std::to_string(v) +
                                            '_' + std::to_string(j) + ".txt";
          Eigen::Vector3f centroid(centroids[j][0], centroids[j][1], centroids[j][2]);
          PersistenceUtils::writeCentroidToFile(path_centroid, centroid);

          const std::string path_descriptor = path + "/descriptor_" +
                                              std::to_string(v) + '_' +
                                              std::to_string(j) + ".pcd";
          pcl::io::savePCDFileBinary(path_descriptor, signatures[j]);

          const std::string path_roll =
              path + "/crh_" + std::to_string(v) + '_' + std::to_string(j) + ".pcd";
          pcl::io::savePCDFileBinary(path_roll, *crh_histograms[j]);
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
