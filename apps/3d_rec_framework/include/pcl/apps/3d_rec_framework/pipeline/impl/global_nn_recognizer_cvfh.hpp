/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/global_nn_recognizer_cvfh.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <flann/flann.hpp>

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::getPose(
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
bool
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::
    getRollPose(ModelT& model, int view_id, int d_id, Eigen::Matrix4f& pose_matrix)
{
  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir = path + "/roll_trans_" + std::to_string(view_id) + '_' +
                          std::to_string(d_id) + ".txt";

  const bf::path file_path = dir;
  if (bf::exists(file_path)) {
    PersistenceUtils::readMatrixFromFile(dir, pose_matrix);
    return true;
  }
  return false;
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::
    getCentroid(ModelT& model, int view_id, int d_id, Eigen::Vector3f& centroid)
{
  const std::string path =
      source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  const std::string dir = path + "/centroid_" + std::to_string(view_id) + '_' +
                          std::to_string(d_id) + ".txt";

  PersistenceUtils::getCentroidFromFile(dir, centroid);
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::getView(
    ModelT& model, int view_id, PointInTPtr& view)
{
  view.reset(new pcl::PointCloud<PointInT>);
  std::string path = source_->getModelDescriptorDir(model, training_dir_, descr_name_);
  std::string dir = path + "/view_" + std::to_string(view_id) + ".pcd";
  pcl::io::loadPCDFile(dir, *view);
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::
    loadFeaturesAndCreateFLANN()
{

  auto models = source_->getModels();

  std::map<std::string, std::shared_ptr<std::vector<int>>> single_categories;
  if (use_single_categories_) {
    for (std::size_t i = 0; i < models->size(); i++) {
      std::map<std::string, std::shared_ptr<std::vector<int>>>::iterator it;
      std::string cat_model = models->at(i).class_;
      it = single_categories.find(cat_model);
      if (it == single_categories.end()) {
        std::shared_ptr<std::vector<int>> v(new std::vector<int>);
        single_categories[cat_model] = v;
      }
    }
  }

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

        if (use_single_categories_) {
          std::map<std::string, std::shared_ptr<std::vector<int>>>::iterator it;
          std::string cat_model = models->at(i).class_;
          it = single_categories.find(cat_model);
          if (it == single_categories.end()) {
            std::cout << cat_model << std::endl;
            std::cout << "Should not happen..." << std::endl;
          }
          else {
            it->second->push_back(static_cast<int>(flann_models_.size()));
          }
        }

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

  if (use_single_categories_) {
    single_categories_data_.resize(single_categories.size());
    single_categories_index_.resize(single_categories.size());
    single_categories_pointers_to_models_.resize(single_categories.size());

    int kk = 0;
    for (const auto& single_category : single_categories) {
      // create index and flann data
      convertToFLANN(
          flann_models_, single_category.second, single_categories_data_[kk]);
      single_categories_index_[kk] = new flann::Index<DistT>(
          single_categories_data_[kk], flann::LinearIndexParams());
      single_categories_pointers_to_models_[kk] = single_category.second;

      category_to_vectors_indices_[single_category.first] = kk;
      kk++;
    }
  }
}

template <template <class> class Distance, typename PointInT, typename FeatureT>
void
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::
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
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::recognize()
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
    micvfh_estimator_->estimate(in, processed, signatures, centroids);
  }

  std::vector<index_score> indices_scores;
  descriptor_distances_.clear();

  if (!signatures.empty()) {

    {
      pcl::ScopeTime t_matching("Matching and roll...");

      if (use_single_categories_ && (!categories_to_be_searched_.empty())) {

        // perform search of the different signatures in the categories_to_be_searched_
        for (std::size_t c = 0; c < categories_to_be_searched_.size(); c++) {
          std::cout << "Using category:" << categories_to_be_searched_[c] << std::endl;
          for (std::size_t idx = 0; idx < signatures.size(); idx++) {
            float* hist = signatures[idx][0].histogram;
            int size_feat = sizeof(signatures[idx][0].histogram) / sizeof(float);
            std::vector<float> std_hist(hist, hist + size_feat);

            flann_model histogram;
            histogram.descr = std_hist;
            flann::Matrix<int> indices;
            flann::Matrix<float> distances;

            std::map<std::string, int>::iterator it;
            it = category_to_vectors_indices_.find(categories_to_be_searched_[c]);
            assert(it != category_to_vectors_indices_.end());

            nearestKSearch(single_categories_index_[it->second],
                           histogram,
                           NN_,
                           indices,
                           distances);
            // gather NN-search results
            for (std::size_t i = 0; i < (std::size_t)NN_; ++i) {
              index_score is;
              is.idx_models_ =
                  single_categories_pointers_to_models_[it->second]->at(indices[0][i]);
              is.idx_input_ = static_cast<int>(idx);
              is.score_ = distances[0][i];
              indices_scores.push_back(is);
            }
          }

          // we cannot add more than nmodels per category, so sort here and remove
          // offending ones...
          std::sort(indices_scores.begin(), indices_scores.end(), sortIndexScoresOp);
          indices_scores.resize((c + 1) * NN_);
        }
      }
      else {
        for (std::size_t idx = 0; idx < signatures.size(); idx++) {

          float* hist = signatures[idx][0].histogram;
          int size_feat = sizeof(signatures[idx][0].histogram) / sizeof(float);
          std::vector<float> std_hist(hist, hist + size_feat);

          flann_model histogram;
          histogram.descr = std_hist;

          flann::Matrix<int> indices;
          flann::Matrix<float> distances;
          nearestKSearch(flann_index_, histogram, NN_, indices, distances);

          // gather NN-search results
          double score = 0;
          for (int i = 0; i < NN_; ++i) {
            score = distances[0][i];
            index_score is;
            is.idx_models_ = indices[0][i];
            is.idx_input_ = static_cast<int>(idx);
            is.score_ = score;
            indices_scores.push_back(is);
          }
        }
      }

      std::sort(indices_scores.begin(), indices_scores.end(), sortIndexScoresOp);

      /*
       * There might be duplicated candidates, in those cases it makes sense to take
       * the closer one in descriptor space
       */

      typename std::map<flann_model, bool> found;
      typename std::map<flann_model, bool>::iterator it_map;
      for (std::size_t i = 0; i < indices_scores.size(); i++) {
        flann_model m = flann_models_[indices_scores[i].idx_models_];
        it_map = found.find(m);
        if (it_map == found.end()) {
          indices_scores[found.size()] = indices_scores[i];
          found[m] = true;
        }
      }
      indices_scores.resize(found.size());

      int num_n = std::min(NN_, static_cast<int>(indices_scores.size()));
      std::cout << "Number of object hypotheses... " << num_n << std::endl;

      std::vector<bool> valid_trans;
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
          transformations;

      micvfh_estimator_->getValidTransformsVec(valid_trans);
      micvfh_estimator_->getTransformsVec(transformations);

      for (int i = 0; i < num_n; ++i) {
        ModelT m = flann_models_[indices_scores[i].idx_models_].model;
        int view_id = flann_models_[indices_scores[i].idx_models_].view_id;
        int desc_id = flann_models_[indices_scores[i].idx_models_].descriptor_id;

        int idx_input = indices_scores[i].idx_input_;

        std::cout << m.class_ << "/" << m.id_ << " ==> " << indices_scores[i].score_
                  << std::endl;

        Eigen::Matrix4f roll_view_pose;
        bool roll_pose_found = getRollPose(m, view_id, desc_id, roll_view_pose);

        if (roll_pose_found && valid_trans[idx_input]) {
          Eigen::Matrix4f transposed = roll_view_pose.transpose();

          PointInTPtr view;
          getView(m, view_id, view);

          Eigen::Matrix4f model_view_pose;
          getPose(m, view_id, model_view_pose);

          Eigen::Matrix4f scale_mat;
          scale_mat.setIdentity(4, 4);

          if (compute_scale_) {
            // compute scale using the whole view
            PointInTPtr view_transformed(new pcl::PointCloud<PointInT>);
            Eigen::Matrix4f hom_from_OVC_to_CC;
            hom_from_OVC_to_CC = transformations[idx_input].inverse() * transposed;
            pcl::transformPointCloud(*view, *view_transformed, hom_from_OVC_to_CC);

            Eigen::Vector3f input_centroid = centroids[indices_scores[i].idx_input_];
            Eigen::Vector3f view_centroid;
            getCentroid(m, view_id, desc_id, view_centroid);

            Eigen::Vector4f cmatch4f(
                view_centroid[0], view_centroid[1], view_centroid[2], 0);
            Eigen::Vector4f cinput4f(
                input_centroid[0], input_centroid[1], input_centroid[2], 0);

            Eigen::Vector4f max_pt_input;
            pcl::getMaxDistance(*processed, cinput4f, max_pt_input);
            max_pt_input[3] = 0;
            float max_dist_input = (cinput4f - max_pt_input).norm();

            // compute max dist for transformed model_view
            pcl::getMaxDistance(*view, cmatch4f, max_pt_input);
            max_pt_input[3] = 0;
            float max_dist_view = (cmatch4f - max_pt_input).norm();

            cmatch4f = hom_from_OVC_to_CC * cmatch4f;
            std::cout << max_dist_view << " " << max_dist_input << std::endl;

            float scale_factor_view = max_dist_input / max_dist_view;
            std::cout << "Scale factor:" << scale_factor_view << std::endl;

            Eigen::Matrix4f center, center_inv;

            center.setIdentity(4, 4);
            center(0, 3) = -cinput4f[0];
            center(1, 3) = -cinput4f[1];
            center(2, 3) = -cinput4f[2];

            center_inv.setIdentity(4, 4);
            center_inv(0, 3) = cinput4f[0];
            center_inv(1, 3) = cinput4f[1];
            center_inv(2, 3) = cinput4f[2];

            scale_mat(0, 0) = scale_factor_view;
            scale_mat(1, 1) = scale_factor_view;
            scale_mat(2, 2) = scale_factor_view;

            scale_mat = center_inv * scale_mat * center;
          }

          Eigen::Matrix4f hom_from_OC_to_CC;
          hom_from_OC_to_CC = scale_mat * transformations[idx_input].inverse() *
                              transposed * model_view_pose;

          models_->push_back(m);
          transforms_->push_back(hom_from_OC_to_CC);
          descriptor_distances_.push_back(static_cast<float>(indices_scores[i].score_));
        }
        else {
          PCL_WARN("The roll pose was not found, should use CRH here... \n");
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

      {
        pcl::ScopeTime t("Voxelize stuff...");
        pcl::VoxelGrid<PointInT> voxel_grid_icp;
        voxel_grid_icp.setInputCloud(processed);
        voxel_grid_icp.setLeafSize(VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
        voxel_grid_icp.filter(*cloud_voxelized_icp);
        source_->voxelizeAllModels(VOXEL_SIZE_ICP_);
      }

      // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(cloud_voxelized_icp, VOXEL_SIZE_ICP_) \
  num_threads(omp_get_num_procs())
      // clang-format on
      for (int i = 0; i < static_cast<int>(models_->size()); i++) {

        ConstPointInTPtr model_cloud;
        PointInTPtr model_aligned(new pcl::PointCloud<PointInT>);

        if (compute_scale_) {
          model_cloud = models_->at(i).getAssembled(-1);
          PointInTPtr model_aligned_m(new pcl::PointCloud<PointInT>);
          pcl::transformPointCloud(*model_cloud, *model_aligned_m, transforms_->at(i));
          pcl::VoxelGrid<PointInT> voxel_grid_icp;
          voxel_grid_icp.setInputCloud(model_aligned_m);
          voxel_grid_icp.setLeafSize(VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
          voxel_grid_icp.filter(*model_aligned);
        }
        else {
          model_cloud = models_->at(i).getAssembled(VOXEL_SIZE_ICP_);
          pcl::transformPointCloud(*model_cloud, *model_aligned, transforms_->at(i));
        }

        pcl::IterativeClosestPoint<PointInT, PointInT> reg;
        reg.setInputSource(model_aligned);       // model
        reg.setInputTarget(cloud_voxelized_icp); // scene
        reg.setMaximumIterations(ICP_iterations_);
        reg.setMaxCorrespondenceDistance(VOXEL_SIZE_ICP_ * 3.f);
        reg.setTransformationEpsilon(1e-6);

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
        ConstPointInTPtr model_cloud;
        PointInTPtr model_aligned(new pcl::PointCloud<PointInT>);

        if (compute_scale_) {
          model_cloud = models_->at(i).getAssembled(-1);
          PointInTPtr model_aligned_m(new pcl::PointCloud<PointInT>);
          pcl::transformPointCloud(*model_cloud, *model_aligned_m, transforms_->at(i));
          pcl::VoxelGrid<PointInT> voxel_grid_icp;
          voxel_grid_icp.setInputCloud(model_aligned_m);
          voxel_grid_icp.setLeafSize(0.005f, 0.005f, 0.005f);
          voxel_grid_icp.filter(*model_aligned);
        }
        else {
          model_cloud = models_->at(i).getAssembled(0.005f);
          pcl::transformPointCloud(*model_cloud, *model_aligned, transforms_->at(i));
        }

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
pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::initialize(
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

        if (view->points.empty())
          PCL_WARN("View has no points!!!\n");

        if (noisify_) {
          std::random_device rd;
          std::mt19937 rng(rd());
          std::normal_distribution<float> nd(0.0f, noise_);
          // Noisify each point in the dataset
          for (std::size_t cp = 0; cp < view->size(); ++cp)
            (*view)[cp].z += nd(rng);
        }

        // pro view, compute signatures
        std::vector<pcl::PointCloud<FeatureT>,
                    Eigen::aligned_allocator<pcl::PointCloud<FeatureT>>>
            signatures;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
            centroids;
        micvfh_estimator_->estimate(view, processed, signatures, centroids);

        std::vector<bool> valid_trans;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
            transforms;

        micvfh_estimator_->getValidTransformsVec(valid_trans);
        micvfh_estimator_->getTransformsVec(transforms);

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
          if (valid_trans[j]) {
            const std::string path_centroid = path + "/centroid_" + std::to_string(v) +
                                              '_' + std::to_string(j) + ".txt";
            Eigen::Vector3f centroid(centroids[j][0], centroids[j][1], centroids[j][2]);
            PersistenceUtils::writeCentroidToFile(path_centroid, centroid);

            const std::string path_descriptor = path + "/descriptor_" +
                                                std::to_string(v) + '_' +
                                                std::to_string(j) + ".pcd";
            pcl::io::savePCDFileBinary(path_descriptor, signatures[j]);

            // save roll transform
            const std::string path_pose = path + "/roll_trans_" + std::to_string(v) +
                                          '_' + std::to_string(j) + ".txt";
            PersistenceUtils::writeMatrixToFile(path_pose, transforms[j]);
          }
        }
      }
    }
    else {
      // else skip model
      std::cout << "The model has already been trained..." << std::endl;
      // there is no need to keep the views in memory once the model has been trained
      models->at(i).views_->clear();
    }
  }

  // load features from disk
  // initialize FLANN structure
  loadFeaturesAndCreateFLANN();
}
