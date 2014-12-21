/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/apps/3d_rec_framework/pipeline/global_nn_recognizer_crh.h>
#include <pcl/recognition/crh_alignment.h>
#include <pcl/registration/icp.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <pcl/common/time.h>

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix)
  {

    if (use_cache_)
    {
      typedef std::pair<std::string, int> mv_pair;
      mv_pair pair_model_view = std::make_pair (model.id_, view_id);

      std::map<mv_pair, Eigen::Matrix4f, std::less<mv_pair>, Eigen::aligned_allocator<std::pair<mv_pair, Eigen::Matrix4f> > >::iterator it =
          poses_cache_.find (pair_model_view);

      if (it != poses_cache_.end ())
      {
        pose_matrix = it->second;
        return;
      }

    }

    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/pose_" << view_id << ".txt";

    PersistenceUtils::readMatrixFromFile (dir.str (), pose_matrix);
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getCRH (ModelT & model, int view_id, int d_id,
                                                                                      CRHPointCloud::Ptr & hist)
  {

    hist.reset (new CRHPointCloud);
    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/crh_" << view_id << "_" << d_id << ".pcd";

    pcl::io::loadPCDFile (dir.str (), *hist);
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getCentroid (ModelT & model, int view_id, int d_id,
                                                                                           Eigen::Vector3f & centroid)
  {
    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/centroid_" << view_id << "_" << d_id << ".txt";

    PersistenceUtils::getCentroidFromFile (dir.str (), centroid);
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::getView (ModelT & model, int view_id, PointInTPtr & view)
  {
    view.reset (new pcl::PointCloud<PointInT>);
    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/view_" << view_id << ".pcd";
    pcl::io::loadPCDFile (dir.str (), *view);

  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::loadFeaturesAndCreateFLANN ()
  {
    boost::shared_ptr < std::vector<ModelT> > models = source_->getModels ();
    for (size_t i = 0; i < models->size (); i++)
    {
      std::string path = source_->getModelDescriptorDir (models->at (i), training_dir_, descr_name_);
      bf::path inside = path;
      bf::directory_iterator end_itr;

      for (bf::directory_iterator itr_in (inside); itr_in != end_itr; ++itr_in)
      {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string file_name = (itr_in->path ().filename ()).string();
#else
        std::string file_name = (itr_in->path ()).filename ();
#endif

        std::vector < std::string > strs;
        boost::split (strs, file_name, boost::is_any_of ("_"));

        if (strs[0] == "descriptor")
        {

          int view_id = atoi (strs[1].c_str ());
          std::vector < std::string > strs1;
          boost::split (strs1, strs[2], boost::is_any_of ("."));
          int descriptor_id = atoi (strs1[0].c_str ());

          std::string full_file_name = itr_in->path ().string ();
          typename pcl::PointCloud<FeatureT>::Ptr signature (new pcl::PointCloud<FeatureT>);
          pcl::io::loadPCDFile (full_file_name, *signature);

          flann_model descr_model;
          descr_model.model = models->at (i);
          descr_model.view_id = view_id;
          descr_model.descriptor_id = descriptor_id;

          int size_feat = sizeof(signature->points[0].histogram) / sizeof(float);
          descr_model.descr.resize (size_feat);
          memcpy (&descr_model.descr[0], &signature->points[0].histogram[0], size_feat * sizeof(float));

          flann_models_.push_back (descr_model);

          if (use_cache_)
          {

            std::stringstream dir_pose;
            dir_pose << path << "/pose_" << descr_model.view_id << ".txt";

            Eigen::Matrix4f pose_matrix;
            PersistenceUtils::readMatrixFromFile (dir_pose.str (), pose_matrix);

            std::pair<std::string, int> pair_model_view = std::make_pair (models->at (i).id_, descr_model.view_id);
            poses_cache_[pair_model_view] = pose_matrix;
          }
        }
      }
    }

    convertToFLANN (flann_models_, flann_data_);
    flann_index_ = new flann::Index<DistT> (flann_data_, flann::LinearIndexParams ());
    flann_index_->buildIndex ();
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::nearestKSearch (flann::Index<DistT> * index, const flann_model &model,
                                                                                              int k, flann::Matrix<int> &indices,
                                                                                              flann::Matrix<float> &distances)
  {
    flann::Matrix<float> p = flann::Matrix<float> (new float[model.descr.size ()], 1, model.descr.size ());
    memcpy (&p.ptr ()[0], &model.descr[0], p.cols * p.rows * sizeof(float));

    indices = flann::Matrix<int> (new int[k], 1, k);
    distances = flann::Matrix<float> (new float[k], 1, k);
    index->knnSearch (p, indices, distances, k, flann::SearchParams (512));
    delete[] p.ptr ();
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::recognize ()
  {

    models_.reset (new std::vector<ModelT>);
    transforms_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

    PointInTPtr processed (new pcl::PointCloud<PointInT>);
    PointInTPtr in (new pcl::PointCloud<PointInT>);

    std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures;
    std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > centroids;

    if (indices_.size ())
      pcl::copyPointCloud (*input_, indices_, *in);
    else
      in = input_;

    {
      pcl::ScopeTime t ("Estimate feature");
      crh_estimator_->estimate (in, processed, signatures, centroids);
    }

    std::vector<CRHPointCloud::Ptr> crh_histograms;
    crh_estimator_->getCRHHistograms (crh_histograms);

    std::vector<index_score> indices_scores;
    if (signatures.size () > 0)
    {

      {
        pcl::ScopeTime t_matching ("Matching and roll...");
        for (size_t idx = 0; idx < signatures.size (); idx++)
        {

          float* hist = signatures[idx].points[0].histogram;
          int size_feat = sizeof(signatures[idx].points[0].histogram) / sizeof(float);
          std::vector<float> std_hist (hist, hist + size_feat);
          ModelT empty;

          flann_model histogram;
          histogram.descr = std_hist;

          flann::Matrix<int> indices;
          flann::Matrix<float> distances;
          nearestKSearch (flann_index_, histogram, NN_, indices, distances);

          //gather NN-search results
          double score = 0;
          for (int i = 0; i < NN_; ++i)
          {
            score = distances[0][i];
            index_score is;
            is.idx_models_ = indices[0][i];
            is.idx_input_ = static_cast<int> (idx);
            is.score_ = score;
            indices_scores.push_back (is);
          }
        }

        std::sort (indices_scores.begin (), indices_scores.end (), sortIndexScoresOp);

        int num_n = std::min (NN_, static_cast<int> (indices_scores.size ()));

        /*
         * Filter some hypothesis regarding to their distance to the first neighbour
         */

        /*std::vector<index_score> indices_scores_filtered;
        indices_scores_filtered.resize (num_n);
        indices_scores_filtered[0] = indices_scores[0];

        float best_score = indices_scores[0].score_;
        int kept = 1;
        for (int i = 1; i < num_n; ++i)
        {
          std::cout << best_score << indices_scores[i].score_ << (best_score / indices_scores[i].score_) << std::endl;
          if ((best_score / indices_scores[i].score_) > 0.75)
          {
            indices_scores_filtered[i] = indices_scores[i];
            kept++;
          }

          //best_score = indices_scores[i].score_;
        }

        indices_scores_filtered.resize (kept);
        std::cout << indices_scores_filtered.size () << " § " << num_n << std::endl;

        indices_scores = indices_scores_filtered;
        num_n = indices_scores.size ();*/

        if (do_CRH_)
        {
          /*
           * Once we have the models, we need to find a 6DOF pose using the roll histogram
           * pass to pcl_recognition::CRHAlignment both views, centroids and CRH
           */

          pcl::CRHAlignment<PointInT, 90> crha;

          for (int i = 0; i < num_n; ++i)
          {
            ModelT m = flann_models_[indices_scores[i].idx_models_].model;
            int view_id = flann_models_[indices_scores[i].idx_models_].view_id;
            int desc_id = flann_models_[indices_scores[i].idx_models_].descriptor_id;

            std::cout << m.id_ << " " << view_id << " " << desc_id << std::endl;

            //get crhs
            CRHPointCloud::Ptr input_crh = crh_histograms[indices_scores[i].idx_input_];
            CRHPointCloud::Ptr view_crh;
            getCRH (m, view_id, desc_id, view_crh);

            //get centroids
            Eigen::Vector3f input_centroid = centroids[indices_scores[i].idx_input_];
            Eigen::Vector3f view_centroid;
            getCentroid (m, view_id, desc_id, view_centroid);

            //crha.setModelAndInputView (view, processed);
            crha.setInputAndTargetCentroids (view_centroid, input_centroid);
            crha.align (*view_crh, *input_crh);

            Eigen::Matrix4f model_view_pose;
            getPose (m, view_id, model_view_pose);

            std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > roll_transforms;
            crha.getTransforms (roll_transforms);

            //create object hypothesis
            for (size_t k = 0; k < roll_transforms.size (); k++)
            {
              Eigen::Matrix4f final_roll_trans (roll_transforms[k] * model_view_pose);
              models_->push_back (m);
              transforms_->push_back (final_roll_trans);
            }
          }
        }
        else
        {
          for (int i = 0; i < num_n; ++i)
          {
            ModelT m = flann_models_[indices_scores[i].idx_models_].model;
            models_->push_back (m);
          }
        }
      }

      std::cout << "Number of object hypotheses:" << models_->size () << std::endl;

      /**
       * POSE REFINEMENT
       **/

      if (ICP_iterations_ > 0)
      {
        pcl::ScopeTime t ("Pose refinement");

        //Prepare scene and model clouds for the pose refinement step
        float VOXEL_SIZE_ICP_ = 0.005f;
        PointInTPtr cloud_voxelized_icp (new pcl::PointCloud<PointInT> ());
        pcl::VoxelGrid<PointInT> voxel_grid_icp;
        voxel_grid_icp.setInputCloud (processed);
        voxel_grid_icp.setLeafSize (VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
        voxel_grid_icp.filter (*cloud_voxelized_icp);
        source_->voxelizeAllModels (VOXEL_SIZE_ICP_);

#pragma omp parallel for num_threads(omp_get_num_procs())
        for (int i = 0; i < static_cast<int> (models_->size ()); i++)
        {

          ConstPointInTPtr model_cloud = models_->at (i).getAssembled (VOXEL_SIZE_ICP_);
          PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
          pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));

          pcl::IterativeClosestPoint<PointInT, PointInT> reg;
          reg.setInputSource (model_aligned); //model
          reg.setInputTarget (cloud_voxelized_icp); //scene
          reg.setMaximumIterations (ICP_iterations_);
          reg.setMaxCorrespondenceDistance (VOXEL_SIZE_ICP_ * 3.f);
          reg.setTransformationEpsilon (1e-5);

          typename pcl::PointCloud<PointInT>::Ptr output_ (new pcl::PointCloud<PointInT> ());
          reg.align (*output_);

          Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();
          transforms_->at (i) = icp_trans * transforms_->at (i);
        }
      }

      /**
       * HYPOTHESES VERIFICATION
       **/

      if (hv_algorithm_)
      {

        pcl::ScopeTime t ("HYPOTHESES VERIFICATION");

        std::vector<typename pcl::PointCloud<PointInT>::ConstPtr> aligned_models;
        aligned_models.resize (models_->size ());

        for (size_t i = 0; i < models_->size (); i++)
        {
          ConstPointInTPtr model_cloud = models_->at (i).getAssembled (0.005f);
          PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
          pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));
          aligned_models[i] = model_aligned;
        }

        std::vector<bool> mask_hv;
        hv_algorithm_->setSceneCloud (input_);
        hv_algorithm_->addModels (aligned_models, true);
        hv_algorithm_->verify ();
        hv_algorithm_->getMask (mask_hv);

        boost::shared_ptr < std::vector<ModelT> > models_temp;
        boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_temp;

        models_temp.reset (new std::vector<ModelT>);
        transforms_temp.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

        for (size_t i = 0; i < models_->size (); i++)
        {
          if (!mask_hv[i])
            continue;

          models_temp->push_back (models_->at (i));
          transforms_temp->push_back (transforms_->at (i));
        }

        models_ = models_temp;
        transforms_ = transforms_temp;
      }

    }
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  pcl::rec_3d_framework::GlobalNNCRHRecognizer<Distance, PointInT, FeatureT>::initialize (bool force_retrain)
  {

    //use the source to know what has to be trained and what not, checking if the descr_name directory exists
    //unless force_retrain is true, then train everything
    boost::shared_ptr < std::vector<ModelT> > models = source_->getModels ();
    std::cout << "Models size:" << models->size () << std::endl;

    if (force_retrain)
    {
      for (size_t i = 0; i < models->size (); i++)
      {
        source_->removeDescDirectory (models->at (i), training_dir_, descr_name_);
      }
    }

    for (size_t i = 0; i < models->size (); i++)
    {
      if (!source_->modelAlreadyTrained (models->at (i), training_dir_, descr_name_))
      {
        for (size_t v = 0; v < models->at (i).views_->size (); v++)
        {
          PointInTPtr processed (new pcl::PointCloud<PointInT>);
          PointInTPtr view = models->at (i).views_->at (v);

          if (noisify_)
          {
            double noise_std = noise_;
            boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration duration( time.time_of_day() );
            boost::mt19937 rng;
            rng.seed (static_cast<unsigned int> (duration.total_milliseconds()));
            boost::normal_distribution<> nd (0.0, noise_std);
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);
            // Noisify each point in the dataset
            for (size_t cp = 0; cp < view->points.size (); ++cp)
              view->points[cp].z += static_cast<float> (var_nor ());

          }

          //pro view, compute signatures and CRH
          std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures;
          std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > centroids;
          crh_estimator_->estimate (view, processed, signatures, centroids);

          std::string path = source_->getModelDescriptorDir (models->at (i), training_dir_, descr_name_);

          bf::path desc_dir = path;
          if (!bf::exists (desc_dir))
            bf::create_directory (desc_dir);

          std::stringstream path_view;
          path_view << path << "/view_" << v << ".pcd";
          pcl::io::savePCDFileBinary (path_view.str (), *processed);

          std::stringstream path_pose;
          path_pose << path << "/pose_" << v << ".txt";
          PersistenceUtils::writeMatrixToFile (path_pose.str (), models->at (i).poses_->at (v));

          std::stringstream path_entropy;
          path_entropy << path << "/entropy_" << v << ".txt";
          PersistenceUtils::writeFloatToFile (path_entropy.str (), models->at (i).self_occlusions_->at (v));

          std::vector<CRHPointCloud::Ptr> crh_histograms;
          crh_estimator_->getCRHHistograms (crh_histograms);

          //save signatures and centroids to disk
          for (size_t j = 0; j < signatures.size (); j++)
          {
            std::stringstream path_centroid;
            path_centroid << path << "/centroid_" << v << "_" << j << ".txt";
            Eigen::Vector3f centroid (centroids[j][0], centroids[j][1], centroids[j][2]);
            PersistenceUtils::writeCentroidToFile (path_centroid.str (), centroid);

            std::stringstream path_descriptor;
            path_descriptor << path << "/descriptor_" << v << "_" << j << ".pcd";
            pcl::io::savePCDFileBinary (path_descriptor.str (), signatures[j]);

            std::stringstream path_roll;
            path_roll << path << "/crh_" << v << "_" << j << ".pcd";
            pcl::io::savePCDFileBinary (path_roll.str (), *crh_histograms[j]);
          }
        }

      }
      else
      {
        //else skip model
        std::cout << "The model has already been trained..." << std::endl;
      }
    }

    //load features from disk
    //initialize FLANN structure
    loadFeaturesAndCreateFLANN ();
  }
