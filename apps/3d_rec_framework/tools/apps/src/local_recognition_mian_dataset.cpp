/*
 * local_recognition_mian_dataset.cpp
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */
#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/console/parse.h>
#include <pcl/apps/3d_rec_framework/pipeline/local_recognizer.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/shot_local_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/shot_local_estimator_omp.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/fpfh_local_estimator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/recognition/hv/greedy_verification.h>

void
getScenesInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
{
  //list models in MODEL_FILES_DIR_ and return list
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {

#if BOOST_FILESYSTEM_VERSION == 3
      std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
      std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif
      bf::path curr_path = itr->path ();
      getScenesInDirectory (curr_path, so_far, relative_paths);
    }
    else
    {
      //check that it is a ply file and then add, otherwise ignore..
      std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = (itr->path ().filename ()).string();
#else
      std::string file = (itr->path ().filename ());
#endif

      boost::split (strs, file, boost::is_any_of ("."));
      std::string extension = strs[strs.size () - 1];

      if (extension == "pcd")
      {

#if BOOST_FILESYSTEM_VERSION == 3
        std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
        std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

        relative_paths.push_back (path);
      }
    }
  }
}

inline bool
sortFiles (const std::string & file1, const std::string & file2)
{
  std::vector < std::string > strs1;
  boost::split (strs1, file1, boost::is_any_of ("/"));

  std::vector < std::string > strs2;
  boost::split (strs2, file2, boost::is_any_of ("/"));

  std::string id_1 = strs1[strs1.size () - 1];
  std::string id_2 = strs2[strs2.size () - 1];

  size_t pos1 = id_1.find (".ply.pcd");
  size_t pos2 = id_2.find (".ply.pcd");

  id_1 = id_1.substr (0, pos1);
  id_2 = id_2.substr (0, pos2);

  id_1 = id_1.substr (2);
  id_2 = id_2.substr (2);

  return atoi (id_1.c_str ()) < atoi (id_2.c_str ());
}

template<template<class > class DistT, typename PointT, typename FeatureT>
  void
  recognizeAndVisualize (typename pcl::rec_3d_framework::LocalRecognitionPipeline<DistT, PointT, FeatureT> & local, std::string & scenes_dir,
                         int scene = -1, bool single_model = false)
  {

    //read mians scenes
    bf::path ply_files_dir = scenes_dir;
    std::vector < std::string > files;
    std::string start = "";
    getScenesInDirectory (ply_files_dir, start, files);

    std::sort (files.begin (), files.end (), sortFiles);

    typename boost::shared_ptr<pcl::rec_3d_framework::Source<PointT> > model_source_ = local.getDataSource ();
    typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;
    typedef pcl::rec_3d_framework::Model<PointT> ModelT;

    if (!single_model)
    {
      pcl::visualization::PCLVisualizer vis ("Mians dataset");

      for (size_t i = 0; i < files.size (); i++)
      {
        std::cout << files[i] << std::endl;
        if (scene != -1)
          if (scene != i)
            continue;

        std::stringstream file;
        file << ply_files_dir.string () << files[i];

        typename pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT> ());
        pcl::io::loadPCDFile (file.str (), *scene);

        local.setVoxelSizeICP (0.005f);
        local.setInputCloud (scene);
        {
          pcl::ScopeTime ttt ("Recognition");
          local.recognize ();
        }

        std::stringstream scene_name;
        scene_name << "Scene " << (i + 1);
        vis.addPointCloud<PointT> (scene, "scene_cloud");
        vis.addText (scene_name.str (), 1, 30, 24, 1, 0, 0, "scene_text");

        //visualize results
        boost::shared_ptr < std::vector<ModelT> > models = local.getModels ();
        boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = local.getTransforms ();

        for (size_t j = 0; j < models->size (); j++)
        {
          std::stringstream name;
          name << "cloud_" << j;

          ConstPointInTPtr model_cloud = models->at (j).getAssembled (0.0025f);
          typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
          pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));

          float r, g, b;
          std::cout << models->at (j).id_ << std::endl;
          r = 255.0f;
          g = 0.0f;
          b = 0.0f;

          if (models->at (j).id_.compare ("cheff") == 0)
          {
            r = 0.0f;
            g = 255.0f;
            b = 0.0f;
          }
          else if (models->at (j).id_.compare ("chicken_high") == 0)
          {
            r = 0.0f;
            g = 255.0f;
            b = 255.0f;
          }
          else if (models->at (j).id_.compare ("parasaurolophus_high") == 0)
          {
            r = 255.0f;
            g = 255.0f;
            b = 0.f;
          }
          else
          {

          }

          pcl::visualization::PointCloudColorHandlerCustom<PointT> random_handler (model_aligned, r, g, b);
          vis.addPointCloud<PointT> (model_aligned, random_handler, name.str ());
        }

        vis.spin ();

        vis.removePointCloud ("scene_cloud");
        vis.removeShape ("scene_text");
        for (size_t j = 0; j < models->size (); j++)
        {
          std::stringstream name;
          name << "cloud_" << j;
          vis.removePointCloud (name.str ());
        }
      }
    }

    if (single_model)
    {
      //some applications prefer to search for a single object only
      std::string id = "chicken_high";
      pcl::visualization::PCLVisualizer vis ("Single model - chicken");
      local.setSearchModel (id);
      local.initialize ();

      for (size_t i = 0; i < files.size (); i++)
      {
        std::stringstream file;
        file << ply_files_dir.string () << files[i];

        typename pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT> ());
        pcl::io::loadPCDFile (file.str (), *scene);

        local.setInputCloud (scene);
        local.recognize ();

        std::stringstream scene_name;
        scene_name << "Scene " << (i + 1);
        vis.addPointCloud<PointT> (scene, "scene_cloud");
        vis.addText (scene_name.str (), 1, 30, 24, 1, 0, 0, "scene_text");

        //visualize results
        boost::shared_ptr < std::vector<ModelT> > models = local.getModels ();
        boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = local.getTransforms ();

        for (size_t j = 0; j < models->size (); j++)
        {
          std::stringstream name;
          name << "cloud_" << j;

          ConstPointInTPtr model_cloud = models->at (j).getAssembled (0.0025f);
          typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
          pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));

          pcl::visualization::PointCloudColorHandlerRandom<PointT> random_handler (model_aligned);
          vis.addPointCloud<PointT> (model_aligned, random_handler, name.str ());
        }

        vis.spin ();

        vis.removePointCloud ("scene_cloud");
        vis.removeShape ("scene_text");
        for (size_t j = 0; j < models->size (); j++)
        {
          std::stringstream name;
          name << "cloud_" << j;
          vis.removePointCloud (name.str ());
        }
      }
    }
  }

void
getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
{
  bf::directory_iterator end_itr;
  for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (*itr))
    {
#if BOOST_FILESYSTEM_VERSION == 3
      std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
      std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

      bf::path curr_path = itr->path ();
      getModelsInDirectory (curr_path, so_far, relative_paths, ext);
    }
    else
    {
      //check that it is a ply file and then add, otherwise ignore..
      std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = (itr->path ().filename ()).string();
#else
      std::string file = (itr->path ()).filename ();
#endif

      boost::split (strs, file, boost::is_any_of ("."));
      std::string extension = strs[strs.size () - 1];

      if (extension.compare (ext) == 0)
      {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
        std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

        relative_paths.push_back (path);
      }
    }
  }
}

typedef pcl::ReferenceFrame RFType;

int CG_SIZE_ = 3;
float CG_THRESHOLD_ = 0.005f;

/** Based on the paper:
  * "A Global Hypotheses Verification Method for 3D Object Recognition",
  * A. Aldoma and F. Tombari and L. Di Stefano and Markus Vincze, ECCV 2012
  *
  * Note: Due to changes PCL experimented between submission and the current status,
  * you might find some inconsistencies between parameter value in code and in the paper.
  * (tested on revision 7453)
  */

int
main (int argc, char ** argv)
{
  std::string path = "";
  std::string desc_name = "shot_omp";
  std::string training_dir = "trained_models/";
  std::string mians_scenes = "";
  int force_retrain = 0;
  int icp_iterations = 20;
  int use_cache = 1;
  int splits = 512;
  int scene = -1;
  int detect_clutter = 1;
  int hv_method = 0;
  int use_hv = 1;
  float thres_hyp_ = 0.2f;
  float desc_radius = 0.04f;

  pcl::console::parse_argument (argc, argv, "-models_dir", path);
  pcl::console::parse_argument (argc, argv, "-training_dir", training_dir);
  pcl::console::parse_argument (argc, argv, "-descriptor_name", desc_name);
  pcl::console::parse_argument (argc, argv, "-mians_scenes_dir", mians_scenes);
  pcl::console::parse_argument (argc, argv, "-force_retrain", force_retrain);
  pcl::console::parse_argument (argc, argv, "-icp_iterations", icp_iterations);
  pcl::console::parse_argument (argc, argv, "-use_cache", use_cache);
  pcl::console::parse_argument (argc, argv, "-splits", splits);
  pcl::console::parse_argument (argc, argv, "-gc_size", CG_SIZE_);
  pcl::console::parse_argument (argc, argv, "-gc_threshold", CG_THRESHOLD_);
  pcl::console::parse_argument (argc, argv, "-scene", scene);
  pcl::console::parse_argument (argc, argv, "-detect_clutter", detect_clutter);
  pcl::console::parse_argument (argc, argv, "-hv_method", hv_method);
  pcl::console::parse_argument (argc, argv, "-use_hv", use_hv);
  pcl::console::parse_argument (argc, argv, "-thres_hyp", thres_hyp_);

  if (mians_scenes.compare ("") == 0)
  {
    PCL_ERROR("Set the directory containing mians scenes using the -mians_scenes_dir [dir] option\n");
    return -1;
  }

  if (path.compare ("") == 0)
  {
    PCL_ERROR("Set the directory containing the models of mian dataset using the -models_dir [dir] option\n");
    return -1;
  }

  bf::path models_dir_path = path;
  if (!bf::exists (models_dir_path))
  {
    PCL_ERROR("Models dir path %s does not exist, use -models_dir [dir] option\n", path.c_str());
    return -1;
  }
  else
  {
    std::vector < std::string > files;
    std::string start = "";
    std::string ext = std::string ("ply");
    bf::path dir = models_dir_path;
    getModelsInDirectory (dir, start, files, ext);
    assert (files.size () == 4);
  }

  //configure mesh source
  boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source (new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
  mesh_source->setPath (path);
  mesh_source->setResolution (250);
  mesh_source->setTesselationLevel (1);
  mesh_source->setViewAngle (57.f);
  mesh_source->setRadiusSphere (1.5f);
  mesh_source->setModelScale (0.001f);
  mesh_source->generate (training_dir);

  boost::shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
  cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > (mesh_source);

  //configure normal estimator
  boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
  normal_estimator.reset (new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
  normal_estimator->setCMR (false);
  normal_estimator->setDoVoxelGrid (true);
  normal_estimator->setRemoveOutliers (true);
  normal_estimator->setValuesForCMRFalse (0.003f, 0.012f);

  //configure keypoint extractor
  boost::shared_ptr<pcl::rec_3d_framework::UniformSamplingExtractor<pcl::PointXYZ> >
                                                                                     uniform_keypoint_extractor (
                                                                                                                 new pcl::rec_3d_framework::UniformSamplingExtractor<
                                                                                                                     pcl::PointXYZ>);
  //uniform_keypoint_extractor->setSamplingDensity (0.01f);
  uniform_keypoint_extractor->setSamplingDensity (0.005f);
  uniform_keypoint_extractor->setFilterPlanar (true);

  boost::shared_ptr<pcl::rec_3d_framework::KeypointExtractor<pcl::PointXYZ> > keypoint_extractor;
  keypoint_extractor = boost::static_pointer_cast<pcl::rec_3d_framework::KeypointExtractor<pcl::PointXYZ> > (uniform_keypoint_extractor);

  //configure cg algorithm (geometric consistency grouping)
  boost::shared_ptr<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ> > cast_cg_alg;
  boost::shared_ptr<pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> > gcg_alg (
                                                                                               new pcl::GeometricConsistencyGrouping<pcl::PointXYZ,
                                                                                                   pcl::PointXYZ>);
  gcg_alg->setGCThreshold (CG_SIZE_);
  gcg_alg->setGCSize (CG_THRESHOLD_);
  cast_cg_alg = boost::static_pointer_cast<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ> > (gcg_alg);

  //configure hypothesis verificator
  boost::shared_ptr<pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> > papazov (new pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ>);
  papazov->setResolution (0.005f);
  papazov->setInlierThreshold (0.005f);
  papazov->setSupportThreshold (0.08f);
  papazov->setPenaltyThreshold (0.05f);
  papazov->setConflictThreshold (0.02f);
  papazov->setOcclusionThreshold (0.01f);

  boost::shared_ptr<pcl::GlobalHypothesesVerification<pcl::PointXYZ, pcl::PointXYZ> > go (
                                                                                          new pcl::GlobalHypothesesVerification<pcl::PointXYZ,
                                                                                              pcl::PointXYZ>);
  go->setResolution (0.005f);
  go->setMaxIterations (7000);
  go->setInlierThreshold (0.005f);
  go->setRadiusClutter (0.04f);
  go->setRegularizer (3.f);
  go->setClutterRegularizer (7.5f);
  go->setDetectClutter (detect_clutter);
  go->setOcclusionThreshold (0.01f);

  boost::shared_ptr<pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> > greedy (new pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> (3.f));
  greedy->setResolution (0.005f);
  greedy->setInlierThreshold (0.005f);
  greedy->setOcclusionThreshold (0.01f);

  boost::shared_ptr<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > cast_hv_alg;

  switch (hv_method)
  {
    case 1:
      cast_hv_alg = boost::static_pointer_cast<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > (greedy);
      break;
    case 2:
      cast_hv_alg = boost::static_pointer_cast<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > (papazov);
      break;
    default:
      cast_hv_alg = boost::static_pointer_cast<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > (go);
  }

  if (desc_name.compare ("shot") == 0)
  {
    boost::shared_ptr<pcl::rec_3d_framework::SHOTLocalEstimation<pcl::PointXYZ, pcl::Histogram<352> > > estimator;
    estimator.reset (new pcl::rec_3d_framework::SHOTLocalEstimation<pcl::PointXYZ, pcl::Histogram<352> >);
    estimator->setNormalEstimator (normal_estimator);
    estimator->addKeypointExtractor (keypoint_extractor);
    estimator->setSupportRadius (0.04f);

    boost::shared_ptr<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > (estimator);

    pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > local;
    local.setDataSource (cast_source);
    local.setTrainingDir (training_dir);
    local.setDescriptorName (desc_name);
    local.setFeatureEstimator (cast_estimator);
    local.setCGAlgorithm (cast_cg_alg);
    if (use_hv)
      local.setHVAlgorithm (cast_hv_alg);
    local.setUseCache (static_cast<bool> (use_cache));
    local.initialize (static_cast<bool> (force_retrain));

    uniform_keypoint_extractor->setSamplingDensity (0.005f);
    local.setICPIterations (icp_iterations);
    local.setKdtreeSplits (splits);

    recognizeAndVisualize<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > (local, mians_scenes);

  }

  if (desc_name.compare ("shot_omp") == 0)
  {
    desc_name = std::string ("shot");
    boost::shared_ptr<pcl::rec_3d_framework::SHOTLocalEstimationOMP<pcl::PointXYZ, pcl::Histogram<352> > > estimator;
    estimator.reset (new pcl::rec_3d_framework::SHOTLocalEstimationOMP<pcl::PointXYZ, pcl::Histogram<352> >);
    estimator->setNormalEstimator (normal_estimator);
    estimator->addKeypointExtractor (keypoint_extractor);
    //estimator->setSupportRadius (0.04f);
    estimator->setSupportRadius (desc_radius);

    boost::shared_ptr<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > (estimator);

    pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > local;
    local.setDataSource (cast_source);
    local.setTrainingDir (training_dir);
    local.setDescriptorName (desc_name);
    local.setFeatureEstimator (cast_estimator);
    local.setCGAlgorithm (cast_cg_alg);
    if (use_hv)
      local.setHVAlgorithm (cast_hv_alg);

    local.setUseCache (static_cast<bool> (use_cache));
    local.initialize (static_cast<bool> (force_retrain));
    local.setThresholdAcceptHyp (thres_hyp_);

    uniform_keypoint_extractor->setSamplingDensity (0.005f);
    local.setICPIterations (icp_iterations);
    local.setKdtreeSplits (splits);

    recognizeAndVisualize<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > (local, mians_scenes, scene);

  }

  if (desc_name.compare ("fpfh") == 0)
  {
    boost::shared_ptr<pcl::rec_3d_framework::FPFHLocalEstimation<pcl::PointXYZ, pcl::FPFHSignature33> > estimator;
    estimator.reset (new pcl::rec_3d_framework::FPFHLocalEstimation<pcl::PointXYZ, pcl::FPFHSignature33>);
    estimator->setNormalEstimator (normal_estimator);
    estimator->addKeypointExtractor (keypoint_extractor);
    estimator->setSupportRadius (0.04f);

    boost::shared_ptr<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::FPFHSignature33> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::FPFHSignature33> > (estimator);

    pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33> local;
    local.setDataSource (cast_source);
    local.setTrainingDir (training_dir);
    local.setDescriptorName (desc_name);
    local.setFeatureEstimator (cast_estimator);
    local.setCGAlgorithm (cast_cg_alg);
    if (use_hv)
      local.setHVAlgorithm (cast_hv_alg);

    local.setUseCache (static_cast<bool> (use_cache));
    local.initialize (static_cast<bool> (force_retrain));

    uniform_keypoint_extractor->setSamplingDensity (0.005f);
    local.setICPIterations (icp_iterations);
    local.setKdtreeSplits (splits);

    recognizeAndVisualize<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33> (local, mians_scenes);
  }
}
