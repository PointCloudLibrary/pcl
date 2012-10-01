/*
 * test_training.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include <pcl/pcl_macros.h>
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/vfh_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/cvfh_estimator.h>
#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/console/parse.h>

template<template<class > class DistT, typename PointT, typename FeatureT>
void
segmentAndClassify (typename pcl::rec_3d_framework::GlobalNNPipeline<DistT, PointT, FeatureT> & global)
{
  //get point cloud from the kinect, segment it and classify it
  OpenNIFrameSource::OpenNIFrameSource camera;
  OpenNIFrameSource::PointCloudPtr frame;

  pcl::visualization::PCLVisualizer vis ("kinect");

  //keyboard callback to stop getting frames and finalize application
  boost::function<void
  (const pcl::visualization::KeyboardEvent&)> keyboard_cb = boost::bind (&OpenNIFrameSource::OpenNIFrameSource::onKeyboardEvent, &camera, _1);
  vis.registerKeyboardCallback (keyboard_cb);
  size_t previous_cluster_size = 0;
  size_t previous_categories_size = 0;

  float Z_DIST_ = 1.25f;
  float text_scale = 0.015f;

  while (camera.isActive ())
  {
    pcl::ScopeTime frame_process ("Global frame processing ------------- ");
    frame = camera.snap ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud (*frame, *xyz_points);

    //Step 1 -> Segment
    pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
    dps.setInputCloud (xyz_points);
    dps.setMaxZBounds (Z_DIST_);
    dps.setObjectMinHeight (0.005);
    dps.setMinClusterSize (1000);
    dps.setWSize (9);
    dps.setDistanceBetweenClusters (0.1f);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<pcl::PointIndices> indices;
    dps.setDownsamplingSize (0.02f);
    dps.compute_fast (clusters);
    dps.getIndicesClusters (indices);
    Eigen::Vector4f table_plane_;
    Eigen::Vector3f normal_plane_ = Eigen::Vector3f (table_plane_[0], table_plane_[1], table_plane_[2]);
    dps.getTableCoefficients (table_plane_);

    vis.removePointCloud ("frame");
    vis.addPointCloud<OpenNIFrameSource::PointT> (frame, "frame");

    for (size_t i = 0; i < previous_cluster_size; i++)
    {
      std::stringstream cluster_name;
      cluster_name << "cluster_" << i;
      vis.removePointCloud (cluster_name.str ());

      cluster_name << "_ply_model_";
      vis.removeShape (cluster_name.str ());
    }

    for (size_t i = 0; i < previous_categories_size; i++)
    {
      std::stringstream cluster_text;
      cluster_text << "cluster_" << i << "_text";
      vis.removeText3D (cluster_text.str ());
    }

    previous_categories_size = 0;
    float dist_ = 0.03f;

    for (size_t i = 0; i < clusters.size (); i++)
    {
      std::stringstream cluster_name;
      cluster_name << "cluster_" << i;
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_handler (clusters[i]);
      vis.addPointCloud<pcl::PointXYZ> (clusters[i], random_handler, cluster_name.str ());

      global.setInputCloud (xyz_points);
      global.setIndices (indices[i].indices);
      global.classify ();

      std::vector < std::string > categories;
      std::vector<float> conf;

      global.getCategory (categories);
      global.getConfidence (conf);

      std::string category = categories[0];
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*xyz_points, indices[i].indices, centroid);
      for (size_t kk = 0; kk < categories.size (); kk++)
      {

        pcl::PointXYZ pos;
        pos.x = centroid[0] + normal_plane_[0] * static_cast<float> (kk + 1) * dist_;
        pos.y = centroid[1] + normal_plane_[1] * static_cast<float> (kk + 1) * dist_;
        pos.z = centroid[2] + normal_plane_[2] * static_cast<float> (kk + 1) * dist_;

        std::ostringstream prob_str;
        prob_str.precision (1);
        prob_str << categories[kk] << " [" << conf[kk] << "]";

        std::stringstream cluster_text;
        cluster_text << "cluster_" << previous_categories_size << "_text";
        vis.addText3D (prob_str.str (), pos, text_scale, 1, 0, 1, cluster_text.str (), 0);
        previous_categories_size++;
      }
    }

    previous_cluster_size = clusters.size ();

    vis.spinOnce ();
  }
}

//bin/pcl_global_classification -models_dir /home/aitor/data/3d-net_one_class/ -descriptor_name esf -training_dir /home/aitor/data/3d-net_one_class_trained_level_1 -nn 10

int
main (int argc, char ** argv)
{

  std::string path = "models/";
  std::string desc_name = "esf";
  std::string training_dir = "trained_models/";
  int NN = 1;

  pcl::console::parse_argument (argc, argv, "-models_dir", path);
  pcl::console::parse_argument (argc, argv, "-training_dir", training_dir);
  pcl::console::parse_argument (argc, argv, "-descriptor_name", desc_name);
  pcl::console::parse_argument (argc, argv, "-nn", NN);

  //pcl::console::parse_argument (argc, argv, "-z_dist", chop_at_z_);
  //pcl::console::parse_argument (argc, argv, "-tesselation_level", views_level_);

  boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source (new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
  mesh_source->setPath (path);
  mesh_source->setResolution (150);
  mesh_source->setTesselationLevel (1);
  mesh_source->setViewAngle (57.f);
  mesh_source->setRadiusSphere (1.5f);
  mesh_source->setModelScale (1.f);
  mesh_source->generate (training_dir);

  boost::shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
  cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > (mesh_source);

  boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
  normal_estimator.reset (new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
  normal_estimator->setCMR (true);
  normal_estimator->setDoVoxelGrid (true);
  normal_estimator->setRemoveOutliers (true);
  normal_estimator->setFactorsForCMR (3, 7);

  if (desc_name.compare ("vfh") == 0)
  {
    boost::shared_ptr<pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
    vfh_estimator.reset (new pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
    vfh_estimator->setNormalEstimator (normal_estimator);

    boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::VFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > (vfh_estimator);

    pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308> global;
    global.setDataSource (cast_source);
    global.setTrainingDir (training_dir);
    global.setDescriptorName (desc_name);
    global.setNN (NN);
    global.setFeatureEstimator (cast_estimator);
    global.initialize (true);

    segmentAndClassify<flann::L1, pcl::PointXYZ, pcl::VFHSignature308> (global);
  }

  if (desc_name.compare ("cvfh") == 0)
  {
    boost::shared_ptr<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > vfh_estimator;
    vfh_estimator.reset (new pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308>);
    vfh_estimator->setNormalEstimator (normal_estimator);

    boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::VFHSignature308> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::CVFHEstimation<pcl::PointXYZ, pcl::VFHSignature308> > (vfh_estimator);

    pcl::rec_3d_framework::GlobalNNPipeline<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308> global;
    global.setDataSource (cast_source);
    global.setTrainingDir (training_dir);
    global.setDescriptorName (desc_name);
    global.setFeatureEstimator (cast_estimator);
    global.setNN (NN);
    global.initialize (false);

    segmentAndClassify<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308> (global);
  }

  if (desc_name.compare ("esf") == 0)
  {
    boost::shared_ptr<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> > estimator;
    estimator.reset (new pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640>);

    boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::ESFSignature640> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> > (estimator);

    pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640> global;
    global.setDataSource (cast_source);
    global.setTrainingDir (training_dir);
    global.setDescriptorName (desc_name);
    global.setFeatureEstimator (cast_estimator);
    global.setNN (NN);
    global.initialize (false);

    segmentAndClassify<flann::L1, pcl::PointXYZ, pcl::ESFSignature640> (global);
  }

}
