#include <pcl/features/ppf.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ppf_registration.h>

using namespace pcl;

#include <iostream>
using namespace std;

const Eigen::Vector4f subsampling_leaf_size (9.0, 9.0, 9.0, 0.0);
const float normal_estimation_search_radius = 25.0;

int
main (int argc, char** argv)
{
  if (argc != 3)
  {
    PCL_ERROR ("Syntax: ./ppf_object_recognition pcd_model pcd_scene\n");
    return -1;
  }


  /// read point clouds from HDD
  PointCloud<PointXYZ>::Ptr cloud_model (new PointCloud<PointXYZ> ()),
      cloud_scene (new PointCloud<PointXYZ> ());
  PCDReader reader;
  reader.read (argv[1], *cloud_model);
  reader.read (argv[2], *cloud_scene);

  /// subsample both model and scene cloud with the same parameters
  PointCloud<PointXYZ>::Ptr cloud_model_subsampled (new PointCloud<PointXYZ> ()),
      cloud_scene_subsampled (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud_model);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_model_subsampled);
  subsampling_filter.setInputCloud (cloud_scene);
  subsampling_filter.filter (*cloud_scene_subsampled);

  /// calculate normals for both model and scene cloud with the same parameters
  PointCloud<Normal>::Ptr cloud_model_subsampled_normals (new PointCloud<Normal> ()),
      cloud_scene_subsampled_normals (new PointCloud<Normal> ());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_model_subsampled);
  KdTreeFLANN<PointXYZ>::Ptr search_tree (new KdTreeFLANN<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (*cloud_model_subsampled_normals);
  normal_estimation_filter.setInputCloud (cloud_scene_subsampled);
  normal_estimation_filter.compute (*cloud_scene_subsampled_normals);

  PointCloud<PPFSignature>::Ptr cloud_model_ppf (new PointCloud<PPFSignature> ());
  PPFEstimation<PointXYZ, Normal, PPFSignature> ppf_estimator;
  ppf_estimator.setInputCloud (cloud_model_subsampled);
  ppf_estimator.setInputNormals (cloud_model_subsampled_normals);
  ppf_estimator.computeFeature (*cloud_model_ppf);


  PCL_INFO("Model cloud size: %u\nsubsampled cloud size: %u\nppf cloud size: %u\n",
           cloud_model->points.size (), cloud_model_subsampled->points.size (), cloud_model_ppf->points.size ());

  PPFRegistration<PointXYZ, Normal> ppf_registration (5,
                                                      50,
                                                      25.0 / 180 * M_PI);
  PPFHashMapSearch::Ptr hashmap_search (new PPFHashMapSearch (12.0 / 180 * M_PI,
                                                              20));
  hashmap_search->setInputFeatureCloud (cloud_model_ppf);
  ppf_registration.setSearchMethod (hashmap_search);
  ppf_registration.setSourceClouds (cloud_model_subsampled->makeShared (),
                                    cloud_model_subsampled_normals->makeShared ());
  ppf_registration.setInputTarget (cloud_scene_subsampled);
  ppf_registration.setInputTargetNormals (cloud_scene_subsampled_normals);

  PointCloud<PointXYZ> cloud_output;
  ppf_registration.computeTransformation (cloud_output);

  io::savePCDFileASCII("output_registered.pcd", cloud_output);

  return 0;
}
