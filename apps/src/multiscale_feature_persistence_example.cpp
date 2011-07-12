#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;

const Eigen::Vector4f subsampling_leaf_size (0.01, 0.01, 0.01, 0.0);
const float normal_estimation_search_radius = 0.05;


void
subsampleAndCalculateNormals (PointCloud<PointXYZ>::Ptr &cloud,
                              PointCloud<PointXYZ>::Ptr &cloud_subsampled,
                              PointCloud<Normal>::Ptr &cloud_subsampled_normals)
{
  cloud_subsampled = PointCloud<PointXYZ>::Ptr (new PointCloud<PointXYZ> ());
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud);
  subsampling_filter.setLeafSize (subsampling_leaf_size);
  subsampling_filter.filter (*cloud_subsampled);

  cloud_subsampled_normals = PointCloud<Normal>::Ptr (new PointCloud<Normal> ());
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled);
  KdTreeFLANN<PointXYZ>::Ptr search_tree (new KdTreeFLANN<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
  normal_estimation_filter.compute (*cloud_subsampled_normals);
}


int
main (int argc, char **argv)
{
  if (argc != 2)
  {
    PCL_ERROR ("Syntax: ./multiscale_feature_persistence_example [path_to_cloud.pcl]\n");
    return -1;
  }

  PointCloud<PointXYZ>::Ptr cloud_scene (new PointCloud<PointXYZ> ());
  PCDReader reader;
  reader.read (argv[1], *cloud_scene);

  PointCloud<PointXYZ>::Ptr cloud_subsampled;
  PointCloud<Normal>::Ptr cloud_subsampled_normals;
  subsampleAndCalculateNormals (cloud_scene, cloud_subsampled, cloud_subsampled_normals);

  PCL_INFO ("STATS:\ninitial point cloud size: %u\nsubsampled point cloud size: %u\n", cloud_scene->points.size (), cloud_subsampled->points.size ());
  visualization::CloudViewer viewer ("Multiscale Feature Persistence Example Visualization");
  viewer.showCloud (cloud_scene, "scene");


  MultiscaleFeaturePersistence<PointXYZ, FPFHSignature33> feature_persistence;
  std::vector<float> scale_values;
  for (float x = 2.0; x < 3.6; x += 0.35)
    scale_values.push_back (x / 100.0);
  feature_persistence.setScalesVector (scale_values);
  feature_persistence.setAlpha (1.3);
  FPFHEstimation<PointXYZ, Normal, FPFHSignature33>::Ptr fpfh_estimation (new FPFHEstimation<PointXYZ, Normal, FPFHSignature33> ());
  fpfh_estimation->setInputCloud (cloud_subsampled);
  fpfh_estimation->setInputNormals (cloud_subsampled_normals);
  KdTreeFLANN<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ> ());
  fpfh_estimation->setSearchMethod (tree);
  feature_persistence.setFeatureEstimator (fpfh_estimation);
  feature_persistence.setDistanceMetric (MultiscaleFeaturePersistence<PointXYZ, FPFHSignature33>::CHI_SQUARE);

  PointCloud<FPFHSignature33>::Ptr output_features (new PointCloud<FPFHSignature33> ());
  boost::shared_ptr<std::vector<int> > output_indices (new std::vector<int> ());
  feature_persistence.determinePersistentFeatures (*output_features, output_indices);

  PCL_INFO ("persistent features cloud size: %u\n", output_features->points.size ());

  ExtractIndices<PointXYZ> extract_indices_filter;
  extract_indices_filter.setInputCloud (cloud_subsampled);
  extract_indices_filter.setIndices (output_indices);
  PointCloud<PointXYZ>::Ptr persistent_features_locations (new PointCloud<PointXYZ> ());
  extract_indices_filter.filter (*persistent_features_locations);

  viewer.showCloud (persistent_features_locations, "persistent features");
  PCL_INFO ("Persistent features have been computed. Waiting for the user to quit the visualization window.\n");

//  io::savePCDFileASCII ("persistent_features.pcd", *persistent_features_locations);
//  PCL_INFO ("\nPersistent feature locations written to persistent_features.pcd\n");

  while (!viewer.wasStopped (50));

  return 0;
}
