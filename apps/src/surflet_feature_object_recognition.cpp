#include <pcl/features/surflet.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
using namespace std;

using namespace pcl;

int
main (int argc, char** argv)
{
  if (argc != 3)
  {
    cerr << "Syntax: ./surflet_feature_object_recognition pcd_model pcd_scene" << endl;
    return -1;
  }

  /// read model point cloud and extract MGML feature model hash map
  PointCloud<PointXYZ> cloud_model;
  PCDReader reader;
  reader.read (argv[1], cloud_model);

  SurfletEstimation<PointXYZ, Normal> surflet_model_estimation;
  pcl::SurfletEstimation<pcl::PointXYZ, pcl::Normal>::FeatureHashMapTypePtr feature_hash_map = surflet_model_estimation.computeSurfletModel(cloud_model);

  /// read scene point cloud and extract MGML feature model hash map
  PointCloud<PointXYZ> cloud_scene;
  reader.read (argv[2], cloud_scene);



  /// subsample scene cloud with same rate as the model cloud
  pcl::PointCloud<PointXYZ> cloud_model_subsampled;
  /// subsample point cloud such that min dist between points is d_dist (+parameter)
  pcl::VoxelGrid<PointXYZ> subsampling_filter;
  /// @todo don't make a copy here
  subsampling_filter.setInputCloud (cloud_model.makeShared ());
  subsampling_filter.setLeafSize (0.01, 0.01, 0.01); /// @TODO parameter goes here
  subsampling_filter.filter (cloud_model_subsampled);

  /// calculate subsampled scene cloud normals
  pcl::PointCloud<Normal> cloud_model_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointXYZ, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_model_subsampled.makeShared());
  typename pcl::KdTreeFLANN<PointXYZ>::Ptr search_tree (new pcl::KdTreeFLANN<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch(0.05); // @TODO another parameter
  normal_estimation_filter.compute (cloud_model_subsampled_normals);

  pcl::SurfletEstimation<pcl::PointXYZ, pcl::Normal>::PoseWithVotesList registration_results = surflet_model_estimation.registerModelToScene(cloud_model_subsampled, cloud_model_subsampled_normals, cloud_scene, feature_hash_map);
  for(size_t i_res = 0; i_res < 10 /*registration_results.size()*/; ++ i_res)
  {
    cerr << "registration #" << i_res << " received votes: " << registration_results[i_res].votes << endl;
    pcl::PointCloud<PointXYZ> cloud_output;
    for(size_t i = 0; i < cloud_model.width; ++i)
    {
      /// @TODO find some native pcl way of doing this - too many conversions
      Eigen::Vector3f point (cloud_model.points[i].x, cloud_model.points[i].y, cloud_model.points[i].z);
      point = registration_results[i_res].pose * point;
      cloud_output.points.push_back (PointXYZ(point[0], point[1], point[2]));
    }
    stringstream output_pcd_name; output_pcd_name << "output_aligned_" << i_res << ".pcd";
    pcl::io::savePCDFileASCII (output_pcd_name.str (), cloud_output);
    cerr << "Output pcl written to file: " << output_pcd_name.str () << endl;
    cerr << "Transform: " << registration_results[i_res].pose.translation() << "     " << registration_results[i_res].pose.rotation() << endl;
  }



  return 0;
}
