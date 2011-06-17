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

  SurfletEstimation<PointXYZ, Normal> surflet_model_estimation (12.0 / 180 * M_PI,
                                                                20,
                                                                50,
                                                                25.0 / 180 * M_PI,
                                                                5,
                                                                Eigen::Vector3f (9, 9, 9),
                                                                25);
  pcl::SurfletEstimation<pcl::PointXYZ, pcl::Normal>::SurfletModel surflet_model;
  surflet_model_estimation.computeSurfletModel(cloud_model, surflet_model);
  cerr << "Max_dist in Surflet model: " << surflet_model.max_dist << endl;

  /// read scene point cloud and extract MGML feature model hash map
  PointCloud<PointXYZ> cloud_scene;
  reader.read (argv[2], cloud_scene);



  /// subsample scene cloud with same rate as the model cloud
  PointCloud<PointXYZ> cloud_model_subsampled;
  /// subsample point cloud such that min dist between points is d_dist (+parameter)
  VoxelGrid<PointXYZ> subsampling_filter;
  subsampling_filter.setInputCloud (cloud_model.makeShared ());
  subsampling_filter.setLeafSize (9, 9, 9); /// @TODO parameter goes here
  subsampling_filter.filter (cloud_model_subsampled);

  /// calculate subsampled scene cloud normals
  PointCloud<Normal> cloud_model_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  NormalEstimation<PointXYZ, Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_model_subsampled.makeShared());
  typename KdTreeFLANN<PointXYZ>::Ptr search_tree (new KdTreeFLANN<PointXYZ>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch(25); // @TODO another parameter
  normal_estimation_filter.compute (cloud_model_subsampled_normals);

  SurfletEstimation<PointXYZ, Normal>::PoseWithVotesList registration_results;
  surflet_model_estimation.registerModelToScene(cloud_model_subsampled, cloud_model_subsampled_normals, cloud_scene, surflet_model, registration_results);
  for(size_t i_res = 0; i_res < registration_results.size (); ++ i_res)
  {
    cerr << "registration #" << i_res << " received votes: " << registration_results[i_res].votes << endl;
    pcl::PointCloud<PointXYZ> cloud_output;
    for(size_t i = 0; i < cloud_model.width; ++i)
    {
      Eigen::Vector3f point = cloud_model.points[i].getVector3fMap ();
      point = registration_results[i_res].pose * point;
      cloud_output.points.push_back (PointXYZ(point[0], point[1], point[2]));
    }
    stringstream output_pcd_name; output_pcd_name << "output_aligned_" << i_res << ".pcd";
    io::savePCDFileASCII (output_pcd_name.str (), cloud_output);
    cerr << "Output pcl written to file: " << output_pcd_name.str () << endl;
    cerr << "Transform: " << registration_results[i_res].pose.translation() << "     " << registration_results[i_res].pose.rotation() << endl;


    //// debugging
    Eigen::Affine3f test_trans = registration_results[i_res].pose,
        test_trans2 = Eigen::AngleAxisf (test_trans.rotation()) * Eigen::Translation3f (test_trans.translation());
    cerr << "TEST: " << test_trans * Eigen::Vector3f(1, 1, 1) << "    ----     "
        << test_trans2 * Eigen::Vector3f(1, 1, 1) << endl;
  }



  return 0;
}
