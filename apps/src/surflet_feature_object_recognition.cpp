#include <pcl/features/surflet.h>
#include <pcl/io/pcd_io.h>

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

  SurfletModelEstimation<PointXYZ, Normal> surflet_model_estimation;
  pcl::SurfletModelEstimation<pcl::PointXYZ, pcl::Normal>::FeatureHashMapTypePtr feature_hash_map = surflet_model_estimation.computeSurfletModel(cloud_model);

  /// read scene point cloud and extract MGML feature model hash map
  PointCloud<PointXYZ> cloud_scene;
  reader.read (argv[2], cloud_scene);



  return 0;
}
