#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;

int
 main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
  sensor_msgs::PointCloud2 cloud_blob;
  // Load bun0.pcd -- should be available with the PCL archive in test 
  loadPCDFile ("bun0.pcd", cloud_blob);
  fromROSMsg (cloud_blob, *cloud);

  // Create a KD-Tree
  KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
  tree->setInputCloud (cloud);

  // Output has the same type as the input one, it will be only smoothed
  PointCloud<PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  MovingLeastSquares<PointXYZ, Normal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  PointCloud<Normal>::Ptr mls_normals (new PointCloud<Normal> ());
  mls.setOutputNormals (mls_normals);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.reconstruct (mls_points);

  // Concatenate fields for saving
  PointCloud<PointNormal> mls_cloud;
  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

  // Save output
  savePCDFile ("bun0-mls.pcd", mls_cloud);
}
