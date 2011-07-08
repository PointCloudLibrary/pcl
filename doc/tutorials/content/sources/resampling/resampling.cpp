#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
 main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  sensor_msgs::PointCloud2 cloud_blob;
  // Load bun0.pcd -- should be available with the PCL archive in test 
  pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
  pcl::fromROSMsg (cloud_blob, *cloud);

  // Create a KD-Tree
  pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree->setInputCloud (cloud);

  // Output has the same type as the input one, it will be only smoothed
  pcl::PointCloud<pcl::PointXYZ> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;

  // Optionally, a pointer to a cloud can be provided, to be set by MLS
  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
  mls.setOutputNormals (mls_normals);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.reconstruct (mls_points);

  // Concatenate fields for saving
  pcl::PointCloud<pcl::PointNormal> mls_cloud;
  pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

  // Save output
  pcl::io::savePCDFile ("bun0-mls.pcd", mls_cloud);
}
