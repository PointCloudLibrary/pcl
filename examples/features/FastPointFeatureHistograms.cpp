#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

int
main (int argc, char** argv)
{
  if(argc < 2)
  {
    throw std::runtime_error("Required arguments: filename.pcd");
  }

  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
  normalEstimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normalEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::Normal>);

  normalEstimation.setRadiusSearch (0.03);

  normalEstimation.compute (*cloudWithNormals);

  // Setup the feature computation

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
  // Provide the original point cloud (without normals)
  fpfhEstimation.setInputCloud (cloud);
  // Provide the point cloud with normals
  fpfhEstimation.setInputNormals(cloudWithNormals);

  // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation
  fpfhEstimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfhFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);

  fpfhEstimation.setRadiusSearch (0.2);

  // Actually compute the spin images
  fpfhEstimation.compute (*pfhFeatures);

  std::cout << "output points.size (): " << pfhFeatures->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::FPFHSignature33 descriptor = pfhFeatures->points[0];
  std::cout << descriptor << std::endl;

  return 0;
}
