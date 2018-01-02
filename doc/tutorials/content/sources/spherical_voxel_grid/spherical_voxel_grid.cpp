#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/spherical_voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile<pcl::PointXYZI> ("spherical_voxel_grid_example.pcd", *input);

  std::cerr << "Size before downsample: " << input->points.size() << std::endl;

  // Create the filtering object
  pcl::SphericalVoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud (input);
  voxel.setLeafSize (0.1, 90, 300);
  voxel.filter (*output);

  std::cerr << "Size after downsample: " << output->points.size() << std::endl;

  pcl::io::savePCDFileASCII ("spherical_voxel_grid_example_downsampled.pcd", *output);

  return (0);
}
