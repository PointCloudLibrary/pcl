#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/spherical_voxel_grid.h>

int
main ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> output;

  pcl::io::loadPCDFile<pcl::PointXYZI> ("spherical_voxel_grid_example.pcd", *input);

  std::cout << "Size before downsample: " << input->size () << std::endl;

  // Create the filtering object
  pcl::SphericalVoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud (input);
  voxel.setLeafSize (0.1, 90, 300);
  voxel.filter (output);

  std::cout << "Size after downsample: " << output.size () << std::endl;

  pcl::io::savePCDFileASCII ("spherical_voxel_grid_example_downsampled.pcd", output);

  return (0);
}
