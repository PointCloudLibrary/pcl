#include "solution/filters.h"

#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s input.pcd <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -t min_depth,max_depth  ............... Threshold depth\n");
    pcl::console::print_info ("    -d leaf_size  ......................... Downsample\n");
    pcl::console::print_info ("    -r radius,min_neighbors ............... Radius outlier removal\n");
    pcl::console::print_info ("    -s output.pcd ......................... Save output\n");
    return (1);
  }

  // Load the input file
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());

  // Threshold depth
  double min_depth, max_depth;
  bool threshold_depth = pcl::console::parse_2x_arguments (argc, argv, "-t", min_depth, max_depth) > 0;
  if (threshold_depth)
  {
    size_t n = cloud->size ();
    cloud = thresholdDepth (cloud, min_depth, max_depth);
    pcl::console::print_info ("Eliminated %lu points outside depth limits\n", n - cloud->size ());
  }

  // Downsample and threshold depth
  double leaf_size;
  bool downsample_cloud = pcl::console::parse_argument (argc, argv, "-d", leaf_size) > 0;
  if (downsample_cloud)
  {
    size_t n = cloud->size ();
    cloud = downsample (cloud, leaf_size);
    pcl::console::print_info ("Downsampled from %lu to %lu points\n", n, cloud->size ());
  }

  // Remove outliers
  double radius, min_neighbors;
  bool remove_outliers = pcl::console::parse_2x_arguments (argc, argv, "-r", radius, min_neighbors) > 0;
  if (remove_outliers)
  {
    size_t n = cloud->size ();
    cloud = removeOutliers (cloud, radius, (int)min_neighbors);
    pcl::console::print_info ("Removed %lu outliers\n", n - cloud->size ());
  }

  // Save output
  std::string output_filename;
  bool save_cloud = pcl::console::parse_argument (argc, argv, "-s", output_filename) > 0;
  if (save_cloud)
  {
    pcl::io::savePCDFile (output_filename, *cloud);
    pcl::console::print_info ("Saved result as %s\n", output_filename.c_str ());
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (cloud);
    vis.resetCamera ();
    vis.spin ();
  }

  return (0);
}
