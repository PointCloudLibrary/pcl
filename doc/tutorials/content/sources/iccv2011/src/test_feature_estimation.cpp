#include "feature_estimation.h"

#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s input.pcd <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -n radius  ...................................... Estimate surface normals\n");
    pcl::console::print_info ("    -k min_scale,nr_octaves,nr_scales,min_contrast... Detect keypoints\n");
    pcl::console::print_info ("    -l radius ....................................... Compute local descriptors\n");
    pcl::console::print_info ("    -s output_name (without .pcd extension).......... Save outputs\n");
    pcl::console::print_info ("Note: \n");
    pcl::console::print_info ("  Each of the first four options depends on the options above it.\n");
    pcl::console::print_info ("  Saving (-s) will output individual files for each option used (-n,-k,-f,-g).\n");
    return (1);
  }
  

  // Load the input file
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());
  
  // Estimate surface normals
  SurfaceNormalsPtr normals;
  double surface_radius;
  bool estimate_surface_normals = pcl::console::parse_argument (argc, argv, "-n", surface_radius) > 0;
  
  if (estimate_surface_normals)
  {  
    normals = estimateSurfaceNormals (cloud, surface_radius);
    pcl::console::print_info ("Estimated surface normals\n");
  }
  
  // Detect keypoints
  PointCloudPtr keypoints;
  std::string params_string;
  bool detect_keypoints = pcl::console::parse_argument (argc, argv, "-k", params_string) > 0;
  if (detect_keypoints)
  {
    assert (normals);
    std::vector<std::string> tokens;
    boost::split (tokens, params_string, boost::is_any_of (","), boost::token_compress_on);
    assert (tokens.size () == 4);    
    float min_scale = atof(tokens[0].c_str ());
    int nr_octaves = atoi(tokens[1].c_str ());
    int nr_scales = atoi(tokens[2].c_str ());
    float min_contrast = atof(tokens[3].c_str ());
    keypoints = detectKeypoints (cloud, normals, min_scale, nr_octaves, nr_scales, min_contrast);
    pcl::console::print_info ("Detected %lu keypoints\n", keypoints->size ());
  }
  
  // Compute local descriptors
  LocalDescriptorsPtr local_descriptors;
  double feature_radius;
  bool compute_local_descriptors = pcl::console::parse_argument (argc, argv, "-l", feature_radius) > 0;
  if (compute_local_descriptors)
  {
    assert (normals && keypoints);
    local_descriptors = computeLocalDescriptors (cloud, normals, keypoints, feature_radius);
    pcl::console::print_info ("Computed local descriptors\n");
  }

  // Compute global descriptor
  GlobalDescriptorsPtr global_descriptor;
  if (normals)
  {
    global_descriptor = computeGlobalDescriptor (cloud, normals);
    pcl::console::print_info ("Computed global descriptor\n");
  }

  // Save output
  std::string base_filename, output_filename;
  bool save_cloud = pcl::console::parse_argument (argc, argv, "-s", base_filename) > 0;
  if (save_cloud)
  {
    if (normals)
    {
      output_filename = base_filename;
      output_filename.append ("_normals.pcd");
      pcl::io::savePCDFile (output_filename, *normals);
      pcl::console::print_info ("Saved surface normals as %s\n", output_filename.c_str ());
    }
    if (keypoints)
    {
      output_filename = base_filename;
      output_filename.append ("_keypoints.pcd");
      pcl::io::savePCDFile (output_filename, *keypoints);
      pcl::console::print_info ("Saved keypoints as %s\n", output_filename.c_str ());
    }
    if (local_descriptors)
    {
      output_filename = base_filename;
      output_filename.append ("_localdesc.pcd");
      pcl::io::savePCDFile (output_filename, *local_descriptors);
      pcl::console::print_info ("Saved local descriptors as %s\n", output_filename.c_str ());
    }
    if (global_descriptor)
    {
      output_filename = base_filename;
      output_filename.append ("_globaldesc.pcd");
      pcl::io::savePCDFile (output_filename, *global_descriptor);
      pcl::console::print_info ("Saved global descriptor as %s\n", output_filename.c_str ());
    }
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit\n");
    pcl::visualization::PCLVisualizer vis;
    pcl::visualization::PCLHistogramVisualizer hist_vis;
    vis.addPointCloud (cloud);
    if (normals)
    {
      vis.addPointCloudNormals<PointT,NormalT> (cloud, normals, 4, 0.02, "normals");
    }
    if (keypoints)
    {
      pcl::visualization::PointCloudColorHandlerCustom<PointT> red (keypoints, 255, 0, 0);
      vis.addPointCloud (keypoints, red, "keypoints");
      vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");
    }
    if (global_descriptor)
    {
      hist_vis.addFeatureHistogram (*global_descriptor, 308, "Global descriptor");
    }
    vis.resetCamera ();
    vis.spin ();
  }

  return (0);
}
