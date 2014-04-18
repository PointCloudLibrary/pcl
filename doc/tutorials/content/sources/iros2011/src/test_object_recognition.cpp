#include "solution/object_recognition.h"

#include <string>
#include <sstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

//./test_object_recognition /home/aitor/PCL_tutorial_iros/data/minnie/raw_4.pcd --objects_root_path /home/aitor/PCL_tutorial_iros/data --objects ../../data/objects.txt --filter ../../params/filter.txt --segment ../../params/segment.txt --feature ../../params/feature.txt --registration ../../params/registration.txt

int 
main (int argc, char ** argv)
{
  if (argc < 3) 
  {
    pcl::console::print_info ("Syntax is: %s query.pcd <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    --objects_root_path root_path ......... Root path to append to files in object_files.txt \n");
    pcl::console::print_info ("    --objects object_files.txt ......... A list of the files to use as exemplars\n");
    pcl::console::print_info ("    --filter parameters.txt ............ Threshold, downsample, and denoise\n");
    pcl::console::print_info ("    --segment parameters.txt  .......... Remove dominant plane and cluster\n");
    pcl::console::print_info ("    --feature parameters.txt ........... Compute normals, keypoints, and descriptors\n");
    pcl::console::print_info ("    --registration parameters.txt ...... Align best fitting model to query\n");
    pcl::console::print_info ("  and where the parameters files must contain the following values (one per line):\n");
    pcl::console::print_info ("    filter parameters:\n");
    pcl::console::print_info ("      * min_depth\n");
    pcl::console::print_info ("      * max_depth\n");
    pcl::console::print_info ("      * downsample_leaf_size\n");
    pcl::console::print_info ("      * outlier_rejection_radius\n");
    pcl::console::print_info ("      * outlier_rejection_min_neighbors\n");
    pcl::console::print_info ("    segmentation parameters:\n");
    pcl::console::print_info ("      * plane_inlier_distance_threshold\n");
    pcl::console::print_info ("      * max_ransac_iterations\n");
    pcl::console::print_info ("      * cluster_tolerance\n");
    pcl::console::print_info ("      * min_cluster_size\n");
    pcl::console::print_info ("      * max_cluster_size\n");
    pcl::console::print_info ("    feature estimation parameters:\n");
    pcl::console::print_info ("      * surface_normal_radius\n");
    pcl::console::print_info ("      * keypoints_min_scale\n");
    pcl::console::print_info ("      * keypoints_nr_octaves\n");
    pcl::console::print_info ("      * keypoints_nr_scales_per_octave\n");
    pcl::console::print_info ("      * keypoints_min_contrast\n");
    pcl::console::print_info ("      * local_descriptor_radius\n");
    pcl::console::print_info ("    registration parameters:\n");
    pcl::console::print_info ("      * initial_alignment_min_sample_distance\n");
    pcl::console::print_info ("      * initial_alignment_max_correspondence_distance\n");
    pcl::console::print_info ("      * initial_alignment_nr_iterations\n");
    pcl::console::print_info ("      * icp_max_correspondence_distance\n");
    pcl::console::print_info ("      * icp_rejection_threshold\n");
    pcl::console::print_info ("      * icp_transformation_epsilon\n");
    pcl::console::print_info ("      * icp_max_iterations\n");

    return (1);
  }

  // Load input file
  PointCloudPtr query (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *query);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], query->size ());    

  ifstream input_stream;
  ObjectRecognitionParameters params;

  // Parse the exemplar files
  std::string objects_root_path;
  pcl::console::parse_argument (argc, argv, "--objects_root_path", objects_root_path);

  std::string objects_file;
  pcl::console::parse_argument (argc, argv, "--objects", objects_file);
  std::vector<std::string> exemplar_filenames (0);
  input_stream.open (objects_file.c_str ());
  if (input_stream.is_open ())
  {
    while (input_stream.good ())
    {
      std::string filename;
      input_stream >> filename;
      if (filename.size () > 0)
      {
        exemplar_filenames.push_back (objects_root_path + "/" + filename);
      }
    }
    input_stream.close ();
  }
  
  //Parse filter parameters
  std::string filter_parameters_file;
  pcl::console::parse_argument (argc, argv, "--filter", filter_parameters_file) > 0;    
  input_stream.open (filter_parameters_file.c_str ());
  if (input_stream.is_open())
  {
    input_stream >> params.min_depth;
    input_stream >> params.max_depth;
    input_stream >> params.downsample_leaf_size;
    input_stream >> params.outlier_rejection_radius;
    input_stream >> params.outlier_rejection_min_neighbors;
    input_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the filter parameters file (%s)\n", filter_parameters_file.c_str ());
    return (1);
  }  
  
  // Parse segmentation parameters
  std::string segmentation_parameters_file;
  pcl::console::parse_argument (argc, argv, "--segment", segmentation_parameters_file) > 0;    
  input_stream.open (segmentation_parameters_file.c_str ());
  if (input_stream.is_open())
  {
    input_stream >> params.plane_inlier_distance_threshold;
    input_stream >> params.max_ransac_iterations;
    input_stream >> params.cluster_tolerance;
    input_stream >> params.min_cluster_size;
    input_stream >> params.max_cluster_size;
    input_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the segmentation parameters file (%s)\n", 
                              segmentation_parameters_file.c_str ());
    return (1);
  }

  // Parse feature estimation parameters
  std::string feature_estimation_parameters_file;
  pcl::console::parse_argument (argc, argv, "--feature", feature_estimation_parameters_file) > 0;    
  input_stream.open (feature_estimation_parameters_file.c_str ());
  if (input_stream.is_open())
  {
    input_stream >> params.surface_normal_radius;
    input_stream >> params.keypoints_min_scale;
    input_stream >> params.keypoints_nr_octaves;
    input_stream >> params.keypoints_nr_scales_per_octave;
    input_stream >> params.keypoints_min_contrast;
    input_stream >> params.local_descriptor_radius;
    input_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the feature estimation parameters file (%s)\n", 
                              feature_estimation_parameters_file.c_str ());
    return (1);
  }

  // Parse the registration parameters
  std::string registration_parameters_file;
  pcl::console::parse_argument (argc, argv, "--registration", registration_parameters_file) > 0;    
  input_stream.open (registration_parameters_file.c_str ());
  if (input_stream.is_open())
  {
    input_stream >> params.initial_alignment_min_sample_distance;
    input_stream >> params.initial_alignment_max_correspondence_distance;
    input_stream >> params.initial_alignment_nr_iterations;
    input_stream >> params.icp_max_correspondence_distance;
    input_stream >> params.icp_outlier_rejection_threshold;
    input_stream >> params.icp_transformation_epsilon;
    input_stream >> params.icp_max_iterations;
    input_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the registration parameters file (%s)\n", 
                              registration_parameters_file.c_str ());
    return (1);
  }

  // Construct the ObjectDatabase
  ObjectRecognition obj_rec (params);
  obj_rec.populateDatabase (exemplar_filenames);

  // Find the object exemplar that best matches the query
  PointCloudPtr aligned_model_points = obj_rec.recognizeAndAlignPoints (query);

  // Visualize the results
  pcl::console::print_info ("Starting visualizer... Close window to exit\n");
  pcl::visualization::PCLVisualizer vis;

  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (aligned_model_points, 255, 0, 0);
  vis.addPointCloud (aligned_model_points, red, "model");
  vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");
  vis.resetCamera ();

  vis.addPointCloud (query, "query");
  vis.spin ();

  return (0); 
}
