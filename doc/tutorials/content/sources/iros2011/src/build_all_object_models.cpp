#include "object_recognition.h"

#include <string>
#include <sstream>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp> // for split, is_any_of
namespace bf = boost::filesystem;

inline void
getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
{
  for (const auto& dir_entry : bf::directory_iterator(dir))
  {
    //check if its a directory, then get models in it
    if (bf::is_directory (dir_entry))
    {
      std::string so_far = rel_path_so_far + dir_entry.path ().filename ().string () + "/";
      bf::path curr_path = dir_entry.path ();
      getModelsInDirectory (curr_path, so_far, relative_paths);
    }
    else
    {
      std::vector<std::string> strs;
      std::string file = dir_entry.path ().filename ().string ();
      boost::split (strs, file, boost::is_any_of ("."));
      std::string extension = strs[strs.size () - 1];

      if((file.compare (0, 3, "raw") == 0) && extension == "pcd") {
        std::string path = rel_path_so_far + dir_entry.path ().filename ().string ();
        relative_paths.push_back (path);
      }
    }
  }
}

int
main (int argc, char ** argv)
{
  if (argc < 3)
  {
    pcl::console::print_info ("Syntax is: %s input_directory <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
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
    pcl::console::print_info ("Note: The output's base filename must be specified without the .pcd extension\n");
    pcl::console::print_info ("      Four output files will be created with the following suffixes:\n");
    pcl::console::print_info ("        * '_points.pcd'\n");
    pcl::console::print_info ("        * '_keypoints.pcd'\n");
    pcl::console::print_info ("        * '_localdesc.pcd'\n");
    pcl::console::print_info ("        * '_globaldesc.pcd'\n");

    return (1);
  }

  ObjectRecognitionParameters params;
  std::ifstream params_stream;

  //Parse filter parameters
  std::string filter_parameters_file;
  if (pcl::console::parse_argument (argc, argv, "--filter", filter_parameters_file) < 0)
  {
    pcl::console::print_error ("Missing option --filter\n");
    return (1);
  }
  params_stream.open (filter_parameters_file.c_str ());
  if (params_stream.is_open())
  {
    params_stream >> params.min_depth;
    params_stream >> params.max_depth;
    params_stream >> params.downsample_leaf_size;
    params_stream >> params.outlier_rejection_radius;
    params_stream >> params.outlier_rejection_min_neighbors;
    params_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the filter parameters file (%s)\n", filter_parameters_file.c_str ());
    return (1);
  }

  // Parse segmentation parameters
  std::string segmentation_parameters_file;
  if (pcl::console::parse_argument (argc, argv, "--segment", segmentation_parameters_file) < 0)
  {
    pcl::console::print_error ("Missing option --segment\n");
    return (1);
  }
  params_stream.open (segmentation_parameters_file.c_str ());
  if (params_stream.is_open())
  {
    params_stream >> params.plane_inlier_distance_threshold;
    params_stream >> params.max_ransac_iterations;
    params_stream >> params.cluster_tolerance;
    params_stream >> params.min_cluster_size;
    params_stream >> params.max_cluster_size;
    params_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the segmentation parameters file (%s)\n",
                              segmentation_parameters_file.c_str ());
    return (1);
  }

  // Parse feature estimation parameters
  std::string feature_estimation_parameters_file;
  if (pcl::console::parse_argument (argc, argv, "--feature", feature_estimation_parameters_file) < 0)
  {
    pcl::console::print_error ("Missing option --feature\n");
    return (1);
  }
  params_stream.open (feature_estimation_parameters_file.c_str ());
  if (params_stream.is_open())
  {
    params_stream >> params.surface_normal_radius;
    params_stream >> params.keypoints_min_scale;
    params_stream >> params.keypoints_nr_octaves;
    params_stream >> params.keypoints_nr_scales_per_octave;
    params_stream >> params.keypoints_min_contrast;
    params_stream >> params.local_descriptor_radius;
    params_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the feature estimation parameters file (%s)\n",
                              feature_estimation_parameters_file.c_str ());
    return (1);
  }

  // Parse the registration parameters
  std::string registration_parameters_file;
  if (pcl::console::parse_argument (argc, argv, "--registration", registration_parameters_file) < 0)
  {
    pcl::console::print_error ("Missing option --registration\n");
    return (1);
  }
  params_stream.open (registration_parameters_file.c_str ());
  if (params_stream.is_open())
  {
    params_stream >> params.initial_alignment_min_sample_distance;
    params_stream >> params.initial_alignment_max_correspondence_distance;
    params_stream >> params.initial_alignment_nr_iterations;
    params_stream >> params.icp_max_correspondence_distance;
    params_stream >> params.icp_outlier_rejection_threshold;
    params_stream >> params.icp_transformation_epsilon;
    params_stream >> params.icp_max_iterations;
    params_stream.close ();
  }
  else
  {
    pcl::console::print_info ("Failed to open the registration parameters file (%s)\n",
                              registration_parameters_file.c_str ());
    return (1);
  }

  std::string directory (argv[1]);
  //Find all raw* files in input_directory
  bf::path dir_path = directory;
  std::vector < std::string > files;
  std::string start = "";
  getModelsInDirectory (dir_path, start, files);

  for(const auto& file : files) {
    // Load input file

    std::string filename = directory;
    filename.append("/");
    filename.append(file);
    PointCloudPtr input (new PointCloud);
    pcl::io::loadPCDFile (filename, *input);
    pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str(), input->size ());

    std::cout << file << std::endl;
    // Construct the object model
    ObjectRecognition obj_rec (params);
    ObjectModel model;
    obj_rec.constructObjectModel (input, model);

    //get directory name
    std::vector < std::string > strs;
    boost::split (strs, file, boost::is_any_of ("/\\"));

    std::string id = strs[0];
    std::string raw_file = strs[1];

    strs.clear();
    boost::split (strs, raw_file, boost::is_any_of ("_"));

    std::stringstream base_filestream;
    base_filestream << directory << "/" << id << "/" << id << strs[1].substr(0,1);
    // Save the model files
    std::string base_filename (base_filestream.str()), output_filename;

    output_filename = base_filename;
    output_filename.append ("_points.pcd");
    pcl::io::savePCDFile (output_filename, *(model.points));
    pcl::console::print_info ("Saved points as %s\n", output_filename.c_str ());

    output_filename = base_filename;
    output_filename.append ("_keypoints.pcd");
    pcl::io::savePCDFile (output_filename, *(model.keypoints));
    pcl::console::print_info ("Saved keypoints as %s\n", output_filename.c_str ());

    output_filename = base_filename;
    output_filename.append ("_localdesc.pcd");
    pcl::io::savePCDFile (output_filename, *(model.local_descriptors));
    pcl::console::print_info ("Saved local descriptors as %s\n", output_filename.c_str ());

    output_filename = base_filename;
    output_filename.append ("_globaldesc.pcd");
    pcl::io::savePCDFile (output_filename, *(model.global_descriptor));
    pcl::console::print_info ("Saved global descriptor as %s\n", output_filename.c_str ());
  }

  return (0);
}
