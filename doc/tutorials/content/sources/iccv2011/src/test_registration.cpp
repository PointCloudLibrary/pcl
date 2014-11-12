/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include "registration.h"

#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "load_clouds.h"

int 
main (int argc, char ** argv)
{
  if (argc < 3) 
  {
    pcl::console::print_info ("Syntax is: %s source target <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -i min_sample_dist,max_dist,nr_iters ................ Compute initial alignment\n");
    pcl::console::print_info ("    -r max_dist,rejection_thresh,tform_eps,max_iters ............. Refine alignment\n");
    pcl::console::print_info ("    -s output.pcd ........................... Save the registered and merged clouds\n");
    pcl::console::print_info ("Note: The inputs (source and target) must be specified without the .pcd extension\n");

    return (1);
  }

  // Load the points
  PointCloudPtr src_points = loadPoints (argv[1]);
  PointCloudPtr tgt_points = loadPoints (argv[2]);
  Eigen::Matrix4f tform = Eigen::Matrix4f::Identity ();

  // Compute the intial alignment
  double min_sample_dist, max_correspondence_dist, nr_iters;
  bool compute_intial_alignment = 
    pcl::console::parse_3x_arguments (argc, argv, "-i", min_sample_dist, max_correspondence_dist, nr_iters) > 0;
  if (compute_intial_alignment)
  {
    // Load the keypoints and local descriptors
    PointCloudPtr src_keypoints = loadKeypoints (argv[1]);
    LocalDescriptorsPtr src_descriptors = loadLocalDescriptors (argv[1]);
    PointCloudPtr tgt_keypoints = loadKeypoints (argv[2]);
    LocalDescriptorsPtr tgt_descriptors = loadLocalDescriptors (argv[2]);

    // Find the transform that roughly aligns the points
    tform = computeInitialAlignment (src_keypoints, src_descriptors, tgt_keypoints, tgt_descriptors,
                                     min_sample_dist, max_correspondence_dist, nr_iters);
    
    pcl::console::print_info ("Computed initial alignment\n");
  }

  // Refine the initial alignment
  std::string params_string;
  bool refine_alignment = pcl::console::parse_argument (argc, argv, "-r", params_string) > 0;
  if (refine_alignment)
  {
    std::vector<std::string> tokens;
    boost::split (tokens, params_string, boost::is_any_of (","), boost::token_compress_on);
    assert (tokens.size () == 4);    
    float max_correspondence_distance = atof(tokens[0].c_str ());
    float outlier_rejection_threshold = atof(tokens[1].c_str ());
    float transformation_epsilon = atoi(tokens[2].c_str ());
    int max_iterations = atoi(tokens[3].c_str ());

    tform = refineAlignment (src_points, tgt_points, tform, max_correspondence_distance,  
                             outlier_rejection_threshold, transformation_epsilon, max_iterations);

    pcl::console::print_info ("Refined alignment\n");
  }  

  // Transform the source point to align them with the target points
  pcl::transformPointCloud (*src_points, *src_points, tform);

  // Save output
  std::string filename;
  bool save_output = pcl::console::parse_argument (argc, argv, "-s", filename) > 0;
  if (save_output)
  {
    // Merge the two clouds
    (*src_points) += (*tgt_points);

    // Save the result
    pcl::io::savePCDFile (filename, *src_points);

    pcl::console::print_info ("Saved registered clouds as %s\n", filename.c_str ());    
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit\n");
    pcl::visualization::PCLVisualizer vis;

    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (src_points, 255, 0, 0);
    vis.addPointCloud (src_points, red, "src_points");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow (tgt_points, 255, 255, 0);
    vis.addPointCloud (tgt_points, yellow, "tgt_points");
    
    vis.resetCamera ();
    vis.spin ();
  }

  return (0);
}
