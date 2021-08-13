/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

std::string default_correspondence_type = "index";

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s source.pcd target.pcd output_intensity.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -correspondence X = the way of selecting the corresponding pair in the target cloud for the current point in the source cloud\n");
  print_info ("                                         options are: index = points with identical indices are paired together. Note: both clouds need to have the same number of points\n");
  print_info ("                                                      nn = source point is paired with its nearest neighbor in the target cloud\n");
  print_info ("                                                      nnplane = source point is paired with its projection on the plane determined by the nearest neighbor in the target cloud. Note: target cloud needs to contain normals\n");
  print_info ("                                         (default: ");
  print_value ("%s", default_correspondence_type.c_str ()); print_info (")\n");
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &cloud_source, const pcl::PCLPointCloud2::ConstPtr &cloud_target,
         pcl::PCLPointCloud2 &output, const std::string &correspondence_type)
{
  PointCloud<PointXYZ>::Ptr xyz_source (new PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*cloud_source, *xyz_source);
  PointCloud<PointXYZ>::Ptr xyz_target (new PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*cloud_target, *xyz_target);

  PointCloud<PointXYZI>::Ptr output_xyzi (new PointCloud<PointXYZI> ());
  output_xyzi->points.resize (xyz_source->size ());
  output_xyzi->height = cloud_source->height;
  output_xyzi->width = cloud_source->width;

  float rmse = 0.0f;

  if (correspondence_type == "index")
  {
//    print_highlight (stderr, "Computing using the equal indices correspondence heuristic.\n");

    if (xyz_source->size () != xyz_target->size ())
    {
      print_error ("Source and target clouds do not have the same number of points.\n");
      return;
    }

    for (std::size_t point_i = 0; point_i < xyz_source->size (); ++point_i)
    {
      if (!std::isfinite ((*xyz_source)[point_i].x) || !std::isfinite ((*xyz_source)[point_i].y) || !std::isfinite ((*xyz_source)[point_i].z))
        continue;
      if (!std::isfinite ((*xyz_target)[point_i].x) || !std::isfinite ((*xyz_target)[point_i].y) || !std::isfinite ((*xyz_target)[point_i].z))
        continue;


      float dist = squaredEuclideanDistance ((*xyz_source)[point_i], (*xyz_target)[point_i]);
      rmse += dist;

      (*output_xyzi)[point_i].x = (*xyz_source)[point_i].x;
      (*output_xyzi)[point_i].y = (*xyz_source)[point_i].y;
      (*output_xyzi)[point_i].z = (*xyz_source)[point_i].z;
      (*output_xyzi)[point_i].intensity = dist;
    }
    rmse = std::sqrt (rmse / static_cast<float> (xyz_source->size ()));
  }
  else if (correspondence_type == "nn")
  {
//    print_highlight (stderr, "Computing using the nearest neighbor correspondence heuristic.\n");

    KdTreeFLANN<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ> ());
    tree->setInputCloud (xyz_target);

    for (std::size_t point_i = 0; point_i < xyz_source->size (); ++ point_i)
    {
      if (!std::isfinite ((*xyz_source)[point_i].x) || !std::isfinite ((*xyz_source)[point_i].y) || !std::isfinite ((*xyz_source)[point_i].z))
        continue;

      pcl::Indices nn_indices (1);
      std::vector<float> nn_distances (1);
      if (!tree->nearestKSearch ((*xyz_source)[point_i], 1, nn_indices, nn_distances))
        continue;
      std::size_t point_nn_i = nn_indices.front();

      float dist = squaredEuclideanDistance ((*xyz_source)[point_i], (*xyz_target)[point_nn_i]);
      rmse += dist;

      (*output_xyzi)[point_i].x = (*xyz_source)[point_i].x;
      (*output_xyzi)[point_i].y = (*xyz_source)[point_i].y;
      (*output_xyzi)[point_i].z = (*xyz_source)[point_i].z;
      (*output_xyzi)[point_i].intensity = dist;
    }
    rmse = std::sqrt (rmse / static_cast<float> (xyz_source->size ()));

  }
  else if (correspondence_type == "nnplane")
  {
//    print_highlight (stderr, "Computing using the nearest neighbor plane projection correspondence heuristic.\n");

    PointCloud<Normal>::Ptr normals_target (new PointCloud<Normal> ());
    fromPCLPointCloud2 (*cloud_target, *normals_target);

    KdTreeFLANN<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ> ());
    tree->setInputCloud (xyz_target);

    for (std::size_t point_i = 0; point_i < xyz_source->size (); ++ point_i)
    {
      if (!std::isfinite ((*xyz_source)[point_i].x) || !std::isfinite ((*xyz_source)[point_i].y) || !std::isfinite ((*xyz_source)[point_i].z))
        continue;

      pcl::Indices nn_indices (1);
      std::vector<float> nn_distances (1);
      if (!tree->nearestKSearch ((*xyz_source)[point_i], 1, nn_indices, nn_distances))
        continue;
      std::size_t point_nn_i = nn_indices.front();

      Eigen::Vector3f normal_target = (*normals_target)[point_nn_i].getNormalVector3fMap (),
          point_source = (*xyz_source)[point_i].getVector3fMap (),
          point_target = (*xyz_target)[point_nn_i].getVector3fMap ();

      float dist = normal_target.dot (point_source - point_target);
      rmse += dist * dist;

      (*output_xyzi)[point_i].x = (*xyz_source)[point_i].x;
      (*output_xyzi)[point_i].y = (*xyz_source)[point_i].y;
      (*output_xyzi)[point_i].z = (*xyz_source)[point_i].z;
      (*output_xyzi)[point_i].intensity = dist * dist;
    }
    rmse = std::sqrt (rmse / static_cast<float> (xyz_source->size ()));
  }
  else
  {
//    print_error ("Unrecognized correspondence type. Check legal arguments by using the -h option\n");
    return;
  }

  toPCLPointCloud2 (*output_xyzi, output);

  print_highlight ("RMSE Error: %f\n", rmse);
}


/* ---[ */
int
main (int argc, char** argv)
{
//  print_info ("Compute the differences between two point clouds and visualizing them as an output intensity cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 4)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 3)
  {
    print_error ("Need two input PCD files and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  std::string correspondence_type = default_correspondence_type;
  parse_argument (argc, argv, "-correspondence", correspondence_type);

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud_source (new pcl::PCLPointCloud2 ());
  if (loadPCDFile (argv[p_file_indices[0]], *cloud_source) != 0)
    return (-1);
  // Load the second file
  pcl::PCLPointCloud2::Ptr cloud_target (new pcl::PCLPointCloud2 ());
  if (loadPCDFile (argv[p_file_indices[1]], *cloud_target) != 0)
    return (-1);

  pcl::PCLPointCloud2 output;
  // Perform the feature estimation
  compute (cloud_source, cloud_target, output, correspondence_type);

  // Output the third file
  savePCDFile (argv[p_file_indices[2]], output);
}
