 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2011, Willow Garage, Inc.
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
  *
  */
  
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/concave_hull.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef PointXYZ PointT;
typedef PointCloud<PointT> CloudT;

const static double default_alpha = 1e3f;

static void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s hull_cloud.pcd input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -alpha X = the hull alpha value (0+) (default: ");
  print_value ("%f", default_alpha);
  print_info (")\n");
}

static bool
loadCloud (std::string const& filename, CloudT &cloud)
{
  TicToc tt;
  print_highlight ("Loading ");
  print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
  print_info ("Available dimensions: ");
  print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

static void
saveCloud (std::string const& filename, CloudT const& cloud)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving ");
  print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, cloud);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", cloud.width * cloud.height);
  print_info (" points]\n");
}

static void
cropToHull (CloudT::Ptr output, CloudT::Ptr input, CloudT::Ptr hull_cloud, std::vector<pcl::Vertices> const& polygons, int dim)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Cropping ");

  CropHull<PointT> crop_filter;
  crop_filter.setInputCloud (input);
  crop_filter.setHullCloud (hull_cloud);
  crop_filter.setHullIndices (polygons);
  crop_filter.setDim (dim);
  
  crop_filter.filter (*output);

  print_info ("[done, ");
  print_value ("%g", tt.toc ());
  print_info (" ms : ");
  print_value ("%d", output->size());
  print_info (" points passed crop]\n");
}

static CloudT::Ptr
calculateHull (std::vector<pcl::Vertices>& polygons, int& dim, CloudT::Ptr cloud, double alpha)
{
  pcl::ConcaveHull<PointT> hull_calculator;
  CloudT::Ptr hull (new CloudT);
  hull_calculator.setInputCloud (cloud);
  hull_calculator.setAlpha (alpha);
  hull_calculator.reconstruct (*hull, polygons);
  
  dim = hull_calculator.getDimension ();
  return hull;
}

int
main (int argc, char** argv)
{
  print_info ("Filter a point cloud using the convex hull of another point "
              "cloud. For more information, use: %s -h\n", argv[0]);

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
    print_error ("Need at least three pcd files to continue.\n");
    return (-1);
  }

  // Command line parsing
  double alpha = default_alpha;
  parse_argument (argc, argv, "-alpha", alpha);

  CloudT::Ptr hull_cloud (new CloudT);
  CloudT::Ptr hull_points (new CloudT);
  CloudT::Ptr input_cloud (new CloudT);
  CloudT::Ptr output_cloud (new CloudT);
  std::vector<pcl::Vertices> hull_polygons;
  int dim = 0;  

  if (!loadCloud (argv[p_file_indices[0]], *hull_cloud))
    return (-1);
    
  if (!loadCloud (argv[p_file_indices[1]], *input_cloud))
    return (-1);
  
  hull_points = calculateHull (hull_polygons, dim, hull_cloud, alpha);

  cropToHull (output_cloud, input_cloud, hull_points, hull_polygons, dim);
  

  if (output_cloud->size ())
    saveCloud (argv[p_file_indices[2]], *output_cloud);
  else
    print_error ("No points passed crop.\n");

  return (0);
}


