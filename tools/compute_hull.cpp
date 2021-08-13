/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_alpha = 0.15f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk [optional_arguments]\n", argv[0]);
  print_info ("  where the optional arguments are:\n");
  print_info ("                     -alpha X = the alpha value for the ConcaveHull (Alpha Shapes) algorithm. If alpha is not specified, the tool will run the ConvexHull method (default: ");
  print_value ("%f", default_alpha); print_info (")\n");
}


void
compute (PointCloud<PointXYZ>::ConstPtr cloud_in,
         bool convex_concave_hull,
         float alpha,
         PolygonMesh &mesh_out)
{
  if (!convex_concave_hull)
  {
    print_info ("Computing the convex hull of a cloud with %lu points.\n", cloud_in->size ());
    ConvexHull<PointXYZ> convex_hull;
    convex_hull.setInputCloud (cloud_in);
    convex_hull.reconstruct (mesh_out);
  }
  else
  {
    print_info ("Computing the concave hull (alpha shapes) with alpha %f of a cloud with %lu points.\n", alpha, cloud_in->size ());
    ConcaveHull<PointXYZ> concave_hull;
    concave_hull.setInputCloud (cloud_in);
    concave_hull.setAlpha (alpha);
    concave_hull.reconstruct (mesh_out);
  }
}


/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute the convex or concave hull of a point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  bool convex_concave_hull = false;
  float alpha = default_alpha;

  if (parse_argument (argc, argv, "-alpha", alpha) != -1)
    convex_concave_hull = true;

  std::vector<int> pcd_file_indices;
  pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need one input PCD file to continue.\n");
    return (-1);
  }

  std::vector<int> vtk_file_indices;
  vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  if (vtk_file_indices.size () != 1)
  {
    print_error ("Need one output VTK file to continue.\n");
    return (-1);
  }


  // Load in the point cloud
  PointCloud<PointXYZ>::Ptr cloud_in (new PointCloud<PointXYZ> ());
  if (loadPCDFile (argv[pcd_file_indices[0]], *cloud_in) != 0)
  {
    print_error ("Could not load input file %s\n", argv[pcd_file_indices[0]]);
    return (-1);
  }

  // Compute the hull
  PolygonMesh mesh_out;
  compute (cloud_in, convex_concave_hull, alpha, mesh_out);

  // Save the mesh
  io::saveVTKFile (argv[vtk_file_indices[0]], mesh_out);

  return (0);
}

