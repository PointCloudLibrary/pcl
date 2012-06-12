/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float default_leaf_size = 0.01f;
float default_iso_level = 0.0f;
int   default_use_dot = 1;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.vtk <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -leaf X    = the voxel size (default: ");
  print_value ("%f", default_leaf_size); print_info (")\n");
  print_info ("                     -iso X     = the iso level (default: ");
  print_value ("%f", default_iso_level); print_info (")\n");
  print_info ("                     -dot X     = use the voxelization algorithm combined with a dot product (i.e. MarchingCubesGreedy vs. MarchingCubesGreedyDot) (default: ");
  print_value ("%d", default_use_dot); print_info (")\n");
}

bool
loadCloud (const std::string &filename, sensor_msgs::PointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const sensor_msgs::PointCloud2::ConstPtr &input, PolygonMesh &output,
         float leaf_size, float iso_level, int use_dot)
{
  PointCloud<PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<PointNormal> ());
  fromROSMsg (*input, *xyz_cloud);

//  boost::shared_ptr<MarchingCubesHoppe<PointNormal> > marching_cubes (new MarchingCubesHoppe<PointNormal> ());
  boost::shared_ptr<MarchingCubesRBF<PointNormal> > marching_cubes (new MarchingCubesRBF<PointNormal> ());
//  if (use_dot)
//    marching_cubes.reset (new MarchingCubesGreedyDot<PointNormal> ());
//  else
//    marching_cubes.reset (new MarchingCubesGreedy<PointNormal> ());

//  marching_cubes->setLeafSize (leaf_size);
  marching_cubes->setGridResolution (50, 50, 50);
  marching_cubes->setOffSurfaceDisplacement (0.1f);
  iso_level = 0;
  marching_cubes->setIsoLevel (iso_level);
  marching_cubes->setInputCloud (xyz_cloud);


  TicToc tt;
  tt.tic ();

  print_highlight ("Computing ");
  marching_cubes->reconstruct (output);

  PointCloud<PointXYZI> dist_cloud;
  for (int x = 0; x < marching_cubes->res_x_; ++x)
      for (int y = 0; y < marching_cubes->res_y_; ++y)
        for (int z = 0; z < marching_cubes->res_z_; ++z)
        {
          PointXYZI p;
          p.x = marching_cubes->min_p_[0] + (marching_cubes->max_p_[0] - marching_cubes->min_p_[0]) / marching_cubes->res_x_ * x;
          p.y = marching_cubes->min_p_[1] + (marching_cubes->max_p_[1] - marching_cubes->min_p_[1]) / marching_cubes->res_y_ * y;
          p.z = marching_cubes->min_p_[2] + (marching_cubes->max_p_[2] - marching_cubes->min_p_[2]) / marching_cubes->res_z_ * z;
          p.intensity = marching_cubes->grid_[x * marching_cubes->res_y_*marching_cubes->res_z_ + y *marching_cubes->res_z_ + z];
          if (p.intensity < 0) p.intensity = -5;
          else p.intensity = 5;
          dist_cloud.push_back (p);
        }

  io::savePCDFile ("dist_cloud.pcd", dist_cloud);


  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

void
saveCloud (const std::string &filename, const PolygonMesh &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());
  saveVTKFile (filename, output);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Compute the surface reconstruction of a point cloud using the marching cubes algorithm (pcl::surface::MarchingCubesGreedy or pcl::surface::MarchingCubesGreedyDot. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> pcd_file_indices;
  pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need one input PCD file and one output VTK file to continue.\n");
    return (-1);
  }

  std::vector<int> vtk_file_indices = parse_file_extension_argument (argc, argv, ".vtk");
  if (vtk_file_indices.size () != 1)
  {
    print_error ("Need one output VTK file to continue.\n");
    return (-1);
  }


  // Command line parsing
  float leaf_size = default_leaf_size;
  parse_argument (argc, argv, "-leaf", leaf_size);
  print_info ("Using a leaf size of: "); print_value ("%f\n", leaf_size);

  float iso_level = default_iso_level;
  parse_argument (argc, argv, "-iso", iso_level);
  print_info ("Setting an iso level of: "); print_value ("%f\n", iso_level);

  int use_dot = default_use_dot;
  parse_argument (argc, argv, "-dot", use_dot);
  if (use_dot)
    print_info ("Selected algorithm: MarchingCubesGreedyDot\n");
  else
    print_info ("Selected algorithm: MarchingCubesGreedy\n");

  // Load the first file
  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2);
  if (!loadCloud (argv[pcd_file_indices[0]], *cloud))
    return (-1);

  // Apply the marching cubes algorithm
  PolygonMesh output;
  compute (cloud, output, leaf_size, iso_level, use_dot);

  // Save into the second file
  saveCloud (argv[vtk_file_indices[0]], output);
}

