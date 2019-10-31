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
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_tesselated_sphere_level = 2;
int default_resolution = 100;
float default_leaf_size = 0.01f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.{ply,obj} output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -level X      = tessellated sphere level (default: ");
  print_value ("%d", default_tesselated_sphere_level);
  print_info (")\n");
  print_info ("                     -resolution X = the sphere resolution in angle increments (default: ");
  print_value ("%d", default_resolution);
  print_info (" deg)\n");
  print_info (
              "                     -leaf_size X  = the XYZ leaf size for the VoxelGrid -- for data reduction (default: ");
  print_value ("%f", default_leaf_size);
  print_info (" m)\n");
  print_info (
              "                     -no_vis_result = flag to stop visualizing the generated pcd\n");
}

/* ---[ */
int
main (int argc, char **argv)
{
  print_info ("Convert a CAD model to a point cloud using ray tracing operations. For more information, use: %s -h\n",
              argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
  int tesselated_sphere_level = default_tesselated_sphere_level;
  parse_argument (argc, argv, "-level", tesselated_sphere_level);
  int resolution = default_resolution;
  parse_argument (argc, argv, "-resolution", resolution);
  float leaf_size = default_leaf_size;
  parse_argument (argc, argv, "-leaf_size", leaf_size);
  bool vis_result = ! find_switch (argc, argv, "-no_vis_result");

  // Parse the command line arguments for .ply and PCD files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_file_indices.size () != 1)
  {
    print_error ("Need a single output PCD file to continue.\n");
    return (-1);
  }
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  if (ply_file_indices.size () != 1 && obj_file_indices.size () != 1)
  {
    print_error ("Need a single input PLY/OBJ file to continue.\n");
    return (-1);
  }

  vtkSmartPointer<vtkPolyData> polydata1;
  if (ply_file_indices.size () == 1)
  {
    vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
    readerQuery->SetFileName (argv[ply_file_indices[0]]);
    readerQuery->Update ();
    polydata1 = readerQuery->GetOutput ();
  }
  else if (obj_file_indices.size () == 1)
  {
    vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
    readerQuery->SetFileName (argv[obj_file_indices[0]]);
    readerQuery->Update ();
    polydata1 = readerQuery->GetOutput ();
  }

  visualization::PCLVisualizer vis;
  vis.addModelFromPolyData (polydata1, "mesh1", 0);
  vis.setRepresentationToSurfaceForAllActors ();

  PointCloud<PointXYZ>::CloudVectorType views_xyz;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
  std::vector<float> enthropies;
  vis.renderViewTesselatedSphere (resolution, resolution, views_xyz, poses, enthropies, tesselated_sphere_level);

  //take views and fuse them together
  std::vector<PointCloud<PointXYZ>::Ptr> aligned_clouds;

  for (std::size_t i = 0; i < views_xyz.size (); i++)
  {
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
    Eigen::Matrix4f pose_inverse;
    pose_inverse = poses[i].inverse ();
    transformPointCloud (views_xyz[i], *cloud, pose_inverse);
    aligned_clouds.push_back (cloud);
  }

  // Fuse clouds
  PointCloud<PointXYZ>::Ptr big_boy (new PointCloud<PointXYZ> ());
  for (const auto &aligned_cloud : aligned_clouds)
    *big_boy += *aligned_cloud;

  if (vis_result)
  {
    visualization::PCLVisualizer vis2 ("visualize");
    vis2.addPointCloud (big_boy);
    vis2.spin ();
  }

  // Voxelgrid
  VoxelGrid<PointXYZ> grid_;
  grid_.setInputCloud (big_boy);
  grid_.setLeafSize (leaf_size, leaf_size, leaf_size);
  grid_.filter (*big_boy);

  if (vis_result)
  {
    visualization::PCLVisualizer vis3 ("visualize");
    vis3.addPointCloud (big_boy);
    vis3.spin ();
  }

  savePCDFileASCII (argv[pcd_file_indices[0]], *big_boy);
}
