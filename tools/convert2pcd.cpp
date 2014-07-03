/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Author: Markus Schoeler */

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/simulation/shape_generator_shapes.h>
#include <boost/filesystem.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void
printHelp (int,
           char **argv)
{
  PCL_INFO ("Syntax is: %s input.(ply,obj,stl) [options]\n", argv[0]);
  PCL_INFO ("where options are: \n");
  PCL_INFO ("     -d x : sets the point density of the sampling to x per square unit (default 1).\n");
  PCL_INFO ("     -n x : sets the approximate number of points sampled (cannot be used together with -d).\n");
}

int
main (int argc,
      char** argv)
{
  PCL_INFO ("Convert a PLY, OBJ or STL file to PCD file with normals (calculated) by sampling on the faces.\n");
  
  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }
  float point_density = 1;
  bool n_param_defined = pcl::console::find_switch (argc, argv, "-n");
  bool d_param_defined = pcl::console::find_switch (argc, argv, "-d");

  if (n_param_defined && d_param_defined)
  {
    PCL_ERROR ("You cannot define both -n and -d parameters\n");
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd, .stl and .obj files
  std::vector<int> pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  std::vector<int> stl_file_indices = parse_file_extension_argument (argc, argv, ".stl");
  std::vector<int> ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");

  bool pcd_file_defined = (pcd_file_indices.size () == 1);
  bool obj_file_defined = (obj_file_indices.size () == 1);
  bool stl_file_defined = (stl_file_indices.size () == 1);
  bool ply_file_defined = (ply_file_indices.size () == 1);

  std::string input_filename;

  if (obj_file_defined + stl_file_defined + ply_file_defined != 1)
  {
    PCL_ERROR ("Need exactly one input PLY, OBJ or STL file.\n");
    printHelp (argc, argv);
    return (-2);
  }

  TicToc tt;

  // Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  if (obj_file_defined)
  {
    input_filename = argv[obj_file_indices[0]];
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New ();
    reader->SetFileName (input_filename.c_str ());
    reader->Update ();
    polydata = reader->GetOutput ();
  }
  else if (stl_file_defined)
  {
    input_filename = argv[stl_file_indices[0]];
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New ();
    reader->SetFileName (input_filename.c_str ());
    reader->Update ();
    polydata = reader->GetOutput ();
  }
  else if (ply_file_defined)
  {
    input_filename = argv[stl_file_indices[0]];
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
    reader->SetFileName (input_filename.c_str ());
    reader->Update ();
    polydata = reader->GetOutput ();
  }

  PCL_INFO ("Loading %s\n", input_filename.c_str ());
  boost::filesystem::path pcd_filename;
  if (!pcd_file_defined)
  {
    pcd_filename = input_filename;
    pcd_filename.replace_extension (".pcd");
  }
  else
    pcd_filename = argv[pcd_file_indices[0]];

  vtkSmartPointer<vtkPoints> vertices = polydata->GetPoints ();
  vtkSmartPointer<vtkCellArray> polygons = polydata->GetPolys ();

  PCL_INFO ("Found %d vertices and %d polygons\n", vertices->GetNumberOfPoints (), polygons->GetNumberOfCells ());
  unsigned int poly_counter = 0;
  // fill the vertices
  std::vector<Eigen::Vector3f> vertices_eigen;
  vertices_eigen.resize (vertices->GetNumberOfPoints ());
  for (vtkIdType i = 0; i < vertices->GetNumberOfPoints (); ++i)
  {
    double point[3];
    vertices->GetPoint (i, point);
    vertices_eigen[i] = Eigen::Vector3f (point[0], point[1], point[2]);
  }

  // fill the polygons
  std::vector<std::vector<unsigned int> > polygon_vector;
  polygon_vector.resize (polygons->GetNumberOfCells ());
  vtkIdType *indices;
  vtkIdType number_of_points;
  for (polygons->InitTraversal (); polygons->GetNextCell (number_of_points, indices); ++poly_counter)
  {
    polygon_vector[poly_counter].resize (number_of_points);
    for (vtkIdType cp = 0; cp < number_of_points; ++cp)
    {
      polygon_vector[poly_counter][cp] = indices[cp];
    }
  }

  // create shape,
  pcl::simulation::ConvexPolygon polygon_shape (vertices_eigen, polygon_vector);

  // get or calculate point density
  if (d_param_defined)
    pcl::console::parse_argument (argc, argv, "-d", point_density);  
  if (n_param_defined)
  {
    float num_points;
    pcl::console::parse_argument (argc, argv, "-n", num_points);
    point_density = num_points / polygon_shape.getPolygonArea ();
  }
  
  // sample and save pointcloud  
  pcl::simulation::GeometricShapeBase::PointCloudT::Ptr shape_cloud = polygon_shape.generate (point_density);
  
  if (shape_cloud->size () == 0)
  {
    PCL_ERROR ("Not points have been sampled, you should try increasing the density using the -d paramter or set the number of points using the -n parameter\n");
    return (1);
  }  
  PCL_INFO ("Saving PCD file %s with %d points (Density: %f pts per sq-unit)\n", pcd_filename.c_str (), shape_cloud->size (), point_density);
  pcl::PointCloud<pcl::PointNormal> cloud_with_normals;
  pcl::copyPointCloud (*shape_cloud, cloud_with_normals);

  pcl::io::savePCDFile (pcd_filename.generic_string (), cloud_with_normals);
  return (0);
}

