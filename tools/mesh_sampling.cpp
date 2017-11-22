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
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
  float r1 = static_cast<float> (uniform_deviate (rand ()));
  float r2 = static_cast<float> (uniform_deviate (rand ()));
  float r1sqr = std::sqrt (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p, bool calcNormal, Eigen::Vector3f& n)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  if (calcNormal)
  {
    // OBJ: Vertices are stored in a counter-clockwise order by default
    Eigen::Vector3f v1 = Eigen::Vector3f (A[0], A[1], A[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
    Eigen::Vector3f v2 = Eigen::Vector3f (B[0], B[1], B[2]) - Eigen::Vector3f (C[0], C[1], C[2]);
    n = v1.cross (v2);
    n.normalize ();
  }
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                       float (B[0]), float (B[1]), float (B[2]),
                       float (C[0]), float (C[1]), float (C[2]), p);
}

void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, bool calc_normal, pcl::PointCloud<pcl::PointNormal> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    Eigen::Vector3f n;
    randPSurface (polydata, &cumulativeAreas, totalArea, p, calc_normal, n);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
    if (calc_normal)
    {
      cloud_out.points[i].normal_x = n[0];
      cloud_out.points[i].normal_y = n[1];
      cloud_out.points[i].normal_z = n[2];
    }
  }
}

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

const int default_number_samples = 100000;
const float default_leaf_size = 0.01f;

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.{ply,obj} output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -n_samples X      = number of samples (default: ");
  print_value ("%d", default_number_samples);
  print_info (")\n");
  print_info (
              "                     -leaf_size X  = the XYZ leaf size for the VoxelGrid -- for data reduction (default: ");
  print_value ("%f", default_leaf_size);
  print_info (" m)\n");
  print_info ("                     -write_normals = flag to write normals to the output pcd\n");
  print_info (
              "                     -no_vis_result = flag to stop visualizing the generated pcd\n");
}

/* ---[ */
int
main (int argc, char **argv)
{
  print_info ("Convert a CAD model to a point cloud using uniform sampling. For more information, use: %s -h\n",
              argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse command line arguments
  int SAMPLE_POINTS_ = default_number_samples;
  parse_argument (argc, argv, "-n_samples", SAMPLE_POINTS_);
  float leaf_size = default_leaf_size;
  parse_argument (argc, argv, "-leaf_size", leaf_size);
  bool vis_result = ! find_switch (argc, argv, "-no_vis_result");
  const bool write_normals = find_switch (argc, argv, "-write_normals");

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

  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
  if (ply_file_indices.size () == 1)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY (argv[ply_file_indices[0]], mesh);
    pcl::io::mesh2vtk (mesh, polydata1);
  }
  else if (obj_file_indices.size () == 1)
  {
    vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
    readerQuery->SetFileName (argv[obj_file_indices[0]]);
    readerQuery->Update ();
    polydata1 = readerQuery->GetOutput ();
  }

  //make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
  triangleFilter->SetInput (polydata1);
#else
  triangleFilter->SetInputData (polydata1);
#endif
  triangleFilter->Update ();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
  triangleMapper->Update ();
  polydata1 = triangleMapper->GetInput ();

  bool INTER_VIS = false;

  if (INTER_VIS)
  {
    visualization::PCLVisualizer vis;
    vis.addModelFromPolyData (polydata1, "mesh1", 0);
    vis.setRepresentationToSurfaceForAllActors ();
    vis.spin ();
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointNormal>);
  uniform_sampling (polydata1, SAMPLE_POINTS_, write_normals, *cloud_1);

  if (INTER_VIS)
  {
    visualization::PCLVisualizer vis_sampled;
    vis_sampled.addPointCloud<pcl::PointNormal> (cloud_1);
    if (write_normals)
      vis_sampled.addPointCloudNormals<pcl::PointNormal> (cloud_1, 1, 0.02f, "cloud_normals");
    vis_sampled.spin ();
  }

  // Voxelgrid
  VoxelGrid<PointNormal> grid_;
  grid_.setInputCloud (cloud_1);
  grid_.setLeafSize (leaf_size, leaf_size, leaf_size);

  pcl::PointCloud<pcl::PointNormal>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointNormal>);
  grid_.filter (*voxel_cloud);

  if (vis_result)
  {
    visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
    vis3.addPointCloud<pcl::PointNormal> (voxel_cloud);
    if (write_normals)
      vis3.addPointCloudNormals<pcl::PointNormal> (voxel_cloud, 1, 0.02f, "cloud_normals");
    vis3.spin ();
  }

  if (!write_normals)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    // Strip uninitialized normals from cloud:
    pcl::copyPointCloud (*voxel_cloud, *cloud_xyz);
    savePCDFileASCII (argv[pcd_file_indices[0]], *cloud_xyz);
  }
  else
  {
    savePCDFileASCII (argv[pcd_file_indices[0]], *voxel_cloud);
  }
}
