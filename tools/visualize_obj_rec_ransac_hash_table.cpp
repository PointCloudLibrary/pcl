/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 */

/*
 * visualize_obj_rec_ransac_hash_table.cpp
 *
 *  Created on: Jun 20, 2012
 *      Author: papazov
 *
 *  Visualizes the hash table cell entries belonging to a model. Since the hash table is a 3D box structure, each cell
 *  is a box in 3D. Each cell is visualized by an ellipsoid placed in the box. The ellipsoid axes are proportional to the
 *  box sides. The more entries in a cell the larger the ellipsoid.
 */

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <vtkPolyDataReader.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <cstdio>
#include <vector>

using namespace std;
using namespace pcl;
using namespace io;
using namespace console;
using namespace recognition;
using namespace visualization;

inline double
my_sqr (double a){ return a*a;}

bool
get_points_and_normals (vtkPolyData* vtk_mesh, PointCloud<Eigen::Vector3d>& eigen_points, PointCloud<Eigen::Vector3d>& eigen_normals)
{
  printf("Copying the points and normals from the vtk input ... "); fflush(stdout);

  // Does the mesh have normals?
  vtkDataArray* vtk_normals = vtk_mesh->GetPointData()->GetNormals();
  if ( !vtk_normals )
  {
    print_error ("ERROR: The vtk mesh has no normals.\n");
    return false;
  }

  uint32_t num_pts = static_cast<uint32_t> (vtk_mesh->GetNumberOfPoints ());

  eigen_points.width = num_pts;
  eigen_points.height = 1; // This indicates that the point cloud is unorganized
  eigen_points.is_dense = false;
  eigen_points.points.resize(num_pts);

  eigen_normals.width = num_pts;
  eigen_normals.height = 1; // This indicates that the point cloud is unorganized
  eigen_normals.is_dense = false;
  eigen_normals.points.resize(num_pts);

  int i, num_points = static_cast<int> (vtk_mesh->GetNumberOfPoints ());
  double p[3], n[3];

  for ( i = 0 ; i < num_points ; ++i )
  {
    // Save the point
    vtk_mesh->GetPoint(i, p);
    eigen_points[i][0] = p[0];
    eigen_points[i][1] = p[1];
    eigen_points[i][2] = p[2];

    // Save the normal
    vtk_normals->GetTuple(i, n);
    eigen_normals[i][0] = n[0];
    eigen_normals[i][1] = n[1];
    eigen_normals[i][2] = n[2];
  }

  printf("OK\n");

  return true;
}

int
main (int argc, char** argv)
{
  // Make sure that we have the right number of arguments
  if (argc != 2)
  {
    print_info ("\nVisualizes the hash table after adding the provided mesh to it.\n"
        "usage:\n"
        "./visualize_obj_rec_ransac_hash_table mesh.vtk\n");
    return (-1);
  }

  // Parse the command line arguments for .vtk files
  if ( parse_file_extension_argument (argc, argv, ".vtk").size () != 1 )
  {
    print_error ("We need a .vtk object.\n");
    return (-1);
  }

  // Load the model
  vtkPolyDataReader *vtk_reader = vtkPolyDataReader::New ();
  vtk_reader->SetFileName (argv[1]);
  vtk_reader->Update ();
  vtkPolyData *vtk_mesh = vtk_reader->GetOutput ();
  // Get the bounds of the mesh
  double mb[6];
  vtk_mesh->ComputeBounds();
  vtk_mesh->GetBounds(mb);

  // Create a point cloud with normals
  ModelLibrary::PointCloudInPtr eigen_points(new ModelLibrary::PointCloudIn());
  ModelLibrary::PointCloudNPtr eigen_normals(new ModelLibrary::PointCloudN());
  if ( !get_points_and_normals (vtk_mesh, *eigen_points.get(), *eigen_normals.get()) )
  {
    vtk_reader->Delete ();
    return (-1);
  }

  // Compute the bounding box diagonal
  double diag = sqrt(my_sqr(mb[1]-mb[0]) + my_sqr(mb[3]-mb[2]) + my_sqr(mb[5]-mb[4]));

  // Create the recognition object (we need it only for its hash table)
  ObjRecRANSAC objrec(diag/8.0, diag/20.0);
  printf("Adding a model to the library ... "); fflush(stdout);
  objrec.addModel(eigen_points, eigen_normals, string("test_model"));
  printf("OK\n");

//  PCLVisualizer vis;
//  vis.setBackgroundColor (0.1, 0.1, 0.1);
//  vis.spin();

  // To be continued ...

  // Cleanup
  vtk_reader->Delete ();

  return 0;
}
