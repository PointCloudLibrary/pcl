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

/*
 * visualize_obj_rec_ransac_hash_table.cpp
 *
 *  Created on: Jun 20, 2012
 *      Author: papazov
 *
 *  Visualizes the hash table cell entries belonging to a model. Since the hash table is a 3D cube structure, each cell
 *  is a cube in 3D and is visualized by a sphere placed in the cube. The more entries in a cell the larger the sphere
 *  radius. However, the largest radius is half the cube edge length, i.e., the largest sphere touches the cube walls.
 */

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <vtkPolyDataReader.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkGlyph3D.h>
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

//===========================================================================================================================================

bool
get_points_and_normals (vtkPolyData* vtk_mesh, PointCloud<Eigen::Vector3d>& eigen_points, PointCloud<Eigen::Vector3d>& eigen_normals)
{
  printf ("Copying the points and normals from the vtk input ... "); fflush (stdout);

  if ( !vtk_mesh->GetNumberOfPoints () )
  {
    print_error ("ERROR: The vtk mesh has no points.\n");
    return false;
  }

  // Does the mesh have normals?
  vtkDataArray* vtk_normals = vtk_mesh->GetPointData ()->GetNormals ();
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

//===========================================================================================================================================

void
visualize (const ModelLibrary::HashTable* hash_table)
{
  const ModelLibrary::HashTableCell* cells = hash_table->getVoxels ();
  size_t max_num_entries = 0;
  int id3[3], num_cells = hash_table->getNumberOfVoxels ();
  double cell_center[3];

  vtkPoints* sphere_centers = vtkPoints::New (VTK_DOUBLE);
  vtkDoubleArray* scale = vtkDoubleArray::New ();
  scale->SetNumberOfComponents(1);

  // Get the positions of the spheres (one sphere per full cell)
  for (int i = 0 ; i < num_cells ; ++i, ++cells)
  {
    // Does the cell have any entries?
    if (cells->size ())
    {
      // Insert the center of the current voxel in the point set
      hash_table->compute3dId (i, id3);
      hash_table->computeVoxelCenter (id3, cell_center);
      sphere_centers->InsertNextPoint (cell_center);

      // Save the number of entries
      scale->InsertNextValue (static_cast<double> (cells->size ()));

      // Get the max
      if (cells->size () > max_num_entries)
        max_num_entries = cells->size ();
    }
  }

  PCLVisualizer vis;
  vis.setBackgroundColor (0.1, 0.1, 0.1);

  // Is there something to visualize?
  if (max_num_entries)
  {
    // Compute the factor which maps all the scale values in (0, 1]
    double factor = 1.0/static_cast<double> (max_num_entries);
    // Set the true scale
    for (vtkIdType i = 0 ; i < scale->GetNumberOfTuples () ; ++i)
      scale->SetValue(i, factor*scale->GetValue (i));

    // Input for the glyph object: the centers + scale
    vtkPolyData *positions = vtkPolyData::New ();
    positions->SetPoints (sphere_centers);
    positions->GetPointData ()->SetScalars (scale);
    // The spheres
    vtkSphereSource* sphere_src = vtkSphereSource::New ();
    sphere_src->SetPhiResolution(8);
    sphere_src->SetThetaResolution(8);
    sphere_src->SetRadius(0.5*hash_table->getVoxelSpacing ()[0]);

    // Now that we have the points and the corresponding scalars, build the glyph object
    vtkGlyph3D *glyph = vtkGlyph3D::New ();
    glyph->SetScaleModeToScaleByScalar ();
    glyph->SetColorModeToColorByScalar ();
    glyph->SetInput (positions);
    glyph->SetSource (sphere_src->GetOutput ());
    glyph->Update ();

    vtkSmartPointer<vtkPolyData> glyph_output (glyph->GetOutput ());
    vis.addModelFromPolyData(glyph_output);

    // Cleanup
    glyph->Delete ();
    positions->Delete ();
    sphere_src->Delete ();
  }

  vis.spin();

  // Cleanup
  sphere_centers->Delete();
  scale->Delete();
}

//===========================================================================================================================================

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
  vtk_mesh->ComputeBounds ();
  vtk_mesh->GetBounds (mb);

  // Create a point cloud with normals
  ModelLibrary::PointCloudInPtr eigen_points (new ModelLibrary::PointCloudIn ());
  ModelLibrary::PointCloudNPtr eigen_normals (new ModelLibrary::PointCloudN ());
  if ( !get_points_and_normals (vtk_mesh, *eigen_points.get(), *eigen_normals.get()) )
  {
    vtk_reader->Delete ();
    return (-1);
  }

  // Compute the bounding box diagonal
  double diag = sqrt (my_sqr (mb[1]-mb[0]) + my_sqr (mb[3]-mb[2]) + my_sqr (mb[5]-mb[4]));

  // Create the recognition object (we need it only for its hash table)
  ObjRecRANSAC objrec (diag/8.0, diag/20.0);
  printf("Adding the model to the library ... "); fflush (stdout);
  objrec.addModel (eigen_points, eigen_normals, string ("test_model"));
  printf("OK\n");

  // Start visualization (and the main VTK loop)
  visualize (objrec.getHashTable ());

  // Cleanup
  vtk_reader->Delete ();

  return 0;
}
