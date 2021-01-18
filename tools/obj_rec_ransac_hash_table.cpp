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
 * obj_rec_ransac_hash_table.cpp
 *
 *  Created on: Jun 20, 2012
 *      Author: papazov
 *
 *  Visualizes the hash table cell entries belonging to a model. Since the hash table is a 3D cube structure, each cell
 *  is a cube in 3D. It is visualized by a cube which is scaled according to the number of entries in the cell -> the more
 *  entries the bigger the cube.
 */

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <vtkPolyDataReader.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkGlyph3D.h>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;
using namespace pcl;
using namespace io;
using namespace console;
using namespace recognition;
using namespace visualization;

inline double
my_sqr (double a){ return a*a;}

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& points_in, PointCloud<Normal>& normals_in, double bounds[6]);
void visualize (const ModelLibrary::HashTable& hash_table);

//===========================================================================================================================================

int
main (int argc, char** argv)
{
  // Make sure that we have the right number of arguments
  if (argc != 2)
  {
    print_info ("\nVisualizes the hash table after adding the provided mesh to it.\n"
        "usage:\n"
        "./obj_rec_ransac_hash_table <mesh.vtk>\n");
    return (-1);
  }

  ObjRecRANSAC::PointCloudIn points_in;
  ObjRecRANSAC::PointCloudN normals_in;
  double b[6];

  if ( !vtk_to_pointcloud (argv[1], points_in, normals_in, b) )
    return (-1);

  // Compute the bounding box diagonal
  float diag = static_cast<float> (sqrt (my_sqr (b[1]-b[0]) + my_sqr (b[3]-b[2]) + my_sqr (b[5]-b[4])));

  // Create the recognition object (we need it only for its hash table)
  ObjRecRANSAC objrec (diag/8.0f, diag/60.0f);
  objrec.addModel (points_in, normals_in, "test_model");

  // Start visualization (and the main VTK loop)
  visualize (objrec.getHashTable ());

  return (0);
}

//===========================================================================================================================================

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& points_in, PointCloud<Normal>& normals_in, double b[6])
{
  std::size_t len = strlen (file_name);
  if ( file_name[len-3] != 'v' || file_name[len-2] != 't' || file_name[len-1] != 'k' )
  {
    fprintf (stderr, "ERROR: we need a .vtk object!\n");
    return false;
  }

  // Load the model
  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
  reader->SetFileName (file_name);
  reader->Update ();

  // Get the points
  vtkPolyData *vtk_poly = reader->GetOutput ();
  vtkPoints *vtk_points = vtk_poly->GetPoints ();
  vtkIdType num_points = vtk_points->GetNumberOfPoints ();
  double p[3];

  vtk_poly->ComputeBounds ();
  vtk_poly->GetBounds (b);
  points_in.resize (num_points);

  // Copy the points
  for ( vtkIdType i = 0 ; i < num_points ; ++i )
  {
    vtk_points->GetPoint (i, p);
    points_in[i].x = static_cast<float> (p[0]);
    points_in[i].y = static_cast<float> (p[1]);
    points_in[i].z = static_cast<float> (p[2]);
  }

  // Check if we have normals
  vtkDataArray *vtk_normals = vtk_poly->GetPointData ()->GetNormals ();
  if ( vtk_normals )
  {
    normals_in.resize (num_points);
    // Copy the normals
    for ( vtkIdType i = 0 ; i < num_points ; ++i )
    {
      vtk_normals->GetTuple (i, p);
      normals_in[i].normal_x = static_cast<float> (p[0]);
      normals_in[i].normal_y = static_cast<float> (p[1]);
      normals_in[i].normal_z = static_cast<float> (p[2]);
    }
  }

  return true;
}

//===========================================================================================================================================

void
visualize (const ModelLibrary::HashTable& hash_table)
{
  PCLVisualizer vis;
  vis.setBackgroundColor (0.1, 0.1, 0.1);

  const ModelLibrary::HashTableCell* cells = hash_table.getVoxels ();
  std::size_t max_num_entries = 0;
  int id3[3], num_cells = hash_table.getNumberOfVoxels ();
  float half_side, b[6], cell_center[3], spacing = hash_table.getVoxelSpacing ()[0];
  char cube_id[128];

  // Just get the maximal number of entries in the cells
  for ( int i = 0 ; i < num_cells ; ++i, ++cells )
  {
    if (!cells->empty ()) // That's the number of models in the cell (it's maximum one, since we loaded only one model)
    {
      std::size_t num_entries = (*cells->begin ()).second.size(); // That's the number of entries in the current cell for the model we loaded
      // Get the max number of entries
      if ( num_entries > max_num_entries )
        max_num_entries = num_entries;
    }
  }

  // Now, that we have the max. number of entries, we can compute the
  // right scale factor for the spheres
  float s = (0.5f*spacing)/static_cast<float> (max_num_entries);

  std::cout << "s = " << s << ", max_num_entries = " << max_num_entries << std::endl;

  // Now, render a sphere with the right radius at the right place
  cells = hash_table.getVoxels ();
  for ( int i = 0; i < num_cells ; ++i, ++cells )
  {
    // Does the cell have any entries?
    if (!cells->empty ())
    {
      hash_table.compute3dId (i, id3);
      hash_table.computeVoxelCenter (id3, cell_center);

      // That's half of the cube's side length
      half_side = s*static_cast<float> ((*cells->begin ()).second.size ());

      // Adjust the bounds of the cube
      b[0] = cell_center[0] - half_side; b[1] = cell_center[0] + half_side;
      b[2] = cell_center[1] - half_side; b[3] = cell_center[1] + half_side;
      b[4] = cell_center[2] - half_side; b[5] = cell_center[2] + half_side;

      // Set the id
      sprintf (cube_id, "cube %i", i);

      // Add to the visualizer
      vis.addCube (b[0], b[1], b[2], b[3], b[4], b[5], 1.0, 1.0, 0.0, cube_id);
    }
  }

  vis.addCoordinateSystem(1.5, "global");
  vis.resetCamera ();

  // Enter the main loop
  while (!vis.wasStopped ())
  {
    vis.spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
}

