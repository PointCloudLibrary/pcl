/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

/*
 * obj_rec_ransac_orr_octree_zprojection.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: papazov
 *
 *  Visualizes the specialized octree class used for the ransac-based object
 *  recognition. Use the left/right arrows to select a full octree leaf which
 *  will be used to place a sphere at it and to cut the sphere against the
 *  other full octree leaves which are visualized in yellow.
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/recognition/ransac_based/orr_octree_zprojection.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkVersion.h>
#include <vtkRenderWindow.h>
#include <vtkPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkCubeSource.h>
#include <vtkPointData.h>
#include <vector>
#include <list>
#include <cstdlib>
#include <cstring>
#include <cstdio>

using namespace pcl;
using namespace pcl::visualization;
using namespace pcl::recognition;
using namespace pcl::io;

void run (const char *file_name, float voxel_size);
bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& points);
void show_octree (ORROctree* octree, PCLVisualizer& viz);
void show_octree_zproj (ORROctreeZProjection* zproj, PCLVisualizer& viz);
void node_to_cube (ORROctree::Node* node, vtkAppendPolyData* additive_octree);
void rectangle_to_vtk (float x1, float x2, float y1, float y2, float z, vtkAppendPolyData* additive_rectangle);

//#define _SHOW_POINTS_

int main (int argc, char ** argv)
{
  if ( argc != 3 )
  {
    fprintf(stderr, "\nERROR: Syntax is ./pcl_obj_rec_ransac_orr_octree_zprojection <vtk file> <leaf_size>\n"
                    "EXAMPLE: ./pcl_obj_rec_ransac_orr_octree_zprojection ../../test/tum_table_scene.vtk 6\n\n");
    return -1;
  }

  // Get the voxel size
  float voxel_size = static_cast<float> (atof (argv[2]));
  if ( voxel_size <= 0.0 )
  {
    fprintf(stderr, "ERROR: leaf_size has to be positive and not %lf\n", voxel_size);
    return -1;
  }

  run(argv[1], voxel_size);
}

//===============================================================================================================================

void run (const char* file_name, float voxel_size)
{
  PointCloud<PointXYZ>::Ptr points_in (new PointCloud<PointXYZ> ());
  PointCloud<PointXYZ>::Ptr points_out (new PointCloud<PointXYZ> ());

  // Get the points and normals from the input vtk file
  if ( !vtk_to_pointcloud (file_name, *points_in) )
    return;

  // Build the octree with the desired resolution
  ORROctree octree;
  octree.build (*points_in, voxel_size);

  // Now build the octree z-projection
  ORROctreeZProjection zproj;
  zproj.build (octree, 0.15f*voxel_size, 0.15f*voxel_size);

  // The visualizer
  PCLVisualizer viz;

  show_octree(&octree, viz);
  show_octree_zproj(&zproj, viz);

#ifdef _SHOW_POINTS_
  // Get the point of every full octree leaf
  octree.getFullLeafPoints (*points_out);

  // Add the point clouds
  viz.addPointCloud (points_in, "cloud in");
  viz.addPointCloud (points_out, "cloud out");

  // Change the appearance
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud in");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud out");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud out");
#endif

  // Enter the main loop
  while (!viz.wasStopped ())
  {
    //main loop of the visualizer
    viz.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

//===============================================================================================================================

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points)
{
  size_t len = strlen (file_name);
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

  pcl_points.resize (num_points);

  // Copy the points
  for ( vtkIdType i = 0 ; i < num_points ; ++i )
  {
    vtk_points->GetPoint (i, p);
    pcl_points[i].x = static_cast<float> (p[0]);
    pcl_points[i].y = static_cast<float> (p[1]);
    pcl_points[i].z = static_cast<float> (p[2]);
  }

  return true;
}

//===============================================================================================================================

void show_octree (ORROctree* octree, PCLVisualizer& viz)
{
  vtkSmartPointer<vtkPolyData> vtk_octree = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer<vtkAppendPolyData> append = vtkSmartPointer<vtkAppendPolyData>::New ();

  cout << "There are " << octree->getFullLeaves ().size () << " full leaves.\n";

  std::vector<ORROctree::Node*>& full_leaves = octree->getFullLeaves ();
  for ( std::vector<ORROctree::Node*>::iterator it = full_leaves.begin () ; it != full_leaves.end () ; ++it )
    // Add it to the other cubes
    node_to_cube (*it, append);

  // Save the result
  append->Update();
  vtk_octree->DeepCopy (append->GetOutput ());

  // Add to the visualizer
  vtkRenderer *renderer = viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ();
  vtkSmartPointer<vtkActor> octree_actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
#if VTK_MAJOR_VERSION < 6
  mapper->SetInput(vtk_octree);
#else
  mapper->SetInputData (vtk_octree);
#endif
  octree_actor->SetMapper(mapper);

  // Set the appearance & add to the renderer
  octree_actor->GetProperty ()->SetColor (1.0, 1.0, 1.0);
  renderer->AddActor(octree_actor);
}

//===============================================================================================================================

void show_octree_zproj (ORROctreeZProjection* zproj, PCLVisualizer& viz)
{
  cout << "There is (are) " << zproj->getFullPixels ().size () << " full pixel(s).\n";

  vtkSmartPointer<vtkAppendPolyData> upper_bound = vtkSmartPointer<vtkAppendPolyData>::New (), lower_bound = vtkSmartPointer<vtkAppendPolyData>::New ();
  const ORROctreeZProjection::Pixel *pixel;
  const float *b = zproj->getBounds ();
  float x, y, psize = zproj->getPixelSize ();
  int i, j, width, height;

  zproj->getNumberOfPixels (width, height);

  for ( i = 0, x = b[0] ; i < width ; ++i, x += psize )
  {
    for ( j = 0, y = b[2] ; j < height ; ++j, y += psize )
    {
      pixel = zproj->getPixel (i, j);

      if ( !pixel )
        continue;

      rectangle_to_vtk (x, x + psize, y, y + psize, pixel->z1 (), lower_bound);
      rectangle_to_vtk (x, x + psize, y, y + psize, pixel->z2 (), upper_bound);
    }
  }

  // Save the result
  upper_bound->Update();
  lower_bound->Update();

  // Add to the visualizer
  vtkRenderer *renderer = viz.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ();
  vtkSmartPointer<vtkActor> upper_actor = vtkSmartPointer<vtkActor>::New(), lower_actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkDataSetMapper> upper_mapper = vtkSmartPointer<vtkDataSetMapper>::New (), lower_mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
#if VTK_MAJOR_VERSION < 6
  upper_mapper->SetInput(upper_bound->GetOutput ());
#else
  upper_mapper->SetInputData (upper_bound->GetOutput ());
#endif
  upper_actor->SetMapper(upper_mapper);
#if VTK_MAJOR_VERSION < 6
  lower_mapper->SetInput(lower_bound->GetOutput ());
#else
  lower_mapper->SetInputData (lower_bound->GetOutput ());
#endif
  lower_actor->SetMapper(lower_mapper);

  // Set the appearance & add to the renderer
  upper_actor->GetProperty ()->SetColor (1.0, 0.0, 0.0);
  renderer->AddActor(upper_actor);
  lower_actor->GetProperty ()->SetColor (1.0, 1.0, 0.0);
  renderer->AddActor(lower_actor);
}

//===============================================================================================================================

void node_to_cube (ORROctree::Node* node, vtkAppendPolyData* additive_octree)
{
  // Define the cube representing the leaf
  const float *b = node->getBounds ();
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (b[0], b[1], b[2], b[3], b[4], b[5]);
  cube->Update ();

#if VTK_MAJOR_VERSION < 6
  additive_octree->AddInput (cube->GetOutput ());
#else
  additive_octree->AddInputData (cube->GetOutput ());
#endif
}

//===============================================================================================================================

void rectangle_to_vtk (float x1, float x2, float y1, float y2, float z, vtkAppendPolyData* additive_rectangle)
{
  // Define the cube representing the leaf
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New ();
  cube->SetBounds (x1, x2, y1, y2, z, z);
  cube->Update ();

#if VTK_MAJOR_VERSION < 6
  additive_rectangle->AddInput (cube->GetOutput ());
#else
  additive_rectangle->AddInputData (cube->GetOutput ());
#endif
}

//===============================================================================================================================
