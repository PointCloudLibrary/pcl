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
 * obj_rec_ransac_model_opps.cpp
 *
 *  Created on: Jan 29, 2013
 *      Author: papazov
 *
 *  Adds a model to the model library and visualizes the oriented point pairs (opps) sampled from the model.
 */

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/point_cloud.h>
#include <vtkVersion.h>
#include <vtkPolyDataReader.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkHedgeHog.h>
#include <cstdio>
#include <thread>

using namespace std::chrono_literals;
using namespace pcl;
using namespace io;
using namespace console;
using namespace recognition;
using namespace visualization;

#define _SHOW_MODEL_OCTREE_POINTS_
//#define _SHOW_MODEL_OCTREE_NORMALS_

void run (float pair_width, float voxel_size, float max_coplanarity_angle);
void showModelOpps (PCLVisualizer& viz, const ModelLibrary::HashTable& hash_table, const ModelLibrary::Model* model, float pair_width);
bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals);

//===========================================================================================================================================

int
main (int argc, char** argv)
{
  printf ("\nUsage: ./pcl_obj_rec_ransac_model_opps <pair_width> <voxel_size> <max_coplanarity_angle>\n\n");

  const int num_params = 3;
  float parameters[num_params] = {10.0f/*pair width*/, 5.0f/*voxel size*/, 5.0f/*max co-planarity angle*/};
  std::string parameter_names[num_params] = {"pair_width", "voxel_size", "max_coplanarity_angle"};

  // Read the user input if any
  for ( int i = 0 ; i < argc-1 && i < num_params ; ++i )
  {
    parameters[i] = static_cast<float> (atof (argv[i+1]));
    if ( parameters[i] <= 0.0f )
    {
      fprintf(stderr, "ERROR: the %i-th parameter has to be positive and not %f\n", i+1, parameters[i]);
      return (-1);
    }
  }

  printf ("The following parameter values will be used:\n");
  for ( int i = 0 ; i < num_params ; ++i )
    std::cout << "  " << parameter_names[i] << " = " << parameters[i] << std::endl;
  std::cout << std::endl;

  run (parameters[0], parameters[1], parameters[2]);

  return (0);
}

//===============================================================================================================================

void run (float pair_width, float voxel_size, float max_coplanarity_angle)
{
  PointCloud<PointXYZ>::Ptr model_points (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr model_normals (new PointCloud<Normal> ());

  char model_name[] = "../../test/tum_amicelli_box.vtk";

  // Get the points and normals from the input vtk file
  if ( !vtk_to_pointcloud (model_name, *model_points, *model_normals) )
    return;

  // The recognition object
  ObjRecRANSAC objrec (pair_width, voxel_size);
  objrec.setMaxCoplanarityAngleDegrees (max_coplanarity_angle);
  // Add the model
  objrec.addModel (*model_points, *model_normals, "amicelli");

  const ModelLibrary::Model* model = objrec.getModel ("amicelli");
  if ( !model )
    return;

  // The visualizer
  PCLVisualizer viz;

  // Run the recognition and update the viewer
  showModelOpps (viz, objrec.getHashTable (), model, pair_width);

  // Visualize a sphere with the radius 'pair_width'
  pcl::PointXYZ sphere_center;
  sphere_center.x = model->getOctree ().getFullLeaves ()[0]->getData ()->getPoint ()[0];
  sphere_center.y = model->getOctree ().getFullLeaves ()[0]->getData ()->getPoint ()[1];
  sphere_center.z = model->getOctree ().getFullLeaves ()[0]->getData ()->getPoint ()[2];
  viz.addSphere (sphere_center, pair_width, 0.0, 0.2, 1.0);

#ifdef _SHOW_MODEL_OCTREE_POINTS_
  PointCloud<PointXYZ>::Ptr octree_points (new PointCloud<PointXYZ> ());

  model->getOctree ().getFullLeavesPoints (*octree_points);
  viz.addPointCloud (octree_points, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "octree points");
#endif

#if defined _SHOW_MODEL_OCTREE_NORMALS_ && defined _SHOW_MODEL_OCTREE_POINTS_
  PointCloud<Normal>::Ptr octree_normals (new PointCloud<Normal> ());

  model->getOctree ().getNormalsOfFullLeaves (*octree_normals);
  viz.addPointCloudNormals<PointXYZ,Normal> (octree_points, octree_normals, 1, 6.0f, "octree normals");
#endif

  // Enter the main loop
  while (!viz.wasStopped ())
  {
    //main loop of the visualizer
    viz.spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
}

//===============================================================================================================================

void showModelOpps (PCLVisualizer& viz, const ModelLibrary::HashTable& hash_table, const ModelLibrary::Model* model, float pair_width)
{
  printf ("Visualizing ... "); fflush (stdout);

  const ModelLibrary::HashTableCell* cells = hash_table.getVoxels ();

  // The opps points and lines
  vtkSmartPointer<vtkPolyData> vtk_opps = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer<vtkPoints> vtk_opps_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> vtk_opps_lines = vtkSmartPointer<vtkCellArray>::New ();
#ifndef _SHOW_MODEL_OCTREE_NORMALS_
  vtkSmartPointer<vtkHedgeHog> vtk_hedge_hog = vtkSmartPointer<vtkHedgeHog>::New ();
  vtkSmartPointer<vtkDoubleArray> vtk_normals = vtkSmartPointer<vtkDoubleArray>::New ();
  vtk_normals->SetNumberOfComponents (3);
#endif
  vtkIdType ids[2] = {0, 1};

  // Check cell by cell
  const int num_cells = hash_table.getNumberOfVoxels ();
  for (int i = 0 ; i < num_cells ; ++i )
  {
    // Make sure that we get only point pairs belonging to 'model'
	auto res = cells[i].find (model);
    if ( res == cells[i].end () )
      continue;

    // Get the opps in the current cell
    const ModelLibrary::node_data_pair_list& data_pairs = res->second;

    for (const auto &data_pair : data_pairs)
    {
      vtk_opps_points->InsertNextPoint (data_pair.first->getPoint ());
      vtk_opps_points->InsertNextPoint (data_pair.second->getPoint ());
      vtk_opps_lines->InsertNextCell (2, ids);
      ids[0] += 2;
      ids[1] += 2;
#ifndef _SHOW_MODEL_OCTREE_NORMALS_
      vtk_normals->InsertNextTuple3 (data_pair.first->getNormal  ()[0], data_pair.first->getNormal  ()[1], data_pair.first->getNormal  ()[2]);
      vtk_normals->InsertNextTuple3 (data_pair.second->getNormal ()[0], data_pair.second->getNormal ()[1], data_pair.second->getNormal ()[2]);
#endif
    }
  }

  // Save points and connecting lines
  vtk_opps->SetPoints (vtk_opps_points);
  vtk_opps->SetLines (vtk_opps_lines);
#ifndef _SHOW_MODEL_OCTREE_NORMALS_
  // Save the normals
  vtk_opps->GetPointData ()->SetNormals (vtk_normals);
  // Setup the hedge hog object
  vtk_hedge_hog->SetInputData (vtk_opps);
  vtk_hedge_hog->SetVectorModeToUseNormal ();
  vtk_hedge_hog->SetScaleFactor (0.5f*pair_width);
  vtk_hedge_hog->Update ();
  // Show the opps' normals
  viz.addModelFromPolyData (vtk_hedge_hog->GetOutput (), "opps' normals");
#endif

  viz.addModelFromPolyData (vtk_opps, "opps");
  viz.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "opps");

  printf ("done.\n");
}

//===============================================================================================================================

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals)
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

  pcl_points.resize (num_points);

  // Copy the points
  for ( vtkIdType i = 0 ; i < num_points ; ++i )
  {
    vtk_points->GetPoint (i, p);
    pcl_points[i].x = static_cast<float> (p[0]);
    pcl_points[i].y = static_cast<float> (p[1]);
    pcl_points[i].z = static_cast<float> (p[2]);
  }

  // Check if we have normals
  vtkDataArray *vtk_normals = vtk_poly->GetPointData ()->GetNormals ();
  if ( !vtk_normals )
    return false;

  pcl_normals.resize (num_points);
  // Copy the normals
  for ( vtkIdType i = 0 ; i < num_points ; ++i )
  {
    vtk_normals->GetTuple (i, p);
    pcl_normals[i].normal_x = static_cast<float> (p[0]);
    pcl_normals[i].normal_y = static_cast<float> (p[1]);
    pcl_normals[i].normal_z = static_cast<float> (p[2]);
  }

  return true;
}

//===============================================================================================================================
