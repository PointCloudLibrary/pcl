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
 * obj_rec_ransac_scene_opps.cpp
 *
 *  Created on: Jan 17, 2013
 *      Author: papazov
 *
 *  Calls recognize() of the ObjRecRANSAC class and visualizes the oriented point pairs (opp) sampled from the scene.
 *  Does NOT perform full recognition.
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

class CallbackParameters;

void run (float pair_width, float voxel_size, float max_coplanarity_angle);
bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals);
void update (CallbackParameters* params);

//#define _SHOW_SCENE_POINTS_
#define _SHOW_OCTREE_POINTS_
//#define _SHOW_OCTREE_NORMALS_

class CallbackParameters
{
  public:
    CallbackParameters (ObjRecRANSAC& objrec, PCLVisualizer& viz, PointCloud<PointXYZ>& points, PointCloud<Normal>& normals)
    : objrec_ (objrec),
      viz_ (viz),
      points_ (points),
      normals_ (normals)
    { }

    ObjRecRANSAC& objrec_;
    PCLVisualizer& viz_;
    PointCloud<PointXYZ>& points_;
    PointCloud<Normal>& normals_;
};

//===========================================================================================================================================

int
main (int argc, char** argv)
{
  printf ("\nUsage: ./pcl_obj_rec_ransac_scene_opps <pair_width> <voxel_size> <max_coplanarity_angle>\n\n");

  const int num_params = 3;
  float parameters[num_params] = {40.0f/*pair width*/, 5.0f/*voxel size*/, 15.0f/*max co-planarity angle*/};
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

void keyboardCB (const pcl::visualization::KeyboardEvent &event, void* params_void)
{
  if (event.getKeyCode () == 13 /*enter*/ && event.keyUp ())
    update (static_cast<CallbackParameters*> (params_void));
}

//===============================================================================================================================

void update (CallbackParameters* params)
{
  std::list<ObjRecRANSAC::Output> dummy_output;

  // Run the recognition method
  params->objrec_.recognize (params->points_, params->normals_, dummy_output);

  // Build the vtk objects visualizing the lines between the opps
  const std::list<ObjRecRANSAC::OrientedPointPair>& opps = params->objrec_.getSampledOrientedPointPairs ();
  std::cout << "There is (are) " << opps.size () << " oriented point pair(s).\n";
  // The opps points
  vtkSmartPointer<vtkPolyData> vtk_opps = vtkSmartPointer<vtkPolyData>::New ();
  vtkSmartPointer<vtkPoints> vtk_opps_points = vtkSmartPointer<vtkPoints>::New ();
    vtk_opps_points->SetNumberOfPoints (2*static_cast<vtkIdType> (opps.size ()));
  vtkSmartPointer<vtkCellArray> vtk_opps_lines = vtkSmartPointer<vtkCellArray>::New ();
  // The opps normals
  vtkSmartPointer<vtkDoubleArray> vtk_normals = vtkSmartPointer<vtkDoubleArray>::New ();
  vtk_normals->SetNumberOfComponents (3);
  vtk_normals->SetNumberOfTuples (2*static_cast<vtkIdType> (opps.size ()));
  vtkIdType ids[2] = {0, 1};

  // Insert the points and compute the lines
  for (const auto &opp : opps)
  {
    vtk_opps_points->SetPoint (ids[0], opp.p1_[0], opp.p1_[1], opp.p1_[2]);
    vtk_opps_points->SetPoint (ids[1], opp.p2_[0], opp.p2_[1], opp.p2_[2]);
    vtk_opps_lines->InsertNextCell (2, ids);

    vtk_normals->SetTuple3 (ids[0], opp.n1_[0], opp.n1_[1], opp.n1_[2]);
    vtk_normals->SetTuple3 (ids[1], opp.n2_[0], opp.n2_[1], opp.n2_[2]);

    ids[0] += 2;
    ids[1] += 2;
  }
  vtk_opps->SetPoints (vtk_opps_points);
  vtk_opps->GetPointData ()->SetNormals (vtk_normals);
  vtk_opps->SetLines (vtk_opps_lines);

  vtkSmartPointer<vtkHedgeHog> vtk_hh = vtkSmartPointer<vtkHedgeHog>::New ();
  vtk_hh->SetVectorModeToUseNormal ();
  vtk_hh->SetScaleFactor (0.5f*params->objrec_.getPairWidth ());
  vtk_hh->SetInputData (vtk_opps);
  vtk_hh->Update ();

  // The lines
  std::string lines_str_id = "opps";
  params->viz_.removeShape(lines_str_id);
  params->viz_.addModelFromPolyData (vtk_opps, lines_str_id);
  params->viz_.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, lines_str_id);
  // The normals
  std::string normals_str_id = "opps normals";
  params->viz_.removeShape(normals_str_id);
  params->viz_.addModelFromPolyData (vtk_hh->GetOutput (), normals_str_id);
  params->viz_.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, normals_str_id);

}

//===============================================================================================================================

void run (float pair_width, float voxel_size, float max_coplanarity_angle)
{
  PointCloud<PointXYZ>::Ptr scene_points (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal> ());

  // Get the points and normals from the input vtk file
  if ( !vtk_to_pointcloud ("../../test/tum_table_scene.vtk", *scene_points, *scene_normals) )
    return;

  // The recognition object
  ObjRecRANSAC objrec (pair_width, voxel_size);
  objrec.setMaxCoplanarityAngleDegrees (max_coplanarity_angle);
  // Switch to the test mode in which only oriented point pairs from the scene are sampled
  objrec.enterTestModeSampleOPP ();

  // The visualizer
  PCLVisualizer viz;

  CallbackParameters params(objrec, viz, *scene_points, *scene_normals);
  viz.registerKeyboardCallback (keyboardCB, static_cast<void*> (&params));

  // Run the recognition and update the viewer
  update (&params);

#ifdef _SHOW_SCENE_POINTS_
  viz.addPointCloud (scene_points, "cloud in");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud in");
#endif

#ifdef _SHOW_OCTREE_POINTS_
  PointCloud<PointXYZ>::Ptr octree_points (new PointCloud<PointXYZ> ());
  objrec.getSceneOctree ().getFullLeavesPoints (*octree_points);
  viz.addPointCloud (octree_points, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "octree points");
#endif

#if defined _SHOW_OCTREE_NORMALS_ && defined _SHOW_OCTREE_POINTS_
  PointCloud<Normal>::Ptr octree_normals (new PointCloud<Normal> ());
  objrec.getSceneOctree ().getNormalsOfFullLeaves (*octree_normals);
  viz.addPointCloudNormals<PointXYZ,Normal> (octree_points, octree_normals, 1, 6.0f, "normals out");
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
