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
 * obj_rec_ransac_result.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: papazov
 *
 *  Visualizes the result of the ObjRecRANSAC class. STILL WORK IN PROGRESS!!
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
#include <vtkHedgeHog.h>
#include <cstdio>
#include <vector>

using namespace std;
using namespace pcl;
using namespace io;
using namespace console;
using namespace recognition;
using namespace visualization;

class CallbackParameters;

void run (float pair_width, float voxel_size, float max_coplanarity_angle);
void show_octree (const ORROctree& octree, PCLVisualizer& viz);
bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals);
void node_to_cube (const ORROctree::Node* node, vtkAppendPolyData* additive_octree);
void update (CallbackParameters* params);
void keyboardCB (const pcl::visualization::KeyboardEvent &event, void* params_void);

//#define _SHOW_SCENE_POINTS_
#define _SHOW_OCTREE_POINTS_
//#define _SHOW_OCTREE_NORMALS_
//#define _SHOW_OCTREE_

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
  // THIS IS STILL WORK IN PROGRESS!!

  printf ("\nUsage: ./pcl_obj_rec_ransac_scene_opps <pair_width> <voxel_size> <max_coplanarity_angle>\n\n");

  const int num_params = 3;
  float parameters[num_params] = {40.0f/*pair width*/, 5.0f/*voxel size*/, 15.0f/*max co-planarity angle*/};
  string parameter_names[num_params] = {"pair_width", "voxel_size", "max_coplanarity_angle"};

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
    cout << "  " << parameter_names[i] << " = " << parameters[i] << endl;
  cout << endl;

  run (parameters[0], parameters[1], parameters[2]);
}

//===========================================================================================================================================

void
run (float pair_width, float voxel_size, float max_coplanarity_angle)
{
  // THIS IS STILL WORK IN PROGRESS!!

  // The input to the object recognition instance
  char scene_file_name[] = "../../test/tum_table_scene.vtk\0", model_file_name[] = "../../test/tum_amicelli_box.vtk\0";

  PointCloud<PointXYZ>::Ptr scene_points (new PointCloud<PointXYZ> ()), amicelli_points (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal> ()), amicelli_normals (new PointCloud<Normal> ());

  // Get the points and normals from the input model
  if ( !vtk_to_pointcloud (model_file_name, *amicelli_points, *amicelli_normals) )
    return;

  // Get the points and normals from the input scene
  if ( !vtk_to_pointcloud (scene_file_name, *scene_points, *scene_normals) )
    return;

  // The recognition object
  ObjRecRANSAC objrec (pair_width, voxel_size);
  objrec.setMaxCoplanarityAngleDegrees (max_coplanarity_angle);
  // Add a model
  objrec.addModel (*amicelli_points, *amicelli_normals, "amicelli box");

  // The visualizer
  PCLVisualizer viz;
  CallbackParameters params(objrec, viz, *scene_points, *scene_normals);
  viz.registerKeyboardCallback (keyboardCB, static_cast<void*> (&params));

  // Run the recognition and update the viewer. Have a look at this method, to see how to start the recognition and use the result!
  update (&params);

  // From this line on: visualization stuff only!
#ifdef _SHOW_OCTREE_
  show_octree(objrec.getSceneOctree (), viz);
#endif

#ifdef _SHOW_SCENE_POINTS_
  viz.addPointCloud (scene_points, "scene points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene points");
#endif

#ifdef _SHOW_OCTREE_POINTS_
  PointCloud<PointXYZ>::Ptr octree_points (new PointCloud<PointXYZ> ());
  objrec.getSceneOctree ().getFullLeafPoints (*octree_points);
  viz.addPointCloud (octree_points, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "octree points");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "octree points");
#endif

#if defined _SHOW_OCTREE_NORMALS_ && defined _SHOW_OCTREE_POINTS_
  PointCloud<Normal>::Ptr normals_octree (new PointCloud<Normal> ());
  objrec.getSceneOctree ().getNormalsOfFullLeaves (*normals_octree);
  viz.addPointCloudNormals<PointXYZ,Normal> (points_octree, normals_octree, 1, 6.0f, "normals out");
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

void keyboardCB (const pcl::visualization::KeyboardEvent &event, void* params_void)
{
  if (event.getKeyCode () == 13 /*enter*/ && event.keyUp ())
    update (static_cast<CallbackParameters*> (params_void));
}

//===============================================================================================================================

void update (CallbackParameters* params)
{
  // That will be the output of the recognition
  list<ObjRecRANSAC::Output> rec_output;

  // Run the recognition method
  params->objrec_.recognize (params->points_, params->normals_, rec_output);

  cout << "There is (are) " << rec_output.size () << " recognized object(s).\n";
}

//===============================================================================================================================

bool vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals)
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
