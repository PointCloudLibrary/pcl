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
 * obj_rec_ransac_accepted_hypotheses.cpp
 *
 *  Created on: Feb 01, 2013
 *      Author: papazov
 *
 *  Adds a model and calls recognize() of the ObjRecRANSAC class and visualizes some of the accepted hypotheses and
 *  the oriented point pairs (opps) sampled from the scene. Does NOT perform full recognition.
 */

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <vtkVersion.h>
#include <vtkPolyDataReader.h>
#include <vtkRenderWindow.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkTransform.h>
#include <vtkHedgeHog.h>
#include <vtkMatrix4x4.h>
#include <algorithm>
#include <cstdio>
#include <vector>

using namespace std;
using namespace pcl;
using namespace io;
using namespace console;
using namespace recognition;
using namespace visualization;

bool
vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals, vtkPolyData* vtk_dst = NULL);

//#define _SHOW_SCENE_POINTS_
#define _SHOW_OCTREE_POINTS_
//#define _SHOW_SCENE_OPPS_
//#define _SHOW_OCTREE_NORMALS_

class CallbackParameters
{
  public:
    CallbackParameters (ObjRecRANSAC& objrec, PCLVisualizer& viz, PointCloud<PointXYZ>& points, PointCloud<Normal>& normals, int num_hypotheses_to_show)
    : objrec_ (objrec),
      viz_ (viz),
      points_ (points),
      normals_ (normals),
      num_hypotheses_to_show_ (num_hypotheses_to_show),
      show_models_ (true)
    { }

    ObjRecRANSAC& objrec_;
    PCLVisualizer& viz_;
    PointCloud<PointXYZ>& points_;
    PointCloud<Normal>& normals_;
    int num_hypotheses_to_show_;
    list<vtkActor*> actors_, model_actors_;
    bool show_models_;
};

//===============================================================================================================================

bool
vtk_to_pointcloud (const char* file_name, PointCloud<PointXYZ>& pcl_points, PointCloud<Normal>& pcl_normals, vtkPolyData* vtk_dst)
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

  // Shall we copy the file to 'vtk_dst'
  if ( vtk_dst )
    vtk_dst->DeepCopy (vtk_poly);

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

void
showHypothesisAsCoordinateFrame (Hypothesis& hypo, CallbackParameters* parameters, string frame_name)
{
  float rot_col[3], x_dir[3], y_dir[3], z_dir[3], origin[3], scale = 2.0f*parameters->objrec_.getPairWidth ();
  pcl::ModelCoefficients coeffs; coeffs.values.resize (6);

  // Get the origin of the coordinate frame
  aux::transform (hypo.rigid_transform_, hypo.obj_model_->getOctreeCenterOfMass (), origin);
  coeffs.values[0] = origin[0];
  coeffs.values[1] = origin[1];
  coeffs.values[2] = origin[2];
  // Setup the axes
  rot_col[0] = hypo.rigid_transform_[0];
  rot_col[1] = hypo.rigid_transform_[3];
  rot_col[2] = hypo.rigid_transform_[6];
  aux::mult3 (rot_col, scale, x_dir);
  rot_col[0] = hypo.rigid_transform_[1];
  rot_col[1] = hypo.rigid_transform_[4];
  rot_col[2] = hypo.rigid_transform_[7];
  aux::mult3 (rot_col, scale, y_dir);
  rot_col[0] = hypo.rigid_transform_[2];
  rot_col[1] = hypo.rigid_transform_[5];
  rot_col[2] = hypo.rigid_transform_[8];
  aux::mult3 (rot_col, scale, z_dir);

  // The x-axis
  coeffs.values[3] = x_dir[0];
  coeffs.values[4] = x_dir[1];
  coeffs.values[5] = x_dir[2];
  parameters->viz_.addLine (coeffs, frame_name + "_x_axis");
  parameters->viz_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, frame_name + "_x_axis");

  // The y-axis
  coeffs.values[3] = y_dir[0];
  coeffs.values[4] = y_dir[1];
  coeffs.values[5] = y_dir[2];
  parameters->viz_.addLine (coeffs, frame_name + "_y_axis");
  parameters->viz_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, frame_name + "_y_axis");

  // The z-axis
  coeffs.values[3] = z_dir[0];
  coeffs.values[4] = z_dir[1];
  coeffs.values[5] = z_dir[2];
  parameters->viz_.addLine (coeffs, frame_name + "_z_axis");
  parameters->viz_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, frame_name + "_z_axis");
}

//===============================================================================================================================

bool
compareHypotheses (const Hypothesis& a, const Hypothesis& b)
{
  return (static_cast<bool> (a.match_confidence_ > b.match_confidence_));
}

//===============================================================================================================================

void
arrayToVtkMatrix (const float* a, vtkMatrix4x4* m)
{
  // Setup the rotation
  m->SetElement (0, 0, a[0]); m->SetElement (0, 1, a[1]); m->SetElement (0, 2, a[2]);
  m->SetElement (1, 0, a[3]); m->SetElement (1, 1, a[4]); m->SetElement (1, 2, a[5]);
  m->SetElement (2, 0, a[6]); m->SetElement (2, 1, a[7]); m->SetElement (2, 2, a[8]);
  // Setup the translation
  m->SetElement (0, 3, a[9]); m->SetElement (1, 3, a[10]); m->SetElement (2, 3, a[11]);
}

//===============================================================================================================================

void
update (CallbackParameters* params)
{
  list<ObjRecRANSAC::Output> dummy_output;

  // Run the recognition method
  params->objrec_.recognize (params->points_, params->normals_, dummy_output);

  // Clear the visualizer
  vtkRenderer *renderer = params->viz_.getRenderWindow ()->GetRenderers ()->GetFirstRenderer ();
  for ( list<vtkActor*>::iterator it = params->actors_.begin () ; it != params->actors_.end () ; ++it )
    renderer->RemoveActor (*it);
  params->actors_.clear ();

  for ( list<vtkActor*>::iterator it = params->model_actors_.begin () ; it != params->model_actors_.end () ; ++it )
    renderer->RemoveActor (*it);
  params->model_actors_.clear ();

  params->viz_.removeAllShapes ();

#ifdef _SHOW_SCENE_OPPS_
  // Build the vtk objects visualizing the lines between the opps
  const list<ObjRecRANSAC::OrientedPointPair>& opps = params->objrec_.getSampledOrientedPointPairs ();
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
  for ( list<ObjRecRANSAC::OrientedPointPair>::const_iterator it = opps.begin () ; it != opps.end () ; ++it )
  {
    vtk_opps_points->SetPoint (ids[0], it->p1_[0], it->p1_[1], it->p1_[2]);
    vtk_opps_points->SetPoint (ids[1], it->p2_[0], it->p2_[1], it->p2_[2]);
    vtk_opps_lines->InsertNextCell (2, ids);

    vtk_normals->SetTuple3 (ids[0], it->n1_[0], it->n1_[1], it->n1_[2]);
    vtk_normals->SetTuple3 (ids[1], it->n2_[0], it->n2_[1], it->n2_[2]);

    ids[0] += 2;
    ids[1] += 2;
  }
  vtk_opps->SetPoints (vtk_opps_points);
  vtk_opps->GetPointData ()->SetNormals (vtk_normals);
  vtk_opps->SetLines (vtk_opps_lines);

  vtkSmartPointer<vtkHedgeHog> vtk_hh = vtkSmartPointer<vtkHedgeHog>::New ();
  vtk_hh->SetVectorModeToUseNormal ();
  vtk_hh->SetScaleFactor (0.5f*params->objrec_.getPairWidth ());
  vtk_hh->SetInput (vtk_opps);
  vtk_hh->Update ();

  // The lines
  string lines_str_id = "opps";
  params->viz_.addModelFromPolyData (vtk_opps, lines_str_id);
  params->viz_.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, lines_str_id);
  // The normals
  string normals_str_id = "opps normals";
  params->viz_.addModelFromPolyData (vtk_hh->GetOutput (), normals_str_id);
  params->viz_.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, normals_str_id);
#endif

  // Now show some of the accepted hypotheses
  vector<Hypothesis> accepted_hypotheses;
  params->objrec_.getAcceptedHypotheses (accepted_hypotheses);
  int i = 0;

  // Sort the hypotheses vector such that the strongest hypotheses are at the front
  std::sort(accepted_hypotheses.begin (), accepted_hypotheses.end (), compareHypotheses);

  // Show the hypotheses
  for ( vector<Hypothesis>::iterator acc_hypo = accepted_hypotheses.begin () ; i < params->num_hypotheses_to_show_ && acc_hypo != accepted_hypotheses.end () ; ++i, ++acc_hypo )
  {
    // Visualize the orientation as a tripod
    char frame_name[128];
    sprintf (frame_name, "frame_%i", i+1);
    showHypothesisAsCoordinateFrame (*acc_hypo, params, frame_name);

    // Make a copy of the VTK model
    vtkSmartPointer<vtkPolyData> vtk_model = vtkSmartPointer<vtkPolyData>::New ();
    vtk_model->DeepCopy (static_cast<vtkPolyData*> (acc_hypo->obj_model_->getUserData ()));

    // Setup the matrix
    vtkSmartPointer<vtkMatrix4x4> vtk_mat = vtkSmartPointer<vtkMatrix4x4>::New ();
    vtk_mat->Identity ();
    arrayToVtkMatrix (acc_hypo->rigid_transform_, vtk_mat);
    // Setup the transform based on the matrix
    vtkSmartPointer<vtkTransform> vtk_transform = vtkSmartPointer<vtkTransform>::New ();
    vtk_transform->SetMatrix (vtk_mat);
    // Setup the transformator
    vtkSmartPointer<vtkTransformPolyDataFilter> vtk_transformator = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
    vtk_transformator->SetTransform (vtk_transform);
#if VTK_MAJOR_VERSION < 6
    vtk_transformator->SetInput (vtk_model);
#else
    vtk_transformator->SetInputData (vtk_model);
#endif
    vtk_transformator->Update ();

    // Visualize
    vtkSmartPointer<vtkActor> vtk_actor = vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
#if VTK_MAJOR_VERSION < 6
    vtk_mapper->SetInput(vtk_transformator->GetOutput ());
#else
    vtk_mapper->SetInputData (vtk_transformator->GetOutput ());
#endif
    vtk_actor->SetMapper(vtk_mapper);
    // Set the appearance & add to the renderer
    vtk_actor->GetProperty ()->SetColor (0.6, 0.7, 0.9);
    renderer->AddActor(vtk_actor);
    params->model_actors_.push_back (vtk_actor);

    // Compose the model's id
    cout << acc_hypo->obj_model_->getObjectName () << "_" << i+1 << " has a confidence value of " << acc_hypo->match_confidence_ << ";  ";
  }

  // Show the bounds of the scene octree
  const float* ob = params->objrec_.getSceneOctree ().getBounds ();
  params->viz_.addCube (ob[0], ob[1], ob[2], ob[3], ob[4], ob[5], 1.0, 1.0, 1.0);

#if 0
  // Compute the angle
  float angle = static_cast<float> (aux::getRandomInteger (0, 100));
  angle -= AUX_PI_FLOAT*std::floor (angle/AUX_PI_FLOAT); // angle = angle mod pi

  // Compute the axis
  Eigen::Matrix<float,3,1> axis;
  axis(0,0) = static_cast<float> (aux::getRandomInteger (-100, 100));
  axis(1,0) = static_cast<float> (aux::getRandomInteger (-100, 100));
  axis(2,0) = static_cast<float> (aux::getRandomInteger (-100, 100));
  // Normalize the axis
  float len = std::sqrt (axis(0,0)*axis(0,0) + axis(1,0)*axis(1,0) + axis(2,0)*axis(2,0));
  axis(0,0) = axis(0,0)/len;
  axis(1,0) = axis(1,0)/len;
  axis(2,0) = axis(2,0)/len;

  cout << "Input angle = " << angle << endl;
  cout << "Input axis = \n" << axis << endl;

  // The eigen axis-angle object
  Eigen::AngleAxis<float> angle_axis(angle, axis);

  Eigen::Matrix<float,3,3> mat = angle_axis.toRotationMatrix ();
  float m[9];
  aux::eigenMatrix3x3ToArray9RowMajor (mat, m);

  // Now compute back the angle and the axis based on eigen
  float comp_angle, comp_axis[3];
  aux::rotationMatrixToAxisAngle (m, comp_axis, comp_angle);
  cout << "\nComputed angle = " << comp_angle << endl;
  cout << "Computed axis = \n" << comp_axis[0] << "\n" << comp_axis[1] << "\n" << comp_axis[2] << endl;
#endif
}

//===============================================================================================================================

void
keyboardCB (const pcl::visualization::KeyboardEvent &event, void* params_void)
{
  CallbackParameters* params = static_cast<CallbackParameters*> (params_void);

  if (event.getKeyCode () == 13 /*enter*/ && event.keyUp ())
    update (params);
  else if ( event.getKeyCode () == '1' && event.keyUp () )
  {
    // Switch models visibility
    params->show_models_ = !params->show_models_;

    for ( list<vtkActor*>::iterator it = params->model_actors_.begin () ; it != params->model_actors_.end () ; ++it )
      (*it)->SetVisibility (static_cast<int> (params->show_models_));

    params->viz_.getRenderWindow ()->Render ();
  }
}

//===============================================================================================================================

void
run (float pair_width, float voxel_size, float max_coplanarity_angle, int num_hypotheses_to_show)
{
  PointCloud<PointXYZ>::Ptr scene_points (new PointCloud<PointXYZ> ()), model_points (new PointCloud<PointXYZ> ());
  PointCloud<Normal>::Ptr scene_normals (new PointCloud<Normal> ()), model_normals (new PointCloud<Normal> ());

  // Get the points and normals from the scene
  if ( !vtk_to_pointcloud ("../../test/tum_table_scene.vtk", *scene_points, *scene_normals) )
    return;

  vtkPolyData *vtk_model = vtkPolyData::New ();
  // Get the points and normals from the scene
  if ( !vtk_to_pointcloud ("../../test/tum_amicelli_box.vtk", *model_points, *model_normals, vtk_model) )
    return;

  // The recognition object
  ObjRecRANSAC objrec (pair_width, voxel_size);
  objrec.setMaxCoplanarityAngleDegrees (max_coplanarity_angle);
  objrec.addModel (*model_points, *model_normals, "amicelli", vtk_model);
  // Switch to the test mode in which only oriented point pairs from the scene are sampled
  objrec.enterTestModeTestHypotheses ();

  // The visualizer
  PCLVisualizer viz;

  CallbackParameters params(objrec, viz, *scene_points, *scene_normals, num_hypotheses_to_show);
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
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  vtk_model->Delete ();
}

//===========================================================================================================================================

int
main (int argc, char** argv)
{
  printf ("\nUsage: ./obj_rec_ransac_accepted_hypotheses <pair_width> <voxel_size> <max_coplanarity_angle> <n_hypotheses_to_show> <show_hypotheses_as_coordinate_frames>\n\n");

  const int num_params = 4;
  float parameters[num_params] = {40.0f/*pair width*/, 5.0f/*voxel size*/, 15.0f/*max co-planarity angle*/, 1/*n_hypotheses_to_show*/};
  string parameter_names[num_params] = {"pair_width", "voxel_size", "max_coplanarity_angle", "n_hypotheses_to_show"};

  // Read the user input if any
  for ( int i = 0 ; i < argc-1 && i < num_params ; ++i )
    parameters[i] = static_cast<float> (atof (argv[i+1]));

  printf ("The following parameter values will be used:\n");
  for ( int i = 0 ; i < num_params ; ++i )
    cout << "  " << parameter_names[i] << " = " << parameters[i] << endl;
  cout << endl;

  run (parameters[0], parameters[1], parameters[2], static_cast<int> (parameters[3] + 0.5f));

  return (0);
}

//===============================================================================================================================
