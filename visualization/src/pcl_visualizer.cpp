/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: pcl_visualizer.cpp 36004 2011-02-15 16:58:41Z dirkholz $
 *
 */

#include <pcl/common/common_headers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCellData.h>

/////////////////////////////////////////////////////////////////////////////////////////////
pcl_visualization::PCLVisualizer::PCLVisualizer (const std::string &name) : 
    rens_ (vtkSmartPointer<vtkRendererCollection>::New ()),
    style_ (vtkSmartPointer<pcl_visualization::PCLVisualizerInteractorStyle>::New ())
{
  // FPS callback
  vtkSmartPointer<vtkTextActor> txt = vtkSmartPointer<vtkTextActor>::New ();
  vtkSmartPointer<FPSCallback> update_fps = vtkSmartPointer<FPSCallback>::New ();
  update_fps->setTextActor (txt);

  // Create a Renderer
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
  ren->AddObserver (vtkCommand::EndEvent, update_fps);
  ren->AddActor (txt);
  // Add it to the list of renderers
  rens_->AddItem (ren);
  
  // Create a RendererWindow
  win_ = vtkSmartPointer<vtkRenderWindow>::New ();
  win_->SetWindowName (name.c_str ());
  win_->SetStereoTypeToAnaglyph ();

  // Get screen size
  int *scr_size = win_->GetScreenSize ();
  // Set the window size as 1/2 of the screen size
  win_->SetSize (scr_size[0] / 2, scr_size[1] / 2);
  
  // Add all renderers to the window
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  while ((renderer = rens_->GetNextItem ()) != NULL)
    win_->AddRenderer (renderer);

  // Create the interactor style
  style_->Initialize ();
  style_->setRendererCollection (rens_);
  style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
  style_->UseTimersOn ();

  // Create the interactor
  //interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New ();
  interactor_ = vtkSmartPointer<PCLVisualizerInteractor>::New ();

  interactor_->SetRenderWindow (win_);
  interactor_->SetInteractorStyle (style_);
  interactor_->SetDesiredUpdateRate (30.0);
  // Initialize and create timer
  interactor_->Initialize ();
  interactor_->CreateRepeatingTimer (5000L);

  exit_main_loop_timer_callback_ = vtkSmartPointer<ExitMainLoopTimerCallback>::New ();
  exit_main_loop_timer_callback_->pcl_visualizer = this;
  exit_main_loop_timer_callback_->right_timer_id = -1;
  interactor_->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

  exit_callback_ = vtkSmartPointer<ExitCallback>::New ();
  exit_callback_->pcl_visualizer = this;
  interactor_->AddObserver (vtkCommand::ExitEvent, exit_callback_);
  
  resetStoppedFlag ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl_visualization::PCLVisualizer::PCLVisualizer (int &argc, char **argv, const std::string &name, PCLVisualizerInteractorStyle* style) : 
    rens_ (vtkSmartPointer<vtkRendererCollection>::New ())
{
  style_ = style;

  // FPS callback
  vtkSmartPointer<vtkTextActor> txt = vtkSmartPointer<vtkTextActor>::New ();
  vtkSmartPointer<FPSCallback> update_fps = vtkSmartPointer<FPSCallback>::New ();
  update_fps->setTextActor (txt);

  // Create a Renderer
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
  ren->AddObserver (vtkCommand::EndEvent, update_fps);
  ren->AddActor (txt);
  // Add it to the list of renderers
  rens_->AddItem (ren);
  
  // Create a RendererWindow
  win_ = vtkSmartPointer<vtkRenderWindow>::New ();
  win_->SetWindowName (name.c_str ());
  win_->SetStereoTypeToAnaglyph ();

  // Get screen size
  int *scr_size = win_->GetScreenSize ();
  camera_.window_size[0] = scr_size[0]; camera_.window_size[1] = scr_size[1] / 2;
  camera_.window_pos[0] = camera_.window_pos[1] = 0;
 
  // Set default camera parameters
  initCameraParameters ();
 
  // Parse the camera settings and update the internal camera
  getCameraParameters (argc, argv);
  updateCamera ();
  // Set the window size as 1/2 of the screen size or the user given parameter
  win_->SetSize (camera_.window_size[0], camera_.window_size[1]);
  win_->SetPosition (camera_.window_pos[0], camera_.window_pos[1]);
 
  // Add all renderers to the window
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  while ((renderer = rens_->GetNextItem ()) != NULL)
    win_->AddRenderer (renderer);

  // Create the interactor style
  style_->Initialize ();
  style_->setRendererCollection (rens_);
  style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
  style_->UseTimersOn ();

  // Create the interactor
  //interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New ();
  interactor_ = vtkSmartPointer<PCLVisualizerInteractor>::New ();

  interactor_->SetRenderWindow (win_);
  interactor_->SetInteractorStyle (style_);
  interactor_->SetDesiredUpdateRate (30.0);
  // Initialize and create timer
  interactor_->Initialize ();
  interactor_->CreateRepeatingTimer (5000L);

  exit_main_loop_timer_callback_ = vtkSmartPointer<ExitMainLoopTimerCallback>::New();
  exit_main_loop_timer_callback_->pcl_visualizer = this;
  exit_main_loop_timer_callback_->right_timer_id = -1;
  interactor_->AddObserver(vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

  exit_callback_ = vtkSmartPointer<ExitCallback>::New();
  exit_callback_->pcl_visualizer = this;
  interactor_->AddObserver(vtkCommand::ExitEvent, exit_callback_);
  
  resetStoppedFlag ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl_visualization::PCLVisualizer::~PCLVisualizer ()
{
  // Clear the collections
  rens_->RemoveAllItems ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::spin ()
{
  resetStoppedFlag ();
  // Render the window before we start the interactor
  win_->Render();
  interactor_->Start ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::spinOnce (int time, bool force_redraw)
{
  resetStoppedFlag ();

  if (time <= 0)
    time = 1;
  
  if (force_redraw)
  {
    interactor_->Render();
    exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer(time);
    interactor_->Start();
    interactor_->DestroyTimer(exit_main_loop_timer_callback_->right_timer_id);
    return;
  }
  
  DO_EVERY(1.0/interactor_->GetDesiredUpdateRate(),
    interactor_->Render();
    exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer(time);
    interactor_->Start();
    interactor_->DestroyTimer(exit_main_loop_timer_callback_->right_timer_id);
  );
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::addCoordinateSystem (double scale, int viewport)
{
  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
  axes->SetOrigin (0, 0, 0);
  axes->SetScaleFactor (scale);
                                                                                            
  vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
  axes_colors->Allocate (6);           
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (1.0);
  axes_colors->InsertNextValue (1.0);

  vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
  axes_data->Update ();
  axes_data->GetPointData ()->SetScalars (axes_colors);            

  vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
  axes_tubes->SetInput (axes_data);
  axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
  axes_tubes->SetNumberOfSides (6);
  
  vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  axes_mapper->SetScalarModeToUsePointData ();
  axes_mapper->SetInput (axes_tubes->GetOutput ());
                                                                                            
  vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
  axes_actor->SetMapper (axes_mapper);

  addActorToRenderer (axes_actor, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::addCoordinateSystem (double scale, float x, float y, float z, int viewport)
{
  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
  axes->SetOrigin (0, 0, 0);
  axes->SetScaleFactor (scale);
                                                                                            
  vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
  axes_colors->Allocate (6);           
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (1.0);
  axes_colors->InsertNextValue (1.0);

  vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
  axes_data->Update ();
  axes_data->GetPointData ()->SetScalars (axes_colors);            

  vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
  axes_tubes->SetInput (axes_data);
  axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
  axes_tubes->SetNumberOfSides (6);
  
  vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  axes_mapper->SetScalarModeToUsePointData ();
  axes_mapper->SetInput (axes_tubes->GetOutput ());
                                                                                            
  vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
  axes_actor->SetMapper (axes_mapper);
  axes_actor->SetPosition (x, y, z);

  addActorToRenderer (axes_actor, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::removeCoordinateSystem (int viewport)
{
/*  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
  axes->SetOrigin (0, 0, 0);
  axes->SetScaleFactor (scale);
                                                                                            
  vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
  axes_colors->Allocate (6);           
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (1.0);
  axes_colors->InsertNextValue (1.0);

  vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
  axes_data->Update ();
  axes_data->GetPointData ()->SetScalars (axes_colors);            

  vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
  axes_tubes->SetInput (axes_data);
  axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
  axes_tubes->SetNumberOfSides (6);
  
  vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  axes_mapper->SetScalarModeToUsePointData ();
  axes_mapper->SetInput (axes_tubes->GetOutput ());
                                                                                            
  vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
  axes_actor->SetMapper (axes_mapper);

  addActorToRenderer (axes_actor, viewport);*/
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::removePointCloud (const std::string &id, int viewport)
{
  // Check to see if the given ID entry exists
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);

  if (am_it == cloud_actor_map_.end ())
  {
    //terminal_tools::print_warn ("[removePointCloud] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }

  // Clear the handlers
  am_it->second.geometry_handlers.clear ();
  am_it->second.color_handlers.clear ();

  // Remove it from all renderers
  removeActorFromRenderer (am_it->second.actor, viewport);

  // Remove the pointer/ID pair to the global actor map
  cloud_actor_map_.erase (am_it);
  style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::removeShape (const std::string &id, int viewport)
{
  // Check to see if the given ID entry exists
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);

  if (am_it == shape_actor_map_.end ())
  {
    //terminal_tools::print_warn ("[removeSape] Could not find any shape with id <%s>!\n", id.c_str ());
    return (false);
  }

  // Remove it from all renderers
  removeActorFromRenderer (am_it->second, viewport);

  // Remove the pointer/ID pair to the global actor map
  shape_actor_map_.erase (am_it);
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addPointCloudPrincipalCurvatures (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                                                    const pcl::PointCloud<pcl::Normal> &normals,
                                                                    const pcl::PointCloud<pcl::PrincipalCurvatures> &pcs,
                                                                    int level, double scale,
                                                                    const std::string &id, int viewport)
{
  if (pcs.points.size () != cloud.points.size () || normals.points.size () != cloud.points.size ())
  {
    terminal_tools::print_error ("[addPointCloudPrincipalCurvatures] The number of points differs from the number of principal curvatures/normals!\n");
    return (false);
  }
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);

  if (am_it != cloud_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addPointCloudPrincipalCurvatures] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkAppendPolyData> polydata_1 = vtkSmartPointer<vtkAppendPolyData>::New ();
  vtkSmartPointer<vtkAppendPolyData> polydata_2 = vtkSmartPointer<vtkAppendPolyData>::New ();
 
  // Setup two colors - one for each line
  unsigned char green[3] = {0, 255, 0};
  unsigned char blue[3] = {0, 0, 255};
 
  // Setup the colors array
  vtkSmartPointer<vtkUnsignedCharArray> line_1_colors =vtkSmartPointer<vtkUnsignedCharArray>::New ();
  line_1_colors->SetNumberOfComponents (3);
  line_1_colors->SetName ("Colors"); 
  vtkSmartPointer<vtkUnsignedCharArray> line_2_colors =vtkSmartPointer<vtkUnsignedCharArray>::New ();
  line_2_colors->SetNumberOfComponents (3);
  line_2_colors->SetName ("Colors"); 

  // Create the first sets of lines
  for (size_t i = 0; i < cloud.points.size (); i+=level)
  {
    pcl::PointXYZ p = cloud.points[i];
    p.x += (pcs.points[i].pc1 * pcs.points[i].principal_curvature[0]) * scale;
    p.y += (pcs.points[i].pc1 * pcs.points[i].principal_curvature[1]) * scale;
    p.z += (pcs.points[i].pc1 * pcs.points[i].principal_curvature[2]) * scale;

    vtkSmartPointer<vtkLineSource> line_1 = vtkSmartPointer<vtkLineSource>::New ();
    line_1->SetPoint1 (cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    line_1->SetPoint2 (p.x, p.y, p.z);
    line_1->Update ();
    polydata_1->AddInput (line_1->GetOutput ());
    line_1_colors->InsertNextTupleValue (green);
  }
  polydata_1->Update (); 
  vtkSmartPointer<vtkPolyData> line_1_data = polydata_1->GetOutput ();
  line_1_data->GetCellData ()->SetScalars (line_1_colors);
  
  // Create the second sets of lines
  for (size_t i = 0; i < cloud.points.size (); i+=level)
  {
    Eigen::Vector3f pc (pcs.points[i].principal_curvature[0], pcs.points[i].principal_curvature[1], pcs.points[i].principal_curvature[2]);
    Eigen::Vector3f normal (normals.points[i].normal[0], normals.points[i].normal[1], normals.points[i].normal[2]); 
    Eigen::Vector3f pc_c = pc.cross (normal);

    pcl::PointXYZ p = cloud.points[i];
    p.x += (pcs.points[i].pc2 * pc_c[0]) * scale;
    p.y += (pcs.points[i].pc2 * pc_c[1]) * scale;
    p.z += (pcs.points[i].pc2 * pc_c[2]) * scale;

    vtkSmartPointer<vtkLineSource> line_2 = vtkSmartPointer<vtkLineSource>::New ();
    line_2->SetPoint1 (cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    line_2->SetPoint2 (p.x, p.y, p.z);
    line_2->Update ();
    polydata_2->AddInput (line_2->GetOutput ());
    line_2_colors->InsertNextTupleValue (blue);
  }
  polydata_2->Update (); 
  vtkSmartPointer<vtkPolyData> line_2_data = polydata_2->GetOutput ();
  line_2_data->GetCellData ()->SetScalars (line_2_colors);

  // Assemble the two sets of lines
  vtkSmartPointer<vtkAppendPolyData> alldata = vtkSmartPointer<vtkAppendPolyData>::New ();
  alldata->AddInput (line_1_data);
  alldata->AddInput (line_2_data);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (alldata->GetOutput (), actor);
  actor->GetMapper ()->SetScalarModeToUseCellData ();

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor act;
  //act.color_handlers.push_back (handler);
  act.actor = actor;
  cloud_actor_map_[id] = act;
  //style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addPointCloud (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                                 const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);

  if (am_it != cloud_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addPointCloud] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }
  vtkSmartPointer<vtkPolyData> polydata;
  
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (cloud, polydata);
  polydata->Update ();

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  PointCloudColorHandlerRandom<pcl::PointXYZ> handler (cloud);
  handler.getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);
 
  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor act;
  //act.color_handlers.push_back (handler);
  act.actor = actor;
  cloud_actor_map_[id] = act;
  //cloud_actor_map_[id] = actor;
  style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor, int viewport)
{
  // Add it to all renderers
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->RemoveActor (actor);
      renderer->Render ();
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->RemoveActor (actor);
      renderer->Render ();
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::addActorToRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
  // Add it to all renderers
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->AddActor (actor);
      renderer->Render ();
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->AddActor (actor);
      renderer->Render ();
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
  // Add it to all renderers
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->RemoveActor (actor);
      renderer->Render ();
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->RemoveActor (actor);
      renderer->Render ();
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, 
                                                               vtkSmartPointer<vtkLODActor> &actor)
{
  // If actor is not initialized, initialize it here
  if (!actor)
    actor = vtkSmartPointer<vtkLODActor>::New ();

  vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
  double minmax[2];
  if (scalars)
    scalars->GetRange (minmax);

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();      
  mapper->SetInput (data);
  if (scalars)
    mapper->SetScalarRange (minmax);
  mapper->SetScalarModeToUsePointData ();
  mapper->ScalarVisibilityOn ();

  actor->SetNumberOfCloudPoints (data->GetNumberOfPoints () / 10);
  actor->GetProperty ()->SetInterpolationToFlat ();

  actor->SetMapper (mapper);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::convertPointCloudToVTKPolyData (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                                                  vtkSmartPointer<vtkPolyData> &polydata)
{
  if (!polydata)
    polydata = vtkSmartPointer<vtkPolyData>::New ();

  // Create the supporting structures
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPoints> points      = vtkSmartPointer<vtkPoints>::New ();

  // Set the points
  points->SetDataTypeToFloat ();
  points->SetNumberOfPoints (cloud.points.size ());
  //
  double p[3];
  for (vtkIdType i = 0; i < (int)cloud.points.size (); ++i)
  {
    p[0] = cloud.points[i].x;
    p[1] = cloud.points[i].y;
    p[2] = cloud.points[i].z;
    points->SetPoint (i, p);
    vertices->InsertNextCell ((vtkIdType)1, &i);
  }
  polydata->SetPoints (points);
  polydata->SetVerts (vertices);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl_visualization::PCLVisualizer::convertPointCloudToVTKPolyData (const GeometryHandlerConstPtr &geometry_handler, vtkSmartPointer<vtkPolyData> &polydata)
{
  if (!polydata)
    polydata = vtkSmartPointer<vtkPolyData>::New ();

  // Create the supporting structures
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();

  // Use the handler to obtain the geometry
  vtkSmartPointer<vtkPoints> points;
  geometry_handler->getGeometry (points);
  
  // Set the vertices
  for (vtkIdType i = 0; i < (int)points->GetNumberOfPoints (); ++i)
    vertices->InsertNextCell ((vtkIdType)1, &i);
  polydata->SetPoints (points);
  polydata->SetVerts (vertices);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl_visualization::PCLVisualizer::setBackgroundColor (const double &r, const double &g, const double &b, int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->SetBackground (r, g, b);
      renderer->Render ();
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->SetBackground (r, g, b);
      renderer->Render ();
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLVisualizer::setPointCloudRenderingProperties (int property, double val1, double val2, double val3, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);

  if (am_it == cloud_actor_map_.end ())
  {
    terminal_tools::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

  switch (property)
  {
    case PCL_VISUALIZER_COLOR:
    {
      actor->GetProperty ()->SetColor (val1, val2, val3);
      actor->Modified ();
      break;
    }
    default:
    {
      terminal_tools::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLVisualizer::getPointCloudRenderingProperties (int property, double &value, const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);

  if (am_it == cloud_actor_map_.end ())
    return (false);
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

  switch (property)
  {
    case PCL_VISUALIZER_POINT_SIZE:
    {
      value = actor->GetProperty ()->GetPointSize ();
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_OPACITY:
    {
      value = actor->GetProperty ()->GetOpacity ();
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_LINE_WIDTH:
    {
      value = actor->GetProperty ()->GetLineWidth ();
      actor->Modified ();
      break;
    }
    default:
    {
      terminal_tools::print_error ("[getPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLVisualizer::setPointCloudRenderingProperties (int property, double value, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);

  if (am_it == cloud_actor_map_.end ())
  {
    terminal_tools::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);

  switch (property)
  {
    case PCL_VISUALIZER_POINT_SIZE:
    {
      actor->GetProperty ()->SetPointSize (value);
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_OPACITY:
    {
      actor->GetProperty ()->SetOpacity (value);
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_LINE_WIDTH:
    {
      actor->GetProperty ()->SetLineWidth (value);
      actor->Modified ();
      break;
    }
    default:
    {
      terminal_tools::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLVisualizer::setShapeRenderingProperties (int property, double value, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);

  if (am_it == shape_actor_map_.end ())
  {
    terminal_tools::print_error ("[setShapeRenderingProperties] Could not find any shape with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);

  switch (property)
  {
    case PCL_VISUALIZER_OPACITY:
    {
      actor->GetProperty ()->SetOpacity (value);
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_LINE_WIDTH:
    {
      actor->GetProperty ()->SetLineWidth (value);
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_FONT_SIZE:
    {
      vtkTextActor* text_actor = vtkTextActor::SafeDownCast (am_it->second);
      vtkSmartPointer<vtkTextProperty> tprop = text_actor->GetTextProperty ();
      tprop->SetFontSize (value);
      text_actor->Modified ();
      break;
    }
    default:
    {
      terminal_tools::print_error ("[setShapeRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::initCameraParameters ()
{
  // Set default camera parameters
  camera_.clip[0] = 0.01; camera_.clip[1] = 1000.01;
  camera_.focal[0] = camera_.focal[1] = camera_.focal[2] = 0;
  camera_.pos[0] = camera_.pos[1] = 0; camera_.pos[2] = 1;
  camera_.view[0] = camera_.view[2] = 0; camera_.view[1] = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::updateCamera ()
{
   // Update the camera parameters
   rens_->InitTraversal ();
   vtkRenderer* renderer = NULL;
   while ((renderer = rens_->GetNextItem ()) != NULL)
   {
     renderer->GetActiveCamera ()->SetPosition (camera_.pos);
     renderer->GetActiveCamera ()->SetFocalPoint (camera_.focal);
     renderer->GetActiveCamera ()->SetViewUp (camera_.view);
     renderer->GetActiveCamera ()->SetClippingRange (camera_.clip);
   }
}
/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::PCLVisualizer::resetCamera ()
{
   // Update the camera parameters
   rens_->InitTraversal ();
   vtkRenderer* renderer = NULL;
   while ((renderer = rens_->GetNextItem ()) != NULL)
   {
     renderer->ResetCamera ();
   }
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::getCameraParameters (int argc, char **argv)
{
  for (int i = 1; i < argc; i++)
  {
    if ((strcmp (argv[i], "-cam") == 0) && (++i < argc))
    {
      std::ifstream fs;
      std::string camfile = std::string (argv[i]);
      std::string line;
      
      std::vector<std::string> camera;
      if (camfile.find (".cam") == std::string::npos)
      {
        // Assume we have clip/focal/pos/view
        boost::split (camera, argv[i], boost::is_any_of ("/"), boost::token_compress_on);
      }
      else
      {
        // Assume that if we don't have clip/focal/pos/view, a filename.cam was given as a parameter
        fs.open (camfile.c_str ());
        while (!fs.eof ())
        {
          getline (fs, line);
          if (line == "")
            continue;

          boost::split (camera, line, boost::is_any_of ("/"), boost::token_compress_on);
          break;
        }
        fs.close ();
      }
      
      // look for '/' as a separator
      if (camera.size () != 6)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Camera parameters given, but with an invalid number of options (%zu vs 6)!\n", camera.size ());
        return (false);
      }

      std::string clip_str  = camera.at (0);
      std::string focal_str = camera.at (1);
      std::string pos_str   = camera.at (2);
      std::string view_str  = camera.at (3);
      std::string win_size_str = camera.at (4);
      std::string win_pos_str  = camera.at (5);
      
      // Get each camera setting separately and parse for ','
      std::vector<std::string> clip_st;
      boost::split (clip_st, clip_str, boost::is_any_of (","), boost::token_compress_on);
      if (clip_st.size () != 2)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera clipping angle!\n");
        return (false);
      }
      camera_.clip[0] = atof (clip_st.at (0).c_str ());
      camera_.clip[1] = atof (clip_st.at (1).c_str ());

      std::vector<std::string> focal_st;
      boost::split (focal_st, focal_str, boost::is_any_of (","), boost::token_compress_on);
      if (focal_st.size () != 3)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera focal point!\n");
        return (false);
      }
      camera_.focal[0] = atof (focal_st.at (0).c_str ());
      camera_.focal[1] = atof (focal_st.at (1).c_str ());
      camera_.focal[2] = atof (focal_st.at (2).c_str ());

      std::vector<std::string> pos_st;
      boost::split (pos_st, pos_str, boost::is_any_of (","), boost::token_compress_on);
      if (pos_st.size () != 3)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera position!\n");
        return (false);
      }
      camera_.pos[0] = atof (pos_st.at (0).c_str ());
      camera_.pos[1] = atof (pos_st.at (1).c_str ());
      camera_.pos[2] = atof (pos_st.at (2).c_str ());

      std::vector<std::string> view_st;
      boost::split (view_st, view_str, boost::is_any_of (","), boost::token_compress_on);
      if (view_st.size () != 3)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera viewup!\n");
        return (false);
      }
      camera_.view[0] = atof (view_st.at (0).c_str ());
      camera_.view[1] = atof (view_st.at (1).c_str ());
      camera_.view[2] = atof (view_st.at (2).c_str ());

      std::vector<std::string> win_size_st;
      boost::split (win_size_st, win_size_str, boost::is_any_of (","), boost::token_compress_on);
      if (win_size_st.size () != 2)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for window size!\n");
        return (false);
      }
      camera_.window_size[0] = atof (win_size_st.at (0).c_str ());
      camera_.window_size[1] = atof (win_size_st.at (1).c_str ());

      std::vector<std::string> win_pos_st;
      boost::split (win_pos_st, win_pos_str, boost::is_any_of (","), boost::token_compress_on);
      if (win_pos_st.size () != 2)
      {
        terminal_tools::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for window position!\n");
        return (false);
      }
      camera_.window_pos[0] = atof (win_pos_st.at (0).c_str ());
      camera_.window_pos[1] = atof (win_pos_st.at (1).c_str ());

      return (true);
    }
  }
  return (false);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addCylinder (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addCylinder] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCylinder (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addSphere (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createSphere (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addModelFromPLYFile (const std::string &filename, const std::string &id, int viewport)
{
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn (
                                "[addModelFromPLYFile] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (filename.c_str ());

  // Create an Actor
  vtkSmartPointer < vtkLODActor > actor;
  createActorFromVTKDataSet (reader->GetOutput (), actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addModelFromPLYFile (const std::string &filename,
                                                       vtkSmartPointer<vtkTransform> transform, const std::string &id,
                                                       int viewport)
{
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn (
                                "[addModelFromPLYFile] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (filename.c_str ());

  //create transformation filter
  vtkSmartPointer < vtkTransformFilter > trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter->SetTransform (transform);
  trans_filter->SetInputConnection (reader->GetOutputPort ());

  // Create an Actor
  vtkSmartPointer < vtkLODActor > actor;
  createActorFromVTKDataSet (trans_filter->GetOutput (), actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addLine (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addLine] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createLine (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Add a plane from a set of given model coefficients 
  * \param coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
  * \param id the plane id/name (default: "plane")
  * \param viewport (optional) the id of the new viewport (default: 0)
  */
bool
  pcl_visualization::PCLVisualizer::addPlane (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addPlane] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createPlane (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addCircle (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addCircle] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = create2DCircle (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addCone (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addCone] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCone (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl_visualization::PCLVisualizer::createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport)
{
  // Create a new renderer
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
  ren->SetViewport (xmin, ymin, xmax, ymax);

  if (rens_->GetNumberOfItems () > 0)
    ren->SetActiveCamera (rens_->GetFirstRenderer ()->GetActiveCamera ());
  ren->ResetCamera ();

  // Add it to the list of renderers
  rens_->AddItem (ren);

  if (rens_->GetNumberOfItems () <= 1)          // If only one renderer
    viewport = 0;                               // set viewport to 'all'
  else
    viewport = rens_->GetNumberOfItems ();

  win_->AddRenderer (ren);
  win_->Modified ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLVisualizer::addText (const std::string &text, int xpos, int ypos, const std::string &id, int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (tid);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addText] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkTextActor> actor = vtkSmartPointer<vtkTextActor>::New ();
  actor->SetPosition (xpos, ypos);
  actor->SetInput (text.c_str ());
  
  vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty ();
  tprop->SetFontSize (10);
  tprop->SetFontFamilyToArial ();
  tprop->SetJustificationToLeft ();
  tprop->BoldOn ();
  tprop->SetColor (1, 1, 1);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[tid] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool 
pcl_visualization::PCLVisualizer::addText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id, int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (tid);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn ("[addText] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkTextActor> actor = vtkSmartPointer<vtkTextActor>::New ();
  actor->SetPosition (xpos, ypos);
  actor->SetInput (text.c_str ());
  
  vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty ();
  tprop->SetFontSize (10);
  tprop->SetFontFamilyToArial ();
  tprop->SetJustificationToLeft ();
  tprop->BoldOn ();
  tprop->SetColor (r, g, b);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[tid] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::updateColorHandlerIndex (const std::string &id, int index)
{
  CloudActorMap::iterator am_it = cloud_actor_map_.find (id);
  if (am_it == cloud_actor_map_.end ())
  {
    terminal_tools::print_warn ("[updateColorHandlerIndex] PointCloud with id <%s> doesn't exist!\n", id.c_str ());
    return (false);
  }

  if (index >= (int)am_it->second.color_handlers.size ())
  {
    terminal_tools::print_warn ("[updateColorHandlerIndex] Invalid index <%d> given! Maximum range is: 0-%zu.\n", index, am_it->second.color_handlers.size ());
    return (false);
  }
  // Get the handler
  PointCloudColorHandler<sensor_msgs::PointCloud2>::ConstPtr color_handler = am_it->second.color_handlers[index];

  vtkSmartPointer<vtkDataArray> scalars;
  color_handler->getColor (scalars);
  double minmax[2];
  scalars->GetRange (minmax);
  // Update the data
  vtkPolyData *data = static_cast<vtkPolyData*>(am_it->second.actor->GetMapper ()->GetInput ());
  data->GetPointData ()->SetScalars (scalars);
  data->Update ();
  // Modify the mapper
  vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ());
  mapper->SetScalarRange (minmax);
  mapper->SetScalarModeToUsePointData ();
  mapper->SetInput (data);
  // Modify the actor
  am_it->second.actor->SetMapper (mapper);
  am_it->second.actor->Modified ();
  am_it->second.color_handler_index_ = index;
  
  style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));

  return (true);
}



/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl_visualization::PCLVisualizer::addPolygonMesh (const pcl::PolygonMesh &poly_mesh, 
                                                  const std::string &id,
                                                  int viewport)
{
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    terminal_tools::print_warn (
                                "[addPolygonMesh] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  // Create points from polyMesh.cloud
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromROSMsg(poly_mesh.cloud, point_cloud);
  poly_points->SetNumberOfPoints (point_cloud.points.size ());

  size_t i;
  for (i = 0; i < point_cloud.points.size (); ++i)
    poly_points->InsertPoint (i, point_cloud.points[i].x, point_cloud.points[i].y, point_cloud.points[i].z);

  vtkSmartPointer<vtkLODActor> actor;
  if (poly_mesh.polygons.size() > 1) 
  {
    //create polys from polyMesh.polygons
    vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();

    for (i = 0; i < poly_mesh.polygons.size (); i++) 
    {
      size_t n_points = poly_mesh.polygons[i].vertices.size ();
      cell_array->InsertNextCell (n_points);
      for (size_t j = 0; j < n_points; j++) 
        cell_array->InsertCellPoint (poly_mesh.polygons[i].vertices[j]);
    }

    vtkPolyData* polydata = vtkPolyData::New ();
    polydata->SetStrips (cell_array);
    polydata->SetPoints (poly_points);

    createActorFromVTKDataSet (polydata, actor);
  } 
  else 
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
    size_t n_points = poly_mesh.polygons[0].vertices.size ();
    polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

    for (size_t j=0; j < (n_points - 1); j++) 
      polygon->GetPointIds ()->SetId (j, poly_mesh.polygons[0].vertices[j]);

    vtkSmartPointer<vtkUnstructuredGrid> poly_grid = vtkSmartPointer<vtkUnstructuredGrid>::New ();
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (poly_points);
    poly_grid->Update ();

    createActorFromVTKDataSet (poly_grid, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
  }

  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_visualization::FPSCallback::Execute (vtkObject *caller, unsigned long, void*)
{
  vtkRenderer *ren = reinterpret_cast<vtkRenderer *> (caller);
  float fps = 1.0 / ren->GetLastRenderTimeInSeconds ();
  char buf[128];
  sprintf (buf, "%.1f FPS", fps);
  actor_->SetInput (buf);
}

