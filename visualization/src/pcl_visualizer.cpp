/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#include <pcl/visualization/common/common.h>
#include <pcl/conversions.h>
#include <vtkVersion.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCellData.h>
#include <vtkWorldPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkTriangle.h>
#include <vtkTransform.h>
#include <vtkPolyDataNormals.h>
#include <vtkMapper.h>
#include <vtkDataSetMapper.h>
#include <vtkCellArray.h>
#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkPointPicker.h>

#include <pcl/visualization/vtk/vtkRenderWindowInteractorFix.h>
#include <pcl/visualization/vtk/pcl_vtk_compatibility.h>

#include <vtkPolyLine.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>
#include <vtkAppendPolyData.h>
#include <vtkPointData.h>
#include <vtkTransformFilter.h>
#include <vtkProperty.h>
#include <vtkPLYReader.h>
#include <vtkAxes.h>
#include <vtkTubeFilter.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAreaPicker.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <vtkPNMReader.h>
#include <vtkPNGReader.h>
#include <vtkTIFFReader.h>
#include <vtkLookupTable.h>
#include <vtkTextureUnitManager.h>

#if VTK_MAJOR_VERSION > 7
#include <vtkTexture.h>
#include <vtkRenderStepsPass.h>
#include <vtkEDLShading.h>
#endif

#include <pcl/visualization/common/shapes.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/common/time.h>
#include <boost/version.hpp> // for BOOST_VERSION
#if (BOOST_VERSION >= 106600)
#include <boost/uuid/detail/sha1.hpp>
#else
#include <boost/uuid/sha1.hpp>
#endif
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp> // for split
#include <pcl/common/utils.h> // pcl::utils::ignore
#include <pcl/console/parse.h>

#include <thread>

// Support for VTK 7.1 upwards
#ifdef vtkGenericDataArray_h
#define SetTupleValue SetTypedTuple
#define InsertNextTupleValue InsertNextTypedTuple
#define GetTupleValue GetTypedTuple
#endif

#if defined(_WIN32)
  // Remove macros defined in Windows.h
  #undef near
  #undef far
#endif

vtkIdType
pcl::visualization::details::fillCells(std::vector<int>& lookup, const std::vector<pcl::Vertices>& vertices, vtkSmartPointer<vtkCellArray> cell_array, int max_size_of_polygon)
{
#ifdef VTK_CELL_ARRAY_V2
  pcl::utils::ignore(max_size_of_polygon);

  if (!lookup.empty())
  {
    for (const auto& verti : vertices)
    {
      std::size_t n_points = verti.vertices.size();
      cell_array->InsertNextCell(n_points);
      for (const auto& vertj : verti.vertices)
        cell_array->InsertCellPoint(lookup[vertj]);
    }
  }
  else
  {
    for (const auto& verti : vertices)
    {
      std::size_t n_points = verti.vertices.size();
      cell_array->InsertNextCell(n_points);
      for (const auto& vertj : verti.vertices)
        cell_array->InsertCellPoint(vertj);
    }
  }
#else
  vtkIdType* cell = cell_array->WritePointer(vertices.size(), vertices.size() * (max_size_of_polygon + 1));

  if (!lookup.empty())
  {
    for (const auto& verti : vertices)
    {
      std::size_t n_points = verti.vertices.size();
      *cell++ = n_points;
      for (const auto& vertj : verti.vertices)
        *cell++ = lookup[vertj];
    }
  }
  else
  {
    for (const auto& verti : vertices)
    {
      std::size_t n_points = verti.vertices.size();
      *cell++ = n_points;
      for (const auto& vertj : verti.vertices)
        *cell++ = vertj;
    }
  }
#endif

  const auto idx = vertices.size() + std::accumulate(vertices.begin(), vertices.end(), static_cast<vtkIdType>(0),
    [](const auto& sum, const auto& vertex) { return sum + vertex.vertices.size(); });

  return idx;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::PCLVisualizer (const std::string &name, const bool create_interactor)
  : update_fps_ (vtkSmartPointer<FPSCallback>::New ())
  , stopped_ ()
  , timer_id_ ()
  , rens_ (vtkSmartPointer<vtkRendererCollection>::New ())
  , win_ (vtkSmartPointer<vtkRenderWindow>::New ())
  , style_ (vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>::New ())
  , cloud_actor_map_ (new CloudActorMap)
  , shape_actor_map_ (new ShapeActorMap)
  , coordinate_actor_map_ (new CoordinateActorMap)
  , camera_set_ ()
  , camera_file_loaded_ (false)
{
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
  setupRenderer (ren);
  setupFPSCallback (ren);
  setupRenderWindow (name);
  setDefaultWindowSizeAndPos ();
  setupStyle ();

  if (create_interactor)
    createInteractor ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::PCLVisualizer (int &argc, char **argv, const std::string &name, PCLVisualizerInteractorStyle* style, const bool create_interactor)
  : update_fps_ (vtkSmartPointer<FPSCallback>::New ())
  , stopped_ ()
  , timer_id_ ()
  , rens_ (vtkSmartPointer<vtkRendererCollection>::New ())
  , win_ (vtkSmartPointer<vtkRenderWindow>::New ())
  , style_ (style)
  , cloud_actor_map_ (new CloudActorMap)
  , shape_actor_map_ (new ShapeActorMap)
  , coordinate_actor_map_ (new CoordinateActorMap)
  , camera_set_ ()
  , camera_file_loaded_ (false)
{
  vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New ();
  setupRenderer (ren);
  setupFPSCallback (ren);
  setupRenderWindow (name);
  setupStyle ();
  setupCamera (argc, argv);

  if(!camera_set_ && !camera_file_loaded_)
    setDefaultWindowSizeAndPos ();

  if (create_interactor)
    createInteractor ();

  //window name should be reset due to its reset somewhere in camera initialization
  win_->SetWindowName (name.c_str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::PCLVisualizer (vtkSmartPointer<vtkRenderer> ren, vtkSmartPointer<vtkRenderWindow> wind,
                                                  const std::string &name, const bool create_interactor)
  : update_fps_ (vtkSmartPointer<FPSCallback>::New ())
  , stopped_ ()
  , timer_id_ ()
  , rens_ (vtkSmartPointer<vtkRendererCollection>::New ())
  , win_ (wind)
  , style_ (vtkSmartPointer<pcl::visualization::PCLVisualizerInteractorStyle>::New ())
  , cloud_actor_map_ (new CloudActorMap)
  , shape_actor_map_ (new ShapeActorMap)
  , coordinate_actor_map_ (new CoordinateActorMap)
  , camera_set_ ()
  , camera_file_loaded_ (false)
{
  setupRenderer (ren);
  setupFPSCallback (ren);
  setupRenderWindow (name);
  setDefaultWindowSizeAndPos ();
  setupStyle ();

  if (create_interactor)
    createInteractor ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::PCLVisualizer (int &argc, char **argv, vtkSmartPointer<vtkRenderer> ren, vtkSmartPointer<vtkRenderWindow> wind,
                                                  const std::string &name, PCLVisualizerInteractorStyle* style, const bool create_interactor)
  : update_fps_ (vtkSmartPointer<FPSCallback>::New ())
  , stopped_ ()
  , timer_id_ ()
  , rens_ (vtkSmartPointer<vtkRendererCollection>::New ())
  , win_ (wind)
  , style_ (style)
  , cloud_actor_map_ (new CloudActorMap)
  , shape_actor_map_ (new ShapeActorMap)
  , coordinate_actor_map_ (new CoordinateActorMap)
  , camera_set_ ()
  , camera_file_loaded_ (false)
{
  setupRenderer (ren);
  setupFPSCallback (ren);
  setupRenderWindow (name);
  setupStyle ();
  setupCamera (argc, argv);
  if (!camera_set_ && !camera_file_loaded_)
    setDefaultWindowSizeAndPos ();
  if (create_interactor)
    createInteractor ();

  //window name should be reset due to its reset somewhere in camera initialization
  win_->SetWindowName (name.c_str ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::createInteractor ()
{
  //interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New ();
  interactor_ = vtkSmartPointer <vtkRenderWindowInteractor>::Take (vtkRenderWindowInteractorFixNew ());

  //win_->PointSmoothingOn ();
  //win_->LineSmoothingOn ();
  //win_->PolygonSmoothingOn ();
  win_->AlphaBitPlanesOff ();
  win_->PointSmoothingOff ();
  win_->LineSmoothingOff ();
  win_->PolygonSmoothingOff ();
  win_->SwapBuffersOn ();
  win_->SetStereoTypeToAnaglyph ();

  interactor_->SetRenderWindow (win_);
  interactor_->SetInteractorStyle (style_);
  //interactor_->SetStillUpdateRate (30.0);
  interactor_->SetDesiredUpdateRate (30.0);

  // Initialize and create timer, also create window
  interactor_->Initialize ();
  timer_id_ = interactor_->CreateRepeatingTimer (5000L);

  // Set a simple PointPicker
  vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
  pp->SetTolerance (pp->GetTolerance () * 2);
  interactor_->SetPicker (pp);

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
void
pcl::visualization::PCLVisualizer::setupInteractor (
  vtkRenderWindowInteractor *iren,
  vtkRenderWindow *win)
{
  win->AlphaBitPlanesOff ();
  win->PointSmoothingOff ();
  win->LineSmoothingOff ();
  win->PolygonSmoothingOff ();
  win->SwapBuffersOn ();
  win->SetStereoTypeToAnaglyph ();

  iren->SetRenderWindow (win);
  iren->SetInteractorStyle (style_);
  //iren->SetStillUpdateRate (30.0);
  iren->SetDesiredUpdateRate (30.0);

  // Initialize and create timer, also create window
  iren->Initialize ();

  // Set a simple PointPicker
  vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
  pp->SetTolerance (pp->GetTolerance () * 2);
  iren->SetPicker (pp);
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setupInteractor (
  vtkRenderWindowInteractor *iren,
  vtkRenderWindow *win,
  vtkInteractorStyle *style)
{
  win->AlphaBitPlanesOff ();
  win->PointSmoothingOff ();
  win->LineSmoothingOff ();
  win->PolygonSmoothingOff ();
  win->SwapBuffersOn ();
  win->SetStereoTypeToAnaglyph ();

  iren->SetRenderWindow (win);
  iren->SetInteractorStyle (style);
  //iren->SetStillUpdateRate (30.0);
  iren->SetDesiredUpdateRate (30.0);

  // Initialize and create timer, also create window
  iren->Initialize ();

  // Set a simple PointPicker
  // vtkSmartPointer<vtkPointPicker> pp = vtkSmartPointer<vtkPointPicker>::New ();
  // pp->SetTolerance (pp->GetTolerance () * 2);
  // iren->SetPicker (pp);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setupRenderer (vtkSmartPointer<vtkRenderer> ren)
{
  if (!ren)
    PCL_ERROR ("Passed pointer to renderer is null\n");

  ren->AddObserver (vtkCommand::EndEvent, update_fps_);
  // Add it to the list of renderers
  rens_->AddItem (ren);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setupFPSCallback (const vtkSmartPointer<vtkRenderer>& ren)
{
  if (!ren)
    PCL_ERROR ("Passed pointer to renderer is null\n");
  // FPS callback
  vtkSmartPointer<vtkTextActor> txt = vtkSmartPointer<vtkTextActor>::New ();
  update_fps_->actor = txt;
  update_fps_->pcl_visualizer = this;
  update_fps_->decimated = false;
  ren->AddActor (txt);
  txt->SetInput ("0 FPS");
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setupRenderWindow (const std::string& name)
{
  if (!win_)
    PCL_ERROR ("Pointer to render window is null\n");

  win_->SetWindowName (name.c_str ());

  // By default, don't use vertex buffer objects
  use_vbos_ = false;

  // Add all renderers to the window
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
    win_->AddRenderer (renderer);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setupStyle ()
{
  if (!style_)
    PCL_ERROR ("Pointer to style is null\n");

  // Set rend erer window in case no interactor is created
  style_->setRenderWindow (win_);

  // Create the interactor style
  style_->Initialize ();
  style_->setRendererCollection (rens_);
  style_->setCloudActorMap (cloud_actor_map_);
  style_->setShapeActorMap (shape_actor_map_);
  style_->UseTimersOn ();
  style_->setUseVbos (use_vbos_);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setDefaultWindowSizeAndPos ()
{
  if (!win_)
    PCL_ERROR ("Pointer to render window is null\n");
  int scr_size_x = win_->GetScreenSize ()[0];
  int scr_size_y = win_->GetScreenSize ()[1];
  win_->SetSize (scr_size_x / 2, scr_size_y / 2);
  win_->SetPosition (0, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void pcl::visualization::PCLVisualizer::setupCamera (int argc, char **argv)
{
  initCameraParameters ();

  // Parse the camera settings and update the internal camera
  camera_set_ = getCameraParameters (argc, argv);
  // Calculate unique camera filename for camera parameter saving/restoring
  if (!camera_set_)
  {
    std::string camera_file = getUniqueCameraFile (argc, argv);
    if (!camera_file.empty ())
    {
      if (boost::filesystem::exists (camera_file) && style_->loadCameraParameters (camera_file))
      {
        camera_file_loaded_ = true;
      }
      else
      {
        style_->setCameraFile (camera_file);
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::PCLVisualizer::~PCLVisualizer ()
{
  if (interactor_)
    interactor_->DestroyTimer (timer_id_);
  // Clear the collections
  rens_->RemoveAllItems ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::saveScreenshot (const std::string &file)
{
  style_->saveScreenshot (file);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::saveCameraParameters (const std::string &file)
{
  style_->saveCameraParameters (file);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::getCameraParameters (pcl::visualization::Camera &camera, int viewport) const
{
  style_->getCameraParameters (camera, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizer::registerKeyboardCallback (std::function<void (const pcl::visualization::KeyboardEvent&)> callback)
{
  return (style_->registerKeyboardCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizer::registerMouseCallback (std::function<void (const pcl::visualization::MouseEvent&)> callback)
{
  return (style_->registerMouseCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizer::registerPointPickingCallback (std::function<void (const pcl::visualization::PointPickingEvent&)> callback)
{
  return (style_->registerPointPickingCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizer::registerPointPickingCallback (void (*callback) (const pcl::visualization::PointPickingEvent&, void*), void* cookie)
{
  return (registerPointPickingCallback ([=] (const pcl::visualization::PointPickingEvent& e) { (*callback) (e, cookie); }));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizer::registerAreaPickingCallback (std::function<void (const pcl::visualization::AreaPickingEvent&)> callback)
{
  return (style_->registerAreaPickingCallback (callback));
}

/////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizer::registerAreaPickingCallback (void (*callback) (const pcl::visualization::AreaPickingEvent&, void*), void* cookie)
{
  return (registerAreaPickingCallback ([=] (const pcl::visualization::AreaPickingEvent& e) { (*callback) (e, cookie); }));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::spin ()
{
  resetStoppedFlag ();
  // Render the window before we start the interactor
  win_->Render ();
  if (interactor_)
    interactor_->Start ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::spinOnce (int time, bool force_redraw)
{
  resetStoppedFlag ();

#if VTK_MAJOR_VERSION < 9 && defined (__APPLE__)
  if (!win_->IsDrawable ())
  {
    close ();
    return;
  }
#endif

  if (!interactor_)
    return;

  if (time <= 0)
    time = 1;

  if (force_redraw)
    interactor_->Render ();

#if VTK_MAJOR_VERSION >= 9 && (VTK_MINOR_VERSION != 0 || VTK_BUILD_VERSION != 0) && (VTK_MINOR_VERSION != 0 || VTK_BUILD_VERSION != 1)
// All VTK 9 versions, except 9.0.0 and 9.0.1
  if(interactor_->IsA("vtkXRenderWindowInteractor")) {
    DO_EVERY (1.0 / interactor_->GetDesiredUpdateRate (),
      interactor_->ProcessEvents ();
      std::this_thread::sleep_for (std::chrono::milliseconds (time));
    );
  }
  else
#endif
  {
    DO_EVERY (1.0 / interactor_->GetDesiredUpdateRate (),
      exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer (time);
      interactor_->Start ();
      interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
    );
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::addOrientationMarkerWidgetAxes (vtkRenderWindowInteractor* interactor)
{
  if ( !axes_widget_ )
  {
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New ();

    axes_widget_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New ();
    axes_widget_->SetOutlineColor (0.9300, 0.5700, 0.1300);
    axes_widget_->SetOrientationMarker (axes);
    axes_widget_->SetInteractor (interactor);
    axes_widget_->SetViewport (0.0, 0.0, 0.4, 0.4);
    axes_widget_->SetEnabled (true);
    axes_widget_->InteractiveOn ();
  }
  else
  {
    axes_widget_->SetEnabled (true);
    pcl::console::print_warn (stderr, "Orientation Widget Axes already exists, just enabling it");
  }

}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::removeOrientationMarkerWidgetAxes ()
{
  if (axes_widget_)
  {
    if (axes_widget_->GetEnabled ())
      axes_widget_->SetEnabled (false);
    else
      pcl::console::print_warn (stderr, "Orientation Widget Axes was already disabled, doing nothing.");
  }
  else
  {
    pcl::console::print_error ("Attempted to delete Orientation Widget Axes which does not exist!\n");
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::addCoordinateSystem (double scale, const std::string &id, int viewport)
{
  if (scale <= 0.0)
    scale = 1.0;
  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
  axes->SetOrigin (0, 0, 0);
  axes->SetScaleFactor (scale);
  axes->Update ();

  vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
  axes_colors->Allocate (6);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (1.0);
  axes_colors->InsertNextValue (1.0);

  vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
  axes_data->GetPointData ()->SetScalars (axes_colors);

  vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
  axes_tubes->SetInputData (axes_data);
  axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
  axes_tubes->SetNumberOfSides (6);

  vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  axes_mapper->SetScalarModeToUsePointData ();
  axes_mapper->SetInputConnection (axes_tubes->GetOutputPort ());

  vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
  axes_actor->SetMapper (axes_mapper);

  // Save the ID and actor pair to the global actor map
  (*coordinate_actor_map_) [id] = axes_actor;

  addActorToRenderer (axes_actor, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::addCoordinateSystem (double scale, float x, float y, float z, const std::string& id, int viewport)
{
  if (scale <= 0.0)
    scale = 1.0;
  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
  axes->SetOrigin (0, 0, 0);
  axes->SetScaleFactor (scale);
  axes->Update ();

  vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
  axes_colors->Allocate (6);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (1.0);
  axes_colors->InsertNextValue (1.0);

  vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
  axes_data->GetPointData ()->SetScalars (axes_colors);

  vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
  axes_tubes->SetInputData (axes_data);
  axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
  axes_tubes->SetNumberOfSides (6);

  vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  axes_mapper->SetScalarModeToUsePointData ();
  axes_mapper->SetInputConnection (axes_tubes->GetOutputPort ());

  vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
  axes_actor->SetMapper (axes_mapper);
  axes_actor->SetPosition (x, y, z);

  // Save the ID and actor pair to the global actor map
  (*coordinate_actor_map_) [id] = axes_actor;

  addActorToRenderer (axes_actor, viewport);
}

int
feq (double a, double b) {
    return std::abs (a - b) < 1e-9;
}

void
quat_to_angle_axis (const Eigen::Quaternionf &qx, double &theta, double axis[3])
{
double q[4];
  q[0] = qx.w();
  q[1] = qx.x();
  q[2] = qx.y();
  q[3] = qx.z();

    double halftheta = std::acos (q[0]);
    theta = halftheta * 2;
    double sinhalftheta = sin (halftheta);
    if (feq (halftheta, 0)) {
        axis[0] = 0;
        axis[1] = 0;
        axis[2] = 1;
        theta = 0;
    } else {
        axis[0] = q[1] / sinhalftheta;
        axis[1] = q[2] / sinhalftheta;
        axis[2] = q[3] / sinhalftheta;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::addCoordinateSystem (double scale, const Eigen::Affine3f& t, const std::string& id, int viewport)
{
  if (scale <= 0.0)
    scale = 1.0;
  vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New ();
  axes->SetOrigin (0, 0, 0);
  axes->SetScaleFactor (scale);
  axes->Update ();

  vtkSmartPointer<vtkFloatArray> axes_colors = vtkSmartPointer<vtkFloatArray>::New ();
  axes_colors->Allocate (6);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.0);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (0.5);
  axes_colors->InsertNextValue (1.0);
  axes_colors->InsertNextValue (1.0);

  vtkSmartPointer<vtkPolyData> axes_data = axes->GetOutput ();
  axes_data->GetPointData ()->SetScalars (axes_colors);

  vtkSmartPointer<vtkTubeFilter> axes_tubes = vtkSmartPointer<vtkTubeFilter>::New ();
  axes_tubes->SetInputData (axes_data);
  axes_tubes->SetRadius (axes->GetScaleFactor () / 50.0);
  axes_tubes->SetNumberOfSides (6);

  vtkSmartPointer<vtkPolyDataMapper> axes_mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  axes_mapper->SetScalarModeToUsePointData ();
  axes_mapper->SetInputConnection (axes_tubes->GetOutputPort ());

  vtkSmartPointer<vtkLODActor> axes_actor = vtkSmartPointer<vtkLODActor>::New ();
  axes_actor->SetMapper (axes_mapper);

  axes_actor->SetPosition (t (0, 3), t(1, 3), t(2, 3));

  Eigen::Matrix3f m;
  m =t.rotation();
  Eigen::Quaternionf rf;
  rf = Eigen::Quaternionf(m);
  double r_angle;
  double r_axis[3];
  quat_to_angle_axis(rf,r_angle,r_axis);
  //
  axes_actor->SetOrientation(0,0,0);
  axes_actor->RotateWXYZ(r_angle*180/M_PI,r_axis[0],r_axis[1],r_axis[2]);
  //WAS:  axes_actor->SetOrientation (roll, pitch, yaw);

  // Save the ID and actor pair to the global actor map
  (*coordinate_actor_map_) [id] = axes_actor;
  addActorToRenderer (axes_actor, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeCoordinateSystem (const std::string& id, int viewport)
{
  // Check to see if the given ID entry exists
  auto am_it = coordinate_actor_map_->find (id);

  if (am_it == coordinate_actor_map_->end ())
    return (false);

  // Remove it from all renderers
  if (removeActorFromRenderer (am_it->second, viewport))
  {
    // Remove the ID pair to the global actor map
    coordinate_actor_map_->erase (am_it);
    return (true);
  }
  return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removePointCloud (const std::string &id, int viewport)
{
  // Check to see if the given ID entry exists
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  // Remove it from all renderers
  if (removeActorFromRenderer (am_it->second.actor, viewport))
  {
    // Remove the pointer/ID pair to the global actor map
    cloud_actor_map_->erase (am_it);
    return (true);
  }
  return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeShape (const std::string &id, int viewport)
{
  // Check to see if the given ID entry exists
  auto am_it = shape_actor_map_->find (id);
  // Extra step: check if there is a cloud with the same ID
  auto ca_it = cloud_actor_map_->find (id);

  bool shape = true;
  // Try to find a shape first
  if (am_it == shape_actor_map_->end ())
  {
    // There is no cloud or shape with this ID, so just exit
    if (ca_it == cloud_actor_map_->end ())
      return (false);
    // Cloud found, set shape to false
    shape = false;
  }

  // Remove the pointer/ID pair to the global actor map
  if (shape)
  {
    if (removeActorFromRenderer (am_it->second, viewport))
    {
      bool update_LUT (true);
      if (!style_->lut_actor_id_.empty() && am_it->first != style_->lut_actor_id_)
        update_LUT = false;
      shape_actor_map_->erase (am_it);
      if (update_LUT)
        style_->updateLookUpTableDisplay (false);
      return (true);
    }
  }
  else
  {
    if (removeActorFromRenderer (ca_it->second.actor, viewport))
    {
      bool update_LUT (true);
      if (!style_->lut_actor_id_.empty() && ca_it->first != style_->lut_actor_id_)
        update_LUT = false;
      cloud_actor_map_->erase (ca_it);
      if (update_LUT)
        style_->updateLookUpTableDisplay (false);
      return (true);
    }
  }
  return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeText3D (const std::string &id, int viewport)
{
  if (viewport < 0)
    return false;

  bool success = true;

  // If there is no custom viewport and the viewport number is not 0, exit
  if (rens_->GetNumberOfItems () <= viewport)
  {
    PCL_ERROR ("[removeText3D] The viewport [%d] doesn't exist (id <%s>)!\n",
               viewport,
               id.c_str ());
    return false;
  }

  // check all or an individual viewport for a similar id
  rens_->InitTraversal ();
  for (std::size_t i = viewport; rens_->GetNextItem (); ++i)
  {
    const std::string uid = id + std::string (i, '*');
    auto am_it = shape_actor_map_->find (uid);

    // was it found
    if (am_it == shape_actor_map_->end ())
    {
      if (viewport > 0)
        return (false);

      continue;
    }

    // Remove it from all renderers
    if (removeActorFromRenderer (am_it->second, i))
    {
      // Remove the pointer/ID pair to the global actor map
      shape_actor_map_->erase (am_it);
      if (viewport > 0)
        return (true);

      success &= true;
    }
    else
      success = false;
  }

  return success;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeAllPointClouds (int viewport)
{
  // Check to see if the given ID entry exists
  auto am_it = cloud_actor_map_->begin ();
  while (am_it != cloud_actor_map_->end () )
  {
    if (removePointCloud (am_it->first, viewport))
      am_it = cloud_actor_map_->begin ();
    else
      ++am_it;
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeAllShapes (int viewport)
{
  bool display_lut (style_->lut_enabled_);
  style_->lut_enabled_ = false; // Temporally disable LUT to fasten shape removal

  // Check to see if the given ID entry exists
  auto am_it = shape_actor_map_->begin ();
  while (am_it != shape_actor_map_->end ())
  {
    if (removeShape (am_it->first, viewport))
      am_it = shape_actor_map_->begin ();
    else
      ++am_it;
  }

  if (display_lut)
  {
    style_->lut_enabled_ = true;
    style_->updateLookUpTableDisplay (true);
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeAllCoordinateSystems (int viewport)
{
  // Check to see if the given ID entry exists
  auto am_it = coordinate_actor_map_->begin ();
  while (am_it != coordinate_actor_map_->end () )
  {
    if (removeCoordinateSystem (am_it->first, viewport))
      am_it = coordinate_actor_map_->begin ();
    else
      ++am_it;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor, int viewport)
{
  vtkLODActor* actor_to_remove = vtkLODActor::SafeDownCast (actor);

  // Add it to all renderers
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Should we remove the actor from all renderers?
    if (viewport == 0)
    {
      renderer->RemoveActor (actor);
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      // Iterate over all actors in this renderer
      vtkPropCollection* actors = renderer->GetViewProps ();
      actors->InitTraversal ();
      vtkProp* current_actor = nullptr;
      while ((current_actor = actors->GetNextProp ()))
      {
        if (current_actor != actor_to_remove)
          continue;
        renderer->RemoveActor (actor);
        // Found the correct viewport and removed the actor
        return (true);
      }
    }
    ++i;
  }
  if (viewport == 0) return (true);
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor, int viewport)
{
  vtkActor* actor_to_remove = vtkActor::SafeDownCast (actor);

  // Add it to all renderers
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Should we remove the actor from all renderers?
    if (viewport == 0)
    {
      renderer->RemoveActor (actor);
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      // Iterate over all actors in this renderer
      vtkPropCollection* actors = renderer->GetViewProps ();
      actors->InitTraversal ();
      vtkProp* current_actor = nullptr;
      while ((current_actor = actors->GetNextProp ()))
      {
        if (current_actor != actor_to_remove)
          continue;
        renderer->RemoveActor (actor);
        // Found the correct viewport and removed the actor
        return (true);
      }
    }
    ++i;
  }
  if (viewport == 0) return (true);
  return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::addActorToRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
  // Add it to all renderers
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->AddActor (actor);
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->AddActor (actor);
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
  vtkProp* actor_to_remove = vtkProp::SafeDownCast (actor);

  // Initialize traversal
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Should we remove the actor from all renderers?
    if (viewport == 0)
    {
      renderer->RemoveActor (actor);
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      // Iterate over all actors in this renderer
      vtkPropCollection* actors = renderer->GetViewProps ();
      actors->InitTraversal ();
      vtkProp* current_actor = nullptr;
      while ((current_actor = actors->GetNextProp ()))
      {
        if (current_actor != actor_to_remove)
          continue;
        renderer->RemoveActor (actor);
        // Found the correct viewport and removed the actor
        return (true);
      }
    }
    ++i;
  }
  if (viewport == 0) return (true);
  return (false);
}

/////////////////////////////////////////////////////////////////////////////////////////////
namespace
{
// Helper function called by createActorFromVTKDataSet () methods.
// This function determines the default setting of vtkMapper::InterpolateScalarsBeforeMapping.
// Return 0, interpolation off, if data is a vtkPolyData that contains only vertices.
// Return 1, interpolation on, for anything else.
int
getDefaultScalarInterpolationForDataSet (vtkDataSet* data)
{
  vtkPolyData* polyData = vtkPolyData::SafeDownCast (data); // Check that polyData != NULL in case of segfault
  return (polyData && polyData->GetNumberOfCells () != polyData->GetNumberOfVerts ());
}

}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
                                                              vtkSmartPointer<vtkLODActor> &actor,
                                                              bool use_scalars) const
{
  // If actor is not initialized, initialize it here
  if (!actor)
    actor = vtkSmartPointer<vtkLODActor>::New ();

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInputData (data);

  if (use_scalars)
  {
    vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
    if (scalars)
    {
      double minmax[2];
      scalars->GetRange (minmax);
      mapper->SetScalarRange (minmax);

      mapper->SetScalarModeToUsePointData ();
      mapper->SetInterpolateScalarsBeforeMapping (getDefaultScalarInterpolationForDataSet (data));
      mapper->ScalarVisibilityOn ();
    }
  }

  actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
  actor->GetProperty ()->SetInterpolationToFlat ();

  /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
  /// shown when there is a vtkActor with backface culling on present in the scene
  /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
  // actor->GetProperty ()->BackfaceCullingOn ();

  actor->SetMapper (mapper);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
                                                              vtkSmartPointer<vtkActor> &actor,
                                                              bool use_scalars) const
{
  // If actor is not initialized, initialize it here
  if (!actor)
    actor = vtkSmartPointer<vtkActor>::New ();

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInputData (data);

  if (use_scalars)
  {
    vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
    if (scalars)
    {
      double minmax[2];
      scalars->GetRange (minmax);
      mapper->SetScalarRange (minmax);

      mapper->SetScalarModeToUsePointData ();
      mapper->SetInterpolateScalarsBeforeMapping (getDefaultScalarInterpolationForDataSet (data));
      mapper->ScalarVisibilityOn ();
    }
  }

    //actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
  actor->GetProperty ()->SetInterpolationToFlat ();

  /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
  /// shown when there is a vtkActor with backface culling on present in the scene
  /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
  // actor->GetProperty ()->BackfaceCullingOn ();

  actor->SetMapper (mapper);

  //actor->SetNumberOfCloudPoints (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10));
  actor->GetProperty ()->SetInterpolationToFlat ();
}
/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::convertPointCloudToVTKPolyData (
    const GeometryHandlerConstPtr &geometry_handler,
    vtkSmartPointer<vtkPolyData> &polydata,
    vtkSmartPointer<vtkIdTypeArray> &initcells)
{
  vtkSmartPointer<vtkCellArray> vertices;

  if (!polydata)
  {
    allocVtkPolyData (polydata);
    vertices = vtkSmartPointer<vtkCellArray>::New ();
    polydata->SetVerts (vertices);
  }

  // Use the handler to obtain the geometry
  vtkSmartPointer<vtkPoints> points;
  geometry_handler->getGeometry (points);
  polydata->SetPoints (points);

  vtkIdType nr_points = points->GetNumberOfPoints ();

  // Create the supporting structures
  vertices = polydata->GetVerts ();
  if (!vertices)
    vertices = vtkSmartPointer<vtkCellArray>::New ();

#ifdef VTK_CELL_ARRAY_V2
  // TODO: Remove when VTK 6,7,8 is unsupported
  pcl::utils::ignore(initcells);

  auto numOfCells = vertices->GetNumberOfCells();

  // If we have less cells than points, add new cells.
  if (numOfCells < nr_points)
  {
    for (int i = numOfCells; i < nr_points; i++)
    {
      vertices->InsertNextCell(1);
      vertices->InsertCellPoint(i);
    }
  }
  // if we too many cells than points, set size (doesn't free excessive memory)
  else if (numOfCells > nr_points)
  {
    vertices->ResizeExact(nr_points, nr_points);
  }

  polydata->SetPoints(points);
  polydata->SetVerts(vertices);

#else
  vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
  updateCells (cells, initcells, nr_points);
  // Set the cells and the vertices
  vertices->SetCells (nr_points, cells);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setBackgroundColor (
    const double &r, const double &g, const double &b, int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->SetBackground (r, g, b);
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->SetBackground (r, g, b);
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setPointCloudRenderingProperties (
    int property, double val1, double val2, double val3, const std::string &id, int)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
  {
    pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_COLOR:
    {
      if (val1 > 1.0 || val2 > 1.0 || val3 > 1.0)
        PCL_WARN ("[setPointCloudRenderingProperties] Colors go from 0.0 to 1.0!\n");
      actor->GetProperty ()->SetColor (val1, val2, val3);
      actor->GetMapper ()->ScalarVisibilityOff ();
      actor->Modified ();
      break;
    }
    default:
    {
      pcl::console::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setPointCloudRenderingProperties (
    int property, double val1, double val2, const std::string &id, int)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
  {
    pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_LUT_RANGE:
    {
      // Check if the mapper has scalars
      if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        break;
      
      // Check that scalars are not unisgned char (i.e. check if a LUT is used to colormap scalars assuming vtk ColorMode is Default)
      if (actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->IsA ("vtkUnsignedCharArray"))
        break;

      // Check that range values are correct
      if (val1 >= val2)
      {
        PCL_WARN ("[setPointCloudRenderingProperties] Range max must be greater than range min!\n");
        return (false);
      }
      
      // Update LUT
      actor->GetMapper ()->GetLookupTable ()->SetRange (val1, val2);
      actor->GetMapper()->UseLookupTableScalarRangeOn ();
      style_->updateLookUpTableDisplay (false);
      break;
    }
    default:
    {
      pcl::console::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::getPointCloudRenderingProperties (int property, double &value, const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
  if (!actor)
    return (false);

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
      pcl::console::print_error ("[getPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}
/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::getPointCloudRenderingProperties (RenderingProperties property,
                                                                     double &val1,
                                                                     double &val2,
                                                                     double &val3,
                                                                     const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_COLOR:
    {
      double rgb[3];
      actor->GetProperty ()->GetColor (rgb);
      val1 = rgb[0];
      val2 = rgb[1];
      val3 = rgb[2];
      break;
    }
    default:
    {
      pcl::console::print_error ("[getPointCloudRenderingProperties] "
                                 "Property (%d) is either unknown or it requires a different "
                                 "number of variables to retrieve its contents.\n",
                                 property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setPointCloudRenderingProperties (
    int property, double value, const std::string &id, int)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
  {
    pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_POINT_SIZE:
    {
      actor->GetProperty ()->SetPointSize (float (value));
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_OPACITY:
    {
      actor->GetProperty ()->SetOpacity (value);
      actor->Modified ();
      break;
    }
    // Turn on/off flag to control whether data is rendered using immediate
    // mode or note. Immediate mode rendering tends to be slower but it can
    // handle larger datasets. The default value is immediate mode off. If you
    // are having problems rendering a large dataset you might want to consider
    // using immediate more rendering.
    case PCL_VISUALIZER_IMMEDIATE_RENDERING:
    {
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_LINE_WIDTH:
    {
      actor->GetProperty ()->SetLineWidth (float (value));
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_LUT:
    {
      // Check if the mapper has scalars
      if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        break;
      
      // Check that scalars are not unisgned char (i.e. check if a LUT is used to colormap scalars assuming vtk ColorMode is Default)
      if (actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->IsA ("vtkUnsignedCharArray"))
        break;

      // Get range limits from existing LUT
      double *range;
      range = actor->GetMapper ()->GetLookupTable ()->GetRange ();
      
      actor->GetMapper ()->ScalarVisibilityOn ();
      actor->GetMapper ()->SetScalarRange (range[0], range[1]);
      vtkSmartPointer<vtkLookupTable> table;
      if (!pcl::visualization::getColormapLUT (static_cast<LookUpTableRepresentationProperties>(static_cast<int>(value)), table))
        break;
      table->SetRange (range[0], range[1]);
      actor->GetMapper ()->SetLookupTable (table);
      style_->updateLookUpTableDisplay (false);
      break;
    }
    case PCL_VISUALIZER_LUT_RANGE:
    {
      // Check if the mapper has scalars
      if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        break;
      
      // Check that scalars are not unisgned char (i.e. check if a LUT is used to colormap scalars assuming vtk ColorMode is Default)
      if (actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->IsA ("vtkUnsignedCharArray"))
        break;

      switch (int(value))
      {
        case PCL_VISUALIZER_LUT_RANGE_AUTO:
          double range[2];
          actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->GetRange (range);
          actor->GetMapper ()->GetLookupTable ()->SetRange (range[0], range[1]);
          actor->GetMapper ()->UseLookupTableScalarRangeOn ();
          style_->updateLookUpTableDisplay (false);
          break;
      }
      break;
    }
    default:
    {
      pcl::console::print_error ("[setPointCloudRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setPointCloudSelected (const bool selected, const std::string &id)
{
   // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
  {
    pcl::console::print_error ("[setPointCloudRenderingProperties] Could not find any PointCloud datasets with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second.actor);
  if (!actor)
    return (false);

  if (selected)
  {
    actor->GetProperty ()->EdgeVisibilityOn ();
    actor->GetProperty ()->SetEdgeColor (1.0, 0.0, 0.0);
    actor->Modified ();
  }
  else
  {
    actor->GetProperty ()->EdgeVisibilityOff ();
    actor->Modified ();
  }

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setShapeRenderingProperties (
    int property, double val1, double val2, double val3, const std::string &id, int)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);

  if (am_it == shape_actor_map_->end ())
  {
    pcl::console::print_error ("[setShapeRenderingProperties] Could not find any shape with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkActor* actor = vtkActor::SafeDownCast (am_it->second);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_COLOR:
    {
      if (val1 > 1.0 || val2 > 1.0 || val3 > 1.0)
        PCL_WARN ("[setShapeRenderingProperties] Colors go from 0.0 to 1.0!\n");

      actor->GetMapper ()->ScalarVisibilityOff ();
      actor->GetProperty ()->SetColor (val1, val2, val3);
      actor->GetProperty ()->SetEdgeColor (val1, val2, val3);
      // The following 3 are set by SetColor automatically according to the VTK docs
      //actor->GetProperty ()->SetAmbientColor  (val1, val2, val3);
      //actor->GetProperty ()->SetDiffuseColor (val1, val2, val3);
      //actor->GetProperty ()->SetSpecularColor (val1, val2, val3);
      actor->GetProperty ()->SetAmbient (0.8);
      actor->GetProperty ()->SetDiffuse (0.8);
      actor->GetProperty ()->SetSpecular (0.8);
      actor->Modified ();
      break;
    }
    default:
    {
      pcl::console::print_error ("[setShapeRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setShapeRenderingProperties (
    int property, double val1, double val2, const std::string &id, int)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);

  if (am_it == shape_actor_map_->end ())
  {
    pcl::console::print_error ("[setShapeRenderingProperties] Could not find any shape with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkActor* actor = vtkActor::SafeDownCast (am_it->second);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_LUT_RANGE:
    {
      // Check if the mapper has scalars
      if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        break;
      
      // Check that scalars are not unisgned char (i.e. check if a LUT is used to colormap scalars assuming vtk ColorMode is Default)
      if (actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->IsA ("vtkUnsignedCharArray"))
        break;

      // Check that range values are correct
      if (val1 >= val2)
      {
        PCL_WARN ("[setShapeRenderingProperties] Range max must be greater than range min!\n");
        return (false);
      }
      
      // Update LUT
      actor->GetMapper ()->GetLookupTable ()->SetRange (val1, val2);
      actor->GetMapper()->UseLookupTableScalarRangeOn ();
      style_->updateLookUpTableDisplay (false);
      break;
    }
    default:
    {
      pcl::console::print_error ("[setShapeRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::setShapeRenderingProperties (
    int property, double value, const std::string &id, int)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);

  if (am_it == shape_actor_map_->end ())
  {
    pcl::console::print_error ("[setShapeRenderingProperties] Could not find any shape with id <%s>!\n", id.c_str ());
    return (false);
  }
  // Get the actor pointer
  vtkActor* actor = vtkActor::SafeDownCast (am_it->second);
  if (!actor)
    return (false);

  switch (property)
  {
    case PCL_VISUALIZER_POINT_SIZE:
    {
      actor->GetProperty ()->SetPointSize (float (value));
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
      actor->GetProperty ()->SetLineWidth (float (value));
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_FONT_SIZE:
    {
      vtkTextActor* text_actor = vtkTextActor::SafeDownCast (am_it->second);
      if (!text_actor)
        return (false);
      vtkSmartPointer<vtkTextProperty> tprop = text_actor->GetTextProperty ();
      tprop->SetFontSize (int (value));
      text_actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_REPRESENTATION:
    {
      switch (int (value))
      {
        case PCL_VISUALIZER_REPRESENTATION_POINTS:
        {
          actor->GetProperty ()->SetRepresentationToPoints ();
          break;
        }
        case PCL_VISUALIZER_REPRESENTATION_WIREFRAME:
        {
          actor->GetProperty ()->SetRepresentationToWireframe ();
          break;
        }
        case PCL_VISUALIZER_REPRESENTATION_SURFACE:
        {
          actor->GetProperty ()->SetRepresentationToSurface ();
          break;
        }
      }
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_SHADING:
    {
      switch (int (value))
      {
        case PCL_VISUALIZER_SHADING_FLAT:
        {
          actor->GetProperty ()->SetInterpolationToFlat ();
          break;
        }
        case PCL_VISUALIZER_SHADING_GOURAUD:
        {
          if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetNormals ())
          {
            PCL_INFO ("[pcl::visualization::PCLVisualizer::setShapeRenderingProperties] Normals do not exist in the dataset, but Gouraud shading was requested. Estimating normals...\n");
            vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New ();
            normals->SetInputConnection (actor->GetMapper ()->GetInputAlgorithm ()->GetOutputPort ());
            vtkDataSetMapper::SafeDownCast (actor->GetMapper ())->SetInputConnection (normals->GetOutputPort ());
          }
          actor->GetProperty ()->SetInterpolationToGouraud ();
          break;
        }
        case PCL_VISUALIZER_SHADING_PHONG:
        {
          if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetNormals ())
          {
            PCL_INFO ("[pcl::visualization::PCLVisualizer::setShapeRenderingProperties] Normals do not exist in the dataset, but Phong shading was requested. Estimating normals...\n");
            vtkSmartPointer<vtkPolyDataNormals> normals = vtkSmartPointer<vtkPolyDataNormals>::New ();
            normals->SetInputConnection (actor->GetMapper ()->GetInputAlgorithm ()->GetOutputPort ());
            vtkDataSetMapper::SafeDownCast (actor->GetMapper ())->SetInputConnection (normals->GetOutputPort ());
          }
          actor->GetProperty ()->SetInterpolationToPhong ();
          break;
        }
      }
      actor->Modified ();
      break;
    }
    case PCL_VISUALIZER_LUT:
    {
      // Check if the mapper has scalars
      if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        break;

      // Check that scalars are not unisgned char (i.e. check if a LUT is used to colormap scalars assuming vtk ColorMode is Default)
      if (actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->IsA ("vtkUnsignedCharArray"))
        break;
      
      // Get range limits from existing LUT
      double *range;
      range = actor->GetMapper ()->GetLookupTable ()->GetRange ();
      
      actor->GetMapper ()->ScalarVisibilityOn ();
      actor->GetMapper ()->SetScalarRange (range[0], range[1]);
      vtkSmartPointer<vtkLookupTable> table;
      if (!pcl::visualization::getColormapLUT (static_cast<LookUpTableRepresentationProperties>(static_cast<int>(value)), table))
        break;
      table->SetRange (range[0], range[1]);
      actor->GetMapper ()->SetLookupTable (table);
      style_->updateLookUpTableDisplay (false);
      break;
    }
    case PCL_VISUALIZER_LUT_RANGE:
    {
      // Check if the mapper has scalars
      if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        break;

      // Check that scalars are not unisgned char (i.e. check if a LUT is used to colormap scalars assuming vtk ColorMode is Default)
      if (actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->IsA ("vtkUnsignedCharArray"))
        break;

      switch (int(value))
      {
        case PCL_VISUALIZER_LUT_RANGE_AUTO:
          double range[2];
          actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ()->GetRange (range);
          actor->GetMapper ()->GetLookupTable ()->SetRange (range[0], range[1]);
          actor->GetMapper ()->UseLookupTableScalarRangeOn ();
          style_->updateLookUpTableDisplay (false);
          break;
      }
      break;
    }
    default:
    {
      pcl::console::print_error ("[setShapeRenderingProperties] Unknown property (%d) specified!\n", property);
      return (false);
    }
  }
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::initCameraParameters ()
{
  Camera camera_temp;

  camera_temp.window_size[0] = win_->GetScreenSize ()[0] / 2;
  camera_temp.window_size[1] = win_->GetScreenSize ()[1] / 2;

  setCameraParameters (camera_temp);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::cameraParamsSet () const
{
  return (camera_set_);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::cameraFileLoaded () const
{
  return (camera_file_loaded_);
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::visualization::PCLVisualizer::getCameraFile () const
{
  return (style_->getCameraFile ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updateShapePose (const std::string &id, const Eigen::Affine3f& pose)
{
  auto am_it = shape_actor_map_->find (id);

  vtkLODActor* actor;

  if (am_it == shape_actor_map_->end ())
    return (false);
  actor = vtkLODActor::SafeDownCast (am_it->second);

  if (!actor)
    return (false);

  vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();

  convertToVtkMatrix (pose.matrix (), matrix);

  actor->SetUserMatrix (matrix);
  actor->Modified ();

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updateCoordinateSystemPose (const std::string &id, const Eigen::Affine3f& pose)
{
  auto am_it = coordinate_actor_map_->find (id);

  vtkLODActor* actor;

  if (am_it == coordinate_actor_map_->end ())
    return (false);
  actor = vtkLODActor::SafeDownCast (am_it->second);

  if (!actor)
    return (false);

  vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New ();

  convertToVtkMatrix (pose.matrix (), matrix);

  actor->SetUserMatrix (matrix);
  actor->Modified ();

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updatePointCloudPose (const std::string &id, const Eigen::Affine3f& pose)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  convertToVtkMatrix (pose.matrix (), transformation);
  am_it->second.viewpoint_transformation_ = transformation;
  am_it->second.actor->SetUserMatrix (transformation);
  am_it->second.actor->Modified ();

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::getCameras (std::vector<pcl::visualization::Camera>& cameras)
{
  cameras.clear ();
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
    cameras.emplace_back (*renderer->GetActiveCamera (), *renderer->GetRenderWindow ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Affine3f
pcl::visualization::PCLVisualizer::getViewerPose (int viewport)
{
  Eigen::Affine3f ret (Eigen::Affine3f::Identity ());

  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  if (viewport == 0)
    viewport = 1;
  int viewport_i = 1;
  while ((renderer = rens_->GetNextItem ()))
  {
    if (viewport_i == viewport)
    {
      vtkCamera& camera = *renderer->GetActiveCamera ();
      Eigen::Vector3d pos, x_axis, y_axis, z_axis;
      camera.GetPosition (pos[0], pos[1], pos[2]);
      camera.GetViewUp (y_axis[0], y_axis[1], y_axis[2]);
      camera.GetFocalPoint (z_axis[0], z_axis[1], z_axis[2]);

      z_axis = (z_axis - pos).normalized ();
      x_axis = y_axis.cross (z_axis).normalized ();

      ret.translation () = pos.cast<float> ();
      ret.linear ().col (0) << x_axis.cast<float> ();
      ret.linear ().col (1) << y_axis.cast<float> ();
      ret.linear ().col (2) << z_axis.cast<float> ();

      return ret;
    }
    viewport_i ++;
  }

  return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::resetCamera ()
{
  // Update the camera parameters
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
    renderer->ResetCamera ();
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setCameraPosition (
    double pos_x, double pos_y, double pos_z,
    double view_x, double view_y, double view_z,
    double up_x, double up_y, double up_z,
    int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Modify all renderer's cameras
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
      cam->SetPosition (pos_x, pos_y, pos_z);
      cam->SetFocalPoint (view_x, view_y, view_z);
      cam->SetViewUp (up_x, up_y, up_z);
      renderer->ResetCameraClippingRange ();
    }
    ++i;
  }
  win_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setCameraPosition (
    double pos_x, double pos_y, double pos_z,
    double up_x, double up_y, double up_z, int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Modify all renderer's cameras
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
      cam->SetPosition (pos_x, pos_y, pos_z);
      cam->SetViewUp (up_x, up_y, up_z);
    }
    ++i;
  }
  win_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setCameraParameters (const Eigen::Matrix3f &intrinsics,
                                                        const Eigen::Matrix4f &extrinsics,
                                                        int viewport)
{
  style_->setCameraParameters (intrinsics, extrinsics, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setCameraParameters (const pcl::visualization::Camera &camera, int viewport)
{
  style_->setCameraParameters (camera, viewport);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setCameraClipDistances (double near, double far, int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Modify all renderer's cameras
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
      cam->SetClippingRange (near, far);
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setCameraFieldOfView (double fovy, int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    // Modify all renderer's cameras
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
      cam->SetUseHorizontalViewAngle (0);
      cam->SetViewAngle (fovy * 180.0 / M_PI);
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::resetCameraViewpoint (const std::string &id)
{
  vtkSmartPointer<vtkMatrix4x4> camera_pose;
  const CloudActorMap::iterator it = cloud_actor_map_->find(id);
  if (it != cloud_actor_map_->end ())
    camera_pose = it->second.viewpoint_transformation_;
  else
    return;

  // Prevent a segfault
  if (!camera_pose)
    return;

  // set all renderer to this viewpoint
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
  {
    vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
    cam->SetPosition (camera_pose->GetElement (0, 3),
                      camera_pose->GetElement (1, 3),
                      camera_pose->GetElement (2, 3));

    cam->SetFocalPoint (camera_pose->GetElement (0, 3) - camera_pose->GetElement (0, 2),
                        camera_pose->GetElement (1, 3) - camera_pose->GetElement (1, 2),
                        camera_pose->GetElement (2, 3) - camera_pose->GetElement (2, 2));

    cam->SetViewUp (camera_pose->GetElement (0, 1),
                    camera_pose->GetElement (1, 1),
                    camera_pose->GetElement (2, 1));

    renderer->SetActiveCamera (cam);
    renderer->ResetCameraClippingRange ();
  }
  win_->Render ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::getCameraParameters (int argc, char **argv)
{
  for (int i = 1; i < argc; i++)
  {
    if ((strcmp (argv[i], "-cam") == 0) && (++i < argc))
    {
      std::string camfile = std::string (argv[i]);

      if (camfile.find (".cam") == std::string::npos)
      {
        // Assume we have clip/focal/pos/view
        std::vector<std::string> camera;
        boost::split (camera, argv[i], boost::is_any_of ("/"), boost::token_compress_on);
        return (style_->getCameraParameters (camera));
      }
      // Assume that if we don't have clip/focal/pos/view, a filename.cam was given as a parameter
      return (style_->loadCameraParameters (camfile));
    }
  }
  return (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::loadCameraParameters (const std::string &file)
{
  return (style_->loadCameraParameters (file));
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addCylinder (const pcl::ModelCoefficients &coefficients,
                                               const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCylinder] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 7)
  {
    PCL_WARN ("[addCylinder] Coefficients size does not match expected size (expected 7).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCylinder (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addCube (const pcl::ModelCoefficients &coefficients,
                                            const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 10)
  {
    PCL_WARN ("[addCube] Coefficients size does not match expected size (expected 10).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCube (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addCube (
  const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
  double width, double height, double depth,
  const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCube (translation, rotation, width, height, depth);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addCube (float x_min, float x_max,
                                            float y_min, float y_max,
                                            float z_min, float z_max,
                                            double r, double g, double b,
                                            const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCube] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCube (x_min, x_max, y_min, y_max, z_min, z_max);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  actor->GetProperty ()->SetColor (r, g, b);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addEllipsoid (
  const Eigen::Isometry3d &transform,
  double radius_x, double radius_y, double radius_z,
  const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addEllipsoid] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createEllipsoid (transform, radius_x, radius_y, radius_z);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addSphere (const pcl::ModelCoefficients &coefficients,
                                             const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 4)
  {
    PCL_WARN ("[addSphere] Coefficients size does not match expected size (expected 4).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createSphere (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addModelFromPolyData (
    vtkSmartPointer<vtkPolyData> polydata, const std::string & id, int viewport)
{
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr,
                                "[addModelFromPolyData] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addModelFromPolyData (
    vtkSmartPointer<vtkPolyData> polydata, vtkSmartPointer<vtkTransform> transform, const std::string & id, int viewport)
{
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr,
                                "[addModelFromPolyData] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  vtkSmartPointer <vtkTransformFilter> trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter->SetTransform (transform);
  trans_filter->SetInputData (polydata);
  trans_filter->Update ();

  // Create an Actor
  vtkSmartPointer <vtkLODActor> actor;
  createActorFromVTKDataSet (trans_filter->GetOutput (), actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addModelFromPLYFile (const std::string &filename,
                                                       const std::string &id, int viewport)
{
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr,
                                "[addModelFromPLYFile] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (filename.c_str ());

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (reader->GetOutput (), actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addModelFromPLYFile (const std::string &filename,
                                                       vtkSmartPointer<vtkTransform> transform, const std::string &id,
                                                       int viewport)
{
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr,
                                "[addModelFromPLYFile] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  vtkSmartPointer <vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (filename.c_str ());

  //create transformation filter
  vtkSmartPointer <vtkTransformFilter> trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter->SetTransform (transform);
  trans_filter->SetInputConnection (reader->GetOutputPort ());

  // Create an Actor
  vtkSmartPointer <vtkLODActor> actor;
  createActorFromVTKDataSet (trans_filter->GetOutput (), actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addLine (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addLine] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 6)
  {
    PCL_WARN ("[addLine] Coefficients size does not match expected size (expected 6).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createLine (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Add a plane from a set of given model coefficients
  * \param coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
  * \param id the plane id/name (default: "plane")
  * \param viewport (optional) the id of the new viewport (default: 0)
  */
bool
  pcl::visualization::PCLVisualizer::addPlane (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addPlane] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 4)
  {
    PCL_WARN ("[addPlane] Coefficients size does not match expected size (expected 4).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createPlane (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

bool
  pcl::visualization::PCLVisualizer::addPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addPlane] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 4)
  {
    PCL_WARN ("[addPlane] Coefficients size does not match expected size (expected 4).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createPlane (coefficients, x, y, z);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addCircle (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCircle] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 3)
  {
    PCL_WARN ("[addCircle] Coefficients size does not match expected size (expected 3).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = create2DCircle (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addCone (const pcl::ModelCoefficients &coefficients, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addCone] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  if (coefficients.values.size () != 7)
  {
    PCL_WARN ("[addCone] Coefficients size does not match expected size (expected 7).\n");
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createCone (coefficients);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport)
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
    viewport = rens_->GetNumberOfItems () - 1;

  win_->AddRenderer (ren);
  win_->Modified ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::createViewPortCamera (const int viewport)
{
  vtkSmartPointer<vtkCamera> cam = vtkSmartPointer<vtkCamera>::New ();
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()))
  {
    if (viewport == 0)
      continue;
    if (viewport == i)
    {
      renderer->SetActiveCamera (cam);
      renderer->ResetCamera ();
    }
    ++i;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addText (const std::string &text, int xpos, int ypos, const std::string &id, int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (tid);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addText] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
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
  (*shape_actor_map_)[tid] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id, int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (tid);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addText] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
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
  (*shape_actor_map_)[tid] = actor;
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id, int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (tid);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addText] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkTextActor> actor = vtkSmartPointer<vtkTextActor>::New ();
  actor->SetPosition (xpos, ypos);
  actor->SetInput (text.c_str ());

  vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty ();
  tprop->SetFontSize (fontsize);
  tprop->SetFontFamilyToArial ();
  tprop->SetJustificationToLeft ();
  tprop->BoldOn ();
  tprop->SetColor (r, g, b);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[tid] = actor;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updateText (const std::string &text, int xpos, int ypos, const std::string &id)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (tid);
  if (am_it == shape_actor_map_->end ())
    return (false);

  // Retrieve the Actor
  vtkTextActor* actor = vtkTextActor::SafeDownCast (am_it->second);
  if (!actor)
    return (false);

  actor->SetPosition (xpos, ypos);
  actor->SetInput (text.c_str ());
  actor->Modified ();
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updateText (const std::string &text, int xpos, int ypos, double r, double g, double b, const std::string &id)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (tid);
  if (am_it == shape_actor_map_->end ())
    return (false);

  // Create the Actor
  vtkTextActor* actor = vtkTextActor::SafeDownCast (am_it->second);
  if (!actor)
    return (false);
  actor->SetPosition (xpos, ypos);
  actor->SetInput (text.c_str ());

  vtkSmartPointer<vtkTextProperty> tprop = actor->GetTextProperty ();
  tprop->SetColor (r, g, b);
  actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updateText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b, const std::string &id)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = shape_actor_map_->find (tid);
  if (am_it == shape_actor_map_->end ())
    return (false);

  // Retrieve the Actor
  vtkTextActor *actor = vtkTextActor::SafeDownCast (am_it->second);
  if (!actor)
    return (false);

  actor->SetPosition (xpos, ypos);
  actor->SetInput (text.c_str ());

  vtkTextProperty* tprop = actor->GetTextProperty ();
  tprop->SetFontSize (fontsize);
  tprop->SetColor (r, g, b);

  actor->Modified ();

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updateColorHandlerIndex (const std::string &id, int index)
{
  auto am_it = cloud_actor_map_->find (id);
  if (am_it == cloud_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[updateColorHandlerIndex] PointCloud with id <%s> doesn't exist!\n", id.c_str ());
    return (false);
  }

  std::size_t color_handler_size = am_it->second.color_handlers.size ();
  if (!(std::size_t (index) < color_handler_size))
  {
    pcl::console::print_warn (stderr, "[updateColorHandlerIndex] Invalid index <%d> given! Index must be less than %d.\n", index, int (color_handler_size));
    return (false);
  }
  // Get the handler
  PointCloudColorHandler<pcl::PCLPointCloud2>::ConstPtr color_handler = am_it->second.color_handlers[index];

  auto scalars = color_handler->getColor ();
  double minmax[2];
  scalars->GetRange (minmax);
  // Update the data
  auto *data = dynamic_cast<vtkPolyData*>(am_it->second.actor->GetMapper ()->GetInput ());
  data->GetPointData ()->SetScalars (scalars);
  // Modify the mapper
  auto* mapper = dynamic_cast<vtkDataSetMapper*>(am_it->second.actor->GetMapper ());
  mapper->SetScalarRange (minmax);
  mapper->SetScalarModeToUsePointData ();
  mapper->SetInputData (data);
  // Modify the actor
  am_it->second.actor->SetMapper (mapper);
  am_it->second.actor->Modified ();
  am_it->second.color_handler_index_ = index;

  //style_->setCloudActorMap (cloud_actor_map_);

  return (true);
}



/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addPolygonMesh (const pcl::PolygonMesh &poly_mesh,
                                                   const std::string &id,
                                                   int viewport)
{
  auto am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    pcl::console::print_warn (stderr,
                                "[addPolygonMesh] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  // Create points from polyMesh.cloud
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2 (poly_mesh.cloud, *point_cloud);
  poly_points->SetNumberOfPoints (point_cloud->size ());

  for (std::size_t i = 0; i < point_cloud->size (); ++i) 
  {
    const pcl::PointXYZ& p = (*point_cloud)[i];
    poly_points->InsertPoint (i, p.x, p.y, p.z);
  }

  bool has_color = false;
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  if (pcl::getFieldIndex(poly_mesh.cloud, "rgb") != -1)
  {
    has_color = true;
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2 (poly_mesh.cloud, cloud);
    for (const auto &point : cloud.points)
    {
      const unsigned char color[3] = { point.r, point.g, point.b };
      colors->InsertNextTupleValue (color);
    }
  }
  if (pcl::getFieldIndex (poly_mesh.cloud, "rgba") != -1)
  {
    has_color = true;
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromPCLPointCloud2 (poly_mesh.cloud, cloud);
    for (const auto &point : cloud.points)
    {
      const unsigned char color[3] = { point.r, point.g, point.b };
      colors->InsertNextTupleValue (color);
    }
  }

  vtkSmartPointer<vtkLODActor> actor;
  if (poly_mesh.polygons.size () > 1)
  {
    //create polys from polyMesh.polygons
    vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();

    for (const auto &polygon : poly_mesh.polygons)
    {
      cell_array->InsertNextCell (static_cast<int> (polygon.vertices.size ()));
      for (const auto &vertex : polygon.vertices)
        cell_array->InsertCellPoint (vertex);
    }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
    //    polydata->SetStrips (cell_array);
    polydata->SetPolys (cell_array);
    polydata->SetPoints (poly_points);

    if (has_color)
      polydata->GetPointData ()->SetScalars (colors);

    createActorFromVTKDataSet (polydata, actor);
  }
  else if (poly_mesh.polygons.size () == 1)
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
    std::size_t n_points = poly_mesh.polygons[0].vertices.size ();
    polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

    for (std::size_t j = 0; j < (n_points - 1); j++)
      polygon->GetPointIds ()->SetId (j, poly_mesh.polygons[0].vertices[j]);

    vtkSmartPointer<vtkUnstructuredGrid> poly_grid = vtkSmartPointer<vtkUnstructuredGrid>::New ();
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (poly_points);

    createActorFromVTKDataSet (poly_grid, actor);
    actor->GetProperty ()->SetRepresentationToSurface ();
  }
  else
  {
    PCL_ERROR ("PCLVisualizer::addPolygonMesh: No polygons\n");
    return false;
  }

  actor->GetProperty ()->SetRepresentationToSurface ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  convertToVtkMatrix (point_cloud->sensor_origin_, point_cloud->sensor_orientation_, transformation);
  (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::updatePolygonMesh (
    const pcl::PolygonMesh &poly_mesh,
    const std::string &id)
{

  if (poly_mesh.polygons.empty ())
  {
    pcl::console::print_error ("[updatePolygonMesh] No vertices given!\n");
    return (false);
  }

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);
  if (am_it == cloud_actor_map_->end ())
    return (false);

  // Create points from polyMesh.cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2 (poly_mesh.cloud, *cloud);

  std::vector<pcl::Vertices> verts (poly_mesh.polygons); // copy vector

  // Get the current poly data
  vtkSmartPointer<vtkPolyData> polydata = dynamic_cast<vtkPolyData*>(am_it->second.actor->GetMapper ()->GetInput ());
  if (!polydata)
    return (false);
  vtkSmartPointer<vtkCellArray> cells = polydata->GetStrips ();
  if (!cells)
    return (false);
  vtkSmartPointer<vtkPoints> points = polydata->GetPoints ();
  // Copy the new point array in
  vtkIdType nr_points = cloud->size ();
  points->SetNumberOfPoints (nr_points);

  // Get a pointer to the beginning of the data array
  float *data = dynamic_cast<vtkFloatArray*> (points->GetData ())->GetPointer (0);

  int ptr = 0;
  std::vector<int> lookup;
  // If the dataset is dense (no NaNs)
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i, ptr += 3) {
      std::copy(&(*cloud)[i].x, &(*cloud)[i].x + 3, &data[ptr]);
    }
  }
  else
  {
    lookup.resize (nr_points);
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!isFinite ((*cloud)[i]))
        continue;

      lookup[i] = static_cast<int> (j);
      std::copy (&(*cloud)[i].x, &(*cloud)[i].x + 3, &data[ptr]);
      j++;
      ptr += 3;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  // Get the maximum size of a polygon
  int max_size_of_polygon = -1;
  for (const auto &vertex : verts)
    if (max_size_of_polygon < static_cast<int> (vertex.vertices.size ()))
      max_size_of_polygon = static_cast<int> (vertex.vertices.size ());

  // Update the cells
  cells = vtkSmartPointer<vtkCellArray>::New ();
  
  const auto idx = details::fillCells(lookup, verts, cells, max_size_of_polygon);

  cells->GetData ()->SetNumberOfValues (idx);
  cells->Squeeze ();
  // Set the the vertices
  polydata->SetStrips (cells);

  return (true);
}


///////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addPolylineFromPolygonMesh (
    const pcl::PolygonMesh &polymesh, const std::string &id, int viewport)
{
  auto am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr,
                                "[addPolylineFromPolygonMesh] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  // Create points from polyMesh.cloud
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::fromPCLPointCloud2 (polymesh.cloud, point_cloud);
  poly_points->SetNumberOfPoints (point_cloud.size ());

  for (std::size_t i = 0; i < point_cloud.size (); ++i)
    poly_points->InsertPoint (i, point_cloud[i].x, point_cloud[i].y, point_cloud[i].z);

  // Create a cell array to store the lines in and add the lines to it
  vtkSmartPointer <vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer <vtkPolyData> polyData;
  allocVtkPolyData (polyData);

  for (const auto &polygon : polymesh.polygons)
  {
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(polygon.vertices.size());
    for(std::size_t v = 0; v < polygon.vertices.size(); v++)
    {
      polyLine->GetPointIds ()->SetId (v, polygon.vertices[v]);
    }

    cells->InsertNextCell (polyLine);
  }

  // Add the points to the dataset
  polyData->SetPoints (poly_points);

  // Add the lines to the dataset
  polyData->SetLines (cells);

  // Setup actor and mapper
  vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputData (polyData);

  vtkSmartPointer < vtkActor > actor = vtkSmartPointer<vtkActor>::New ();
  actor->SetMapper (mapper);


  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addTextureMesh (const pcl::TextureMesh &mesh,
                                                   const std::string &id,
                                                   int viewport)
{
  auto am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    PCL_ERROR ("[PCLVisualizer::addTextureMesh] A shape with id <%s> already exists!"
               " Please choose a different id and retry.\n",
               id.c_str ());
    return (false);
  }
  // no texture materials --> exit
  if (mesh.tex_materials.empty ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No textures found!\n");
    return (false);
  }
  // polygons are mapped to texture materials
  if (mesh.tex_materials.size () != mesh.tex_polygons.size ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] Materials number %lu differs from polygons number %lu!\n",
              mesh.tex_materials.size (), mesh.tex_polygons.size ());
    return (false);
  }
  // each texture material should have its coordinates set
  if (mesh.tex_materials.size () != mesh.tex_coordinates.size ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] Coordinates number %lu differs from materials number %lu!\n",
              mesh.tex_coordinates.size (), mesh.tex_materials.size ());
    return (false);
  }
  // total number of vertices
  std::size_t nb_vertices = 0;
  for (const auto &tex_polygon : mesh.tex_polygons)
    nb_vertices += tex_polygon.size ();
  // no vertices --> exit
  if (nb_vertices == 0)
  {
    PCL_ERROR ("[PCLVisualizer::addTextureMesh] No vertices found!\n");
    return (false);
  }
  // total number of coordinates
  std::size_t nb_coordinates = 0;
  for (const auto &tex_coordinate : mesh.tex_coordinates)
    nb_coordinates += tex_coordinate.size ();
  // no texture coordinates --> exit
  if (nb_coordinates == 0)
  {
    PCL_ERROR ("[PCLVisualizer::addTextureMesh] No textures coordinates found!\n");
    return (false);
  }

  // Create points from mesh.cloud
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  bool has_color = false;
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  if ((pcl::getFieldIndex(mesh.cloud, "rgba") != -1) ||
      (pcl::getFieldIndex(mesh.cloud, "rgb") != -1))
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2 (mesh.cloud, cloud);
    if (cloud.empty ())
    {
      PCL_ERROR ("[PCLVisualizer::addTextureMesh] Cloud is empty!\n");
      return (false);
    }
    convertToVtkMatrix (cloud.sensor_origin_, cloud.sensor_orientation_, transformation);
    has_color = true;
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    poly_points->SetNumberOfPoints (cloud.size ());
    for (std::size_t i = 0; i < cloud.size (); ++i)
    {
      const pcl::PointXYZRGB &p = cloud[i];
      poly_points->InsertPoint (i, p.x, p.y, p.z);
      const unsigned char color[3] = { p.r, p.g, p.b };
      colors->InsertNextTupleValue (color);
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2 (mesh.cloud, *cloud);
    // no points --> exit
    if (cloud->points.empty ())
    {
      PCL_ERROR ("[PCLVisualizer::addTextureMesh] Cloud is empty!\n");
      return (false);
    }
    convertToVtkMatrix (cloud->sensor_origin_, cloud->sensor_orientation_, transformation);
    poly_points->SetNumberOfPoints (cloud->size ());
    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
      const pcl::PointXYZ &p = (*cloud)[i];
      poly_points->InsertPoint (i, p.x, p.y, p.z);
    }
  }

  //create polys from polyMesh.tex_polygons
  vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New ();
  for (const auto &tex_polygon : mesh.tex_polygons)
  {
    for (const auto &vertex : tex_polygon)
    {
      polys->InsertNextCell (static_cast<int> (vertex.vertices.size ()));
      for (const auto &point : vertex.vertices)
        polys->InsertCellPoint (point);
    }
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New ();
  polydata->SetPolys (polys);
  polydata->SetPoints (poly_points);
  if (has_color)
    polydata->GetPointData ()->SetScalars (colors);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputData (polydata);

  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  vtkTextureUnitManager* tex_manager = vtkOpenGLRenderWindow::SafeDownCast (win_)->GetTextureUnitManager ();
  if (!tex_manager)
    return (false);
  // hardware always supports multitexturing of some degree
  int texture_units = tex_manager->GetNumberOfTextureUnits ();
  if ((std::size_t) texture_units < mesh.tex_materials.size ())
    PCL_WARN ("[PCLVisualizer::addTextureMesh] GPU texture units %d < mesh textures %d!\n",
              texture_units, mesh.tex_materials.size ());
  // Load textures
  std::size_t last_tex_id = std::min (static_cast<int> (mesh.tex_materials.size ()), texture_units);
  std::size_t tex_id = 0;
  while (tex_id < last_tex_id)
  {
#if (VTK_MAJOR_VERSION == 8 && VTK_MINOR_VERSION >= 2) || VTK_MAJOR_VERSION > 8
    const char *tu = mesh.tex_materials[tex_id].tex_name.c_str();
#else
    int tu = vtkProperty::VTK_TEXTURE_UNIT_0 + tex_id;
#endif
    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New ();
    if (textureFromTexMaterial (mesh.tex_materials[tex_id], texture))
    {
      PCL_WARN ("[PCLVisualizer::addTextureMesh] Failed to load texture %s, skipping!\n",
                mesh.tex_materials[tex_id].tex_name.c_str ());
      continue;
    }
    // the first texture is in REPLACE mode others are in ADD mode
    if (tex_id == 0)
      texture->SetBlendingMode(vtkTexture::VTK_TEXTURE_BLENDING_MODE_REPLACE);
    else
      texture->SetBlendingMode(vtkTexture::VTK_TEXTURE_BLENDING_MODE_ADD);
    // add a texture coordinates array per texture
    vtkSmartPointer<vtkFloatArray> coordinates = vtkSmartPointer<vtkFloatArray>::New ();
    coordinates->SetNumberOfComponents (2);
    const std::string this_coordinates_name = "TCoords" + std::to_string(tex_id);
    coordinates->SetName (this_coordinates_name.c_str ());

    for (std::size_t t = 0; t < mesh.tex_coordinates.size (); ++t)
      if (t == tex_id)
        for (const auto &tc : mesh.tex_coordinates[t])
          coordinates->InsertNextTuple2 (tc[0], tc[1]);
      else
        for (std::size_t tc = 0; tc < mesh.tex_coordinates[t].size (); ++tc)
          coordinates->InsertNextTuple2 (-1.0, -1.0);
    mapper->MapDataArrayToMultiTextureAttribute(tu,
      this_coordinates_name.c_str(),
      vtkDataObject::FIELD_ASSOCIATION_POINTS);
    
    polydata->GetPointData ()->AddArray (coordinates);
    actor->GetProperty ()->SetTexture (tu, texture);
    ++tex_id;
  }

  // set mapper
  actor->SetMapper (mapper);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;

  // Save the viewpoint transformation matrix to the global actor map
  (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;

  return (true);
}

void
pcl::visualization::PCLVisualizer::enableEDLRendering(int viewport)
{
#if VTK_MAJOR_VERSION > 7
  auto* basicPass = vtkRenderStepsPass::New();

  auto* edl = vtkEDLShading::New();
  edl->SetDelegatePass(basicPass);

    // Add it to all renderers
  rens_->InitTraversal();
  vtkRenderer* renderer = nullptr;
  int i = 0;
  while ((renderer = rens_->GetNextItem())) {
    if (i == 0) {
      renderer->SetPass(edl);
    }
    else if (i == viewport) {
      renderer->SetPass(edl);
    }
    i++;
  }
#else
  PCL_WARN("EDL requires VTK version 8 or newer.");
  utils::ignore(viewport);
#endif
}

///////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setRepresentationToSurfaceForAllActors ()
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
  {
    vtkActorCollection * actors = renderer->GetActors ();
    actors->InitTraversal ();
    vtkActor * actor;
    while ((actor = actors->GetNextActor ()))
    {
      actor->GetProperty ()->SetRepresentationToSurface ();
      actor->GetProperty ()->SetLighting (true);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setRepresentationToPointsForAllActors ()
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
  {
    vtkActorCollection * actors = renderer->GetActors ();
    actors->InitTraversal ();
    vtkActor * actor;
    while ((actor = actors->GetNextActor ()))
    {
      actor->GetProperty ()->SetRepresentationToPoints ();
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setRepresentationToWireframeForAllActors ()
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = nullptr;
  while ((renderer = rens_->GetNextItem ()))
  {
    vtkActorCollection * actors = renderer->GetActors ();
    actors->InitTraversal ();
    vtkActor * actor;
    while ((actor = actors->GetNextActor ()))
    {
      actor->GetProperty ()->SetRepresentationToWireframe ();
      actor->GetProperty ()->SetLighting (false);
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setShowFPS (bool show_fps)
{
  update_fps_->actor->SetVisibility (show_fps);
}


///////////////////////////////////////////////////////////////////////////////////
float
pcl::visualization::PCLVisualizer::getFPS () const
{
  return (update_fps_->last_fps);
}


///////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::renderViewTesselatedSphere (
                                                               int xres,
                                                               int yres,
                                                               pcl::PointCloud<pcl::PointXYZ>::CloudVectorType &clouds,
                                                               std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<
                                                               Eigen::Matrix4f> > & poses,
                                                               std::vector<float> & enthropies, int tesselation_level,
                                                               float view_angle, float radius_sphere, bool use_vertices)
{
  if (rens_->GetNumberOfItems () > 1)
  {
    PCL_WARN ("[renderViewTesselatedSphere] Method works only with one renderer.\n");
    return;
  }

  rens_->InitTraversal ();
  vtkRenderer* renderer_pcl_vis = rens_->GetNextItem ();
  vtkActorCollection * actors = renderer_pcl_vis->GetActors ();

  if (actors->GetNumberOfItems () > 1)
    PCL_INFO ("[renderViewTesselatedSphere] Method only consider the first actor on the scene, more than one found.\n");

  //get vtk object from the visualizer
  actors->InitTraversal ();
  vtkActor * actor = actors->GetNextActor ();
  vtkSmartPointer<vtkPolyData> polydata;
  allocVtkPolyData (polydata);
  polydata->CopyStructure (actor->GetMapper ()->GetInput ());

  //center object
  double CoM[3];
  vtkIdType npts_com = 0;
  vtkCellPtsPtr ptIds_com = nullptr;
  vtkSmartPointer<vtkCellArray> cells_com = polydata->GetPolys ();

  double center[3], p1_com[3], p2_com[3], p3_com[3], totalArea_com = 0;
  double comx = 0, comy = 0, comz = 0;
  for (cells_com->InitTraversal (); cells_com->GetNextCell (npts_com, ptIds_com);)
  {
    polydata->GetPoint (ptIds_com[0], p1_com);
    polydata->GetPoint (ptIds_com[1], p2_com);
    polydata->GetPoint (ptIds_com[2], p3_com);
    vtkTriangle::TriangleCenter (p1_com, p2_com, p3_com, center);
    double area_com = vtkTriangle::TriangleArea (p1_com, p2_com, p3_com);
    comx += center[0] * area_com;
    comy += center[1] * area_com;
    comz += center[2] * area_com;
    totalArea_com += area_com;
  }

  CoM[0] = comx / totalArea_com;
  CoM[1] = comy / totalArea_com;
  CoM[2] = comz / totalArea_com;

  vtkSmartPointer<vtkTransform> trans_center = vtkSmartPointer<vtkTransform>::New ();
  trans_center->Translate (-CoM[0], -CoM[1], -CoM[2]);
  vtkSmartPointer<vtkMatrix4x4> matrixCenter = trans_center->GetMatrix ();

  vtkSmartPointer<vtkTransformFilter> trans_filter_center = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter_center->SetTransform (trans_center);
  trans_filter_center->SetInputData (polydata);
  trans_filter_center->Update ();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputConnection (trans_filter_center->GetOutputPort ());
  mapper->Update ();

  //scale so it fits in the unit sphere!
  double bb[6];
  mapper->GetBounds (bb);
  double ms = (std::max) ((std::fabs) (bb[0] - bb[1]),
                          (std::max) ((std::fabs) (bb[2] - bb[3]), (std::fabs) (bb[4] - bb[5])));
  double max_side = radius_sphere / 2.0;
  double scale_factor = max_side / ms;

  vtkSmartPointer<vtkTransform> trans_scale = vtkSmartPointer<vtkTransform>::New ();
  trans_scale->Scale (scale_factor, scale_factor, scale_factor);
  vtkSmartPointer<vtkMatrix4x4> matrixScale = trans_scale->GetMatrix ();

  vtkSmartPointer<vtkTransformFilter> trans_filter_scale = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter_scale->SetTransform (trans_scale);
  trans_filter_scale->SetInputConnection (trans_filter_center->GetOutputPort ());
  trans_filter_scale->Update ();

  mapper->SetInputConnection (trans_filter_scale->GetOutputPort ());
  mapper->Update ();

  //create icosahedron
  vtkSmartPointer<vtkPlatonicSolidSource> ico = vtkSmartPointer<vtkPlatonicSolidSource>::New ();
  ico->SetSolidTypeToIcosahedron ();
  ico->Update ();

  //tessellate cells from icosahedron
  vtkSmartPointer<vtkLoopSubdivisionFilter> subdivide = vtkSmartPointer<vtkLoopSubdivisionFilter>::New ();
  subdivide->SetNumberOfSubdivisions (tesselation_level);
  subdivide->SetInputConnection (ico->GetOutputPort ());
  subdivide->Update ();

  // Get camera positions
  vtkPolyData *sphere = subdivide->GetOutput ();

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > cam_positions;
  if (!use_vertices)
  {
    vtkSmartPointer<vtkCellArray> cells_sphere = sphere->GetPolys ();
    cam_positions.resize (sphere->GetNumberOfPolys ());

    std::size_t i = 0;
    for (cells_sphere->InitTraversal (); cells_sphere->GetNextCell (npts_com, ptIds_com);)
    {
      sphere->GetPoint (ptIds_com[0], p1_com);
      sphere->GetPoint (ptIds_com[1], p2_com);
      sphere->GetPoint (ptIds_com[2], p3_com);
      vtkTriangle::TriangleCenter (p1_com, p2_com, p3_com, center);
      cam_positions[i] = Eigen::Vector3f (float (center[0]), float (center[1]), float (center[2]));
      cam_positions[i].normalize ();
      i++;
    }

  }
  else
  {
    cam_positions.resize (sphere->GetNumberOfPoints ());
    for (vtkIdType i = 0; i < sphere->GetNumberOfPoints (); i++)
    {
      double cam_pos[3];
      sphere->GetPoint (i, cam_pos);
      cam_positions[i] = Eigen::Vector3f (float (cam_pos[0]), float (cam_pos[1]), float (cam_pos[2]));
      cam_positions[i].normalize ();
    }
  }

  double camera_radius = radius_sphere;
  double cam_pos[3];
  double first_cam_pos[3];

  first_cam_pos[0] = cam_positions[0][0] * radius_sphere;
  first_cam_pos[1] = cam_positions[0][1] * radius_sphere;
  first_cam_pos[2] = cam_positions[0][2] * radius_sphere;

  //create renderer and window
  vtkSmartPointer<vtkRenderWindow> render_win = vtkSmartPointer<vtkRenderWindow>::New ();
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New ();
  render_win->AddRenderer (renderer);
  render_win->SetSize (xres, yres);
  renderer->SetBackground (1.0, 0, 0);

  //create picker
  vtkSmartPointer<vtkWorldPointPicker> worldPicker = vtkSmartPointer<vtkWorldPointPicker>::New ();

  vtkSmartPointer<vtkCamera> cam = vtkSmartPointer<vtkCamera>::New ();
  cam->SetFocalPoint (0, 0, 0);

  Eigen::Vector3f cam_pos_3f = cam_positions[0];
  Eigen::Vector3f perp = cam_pos_3f.cross (Eigen::Vector3f::UnitY ());
  cam->SetViewUp (perp[0], perp[1], perp[2]);

  cam->SetPosition (first_cam_pos);
  cam->SetViewAngle (view_angle);
  cam->Modified ();

  //For each camera position, transform the object and render view
  for (const auto &cam_position : cam_positions)
  {
    cam_pos[0] = cam_position[0];
    cam_pos[1] = cam_position[1];
    cam_pos[2] = cam_position[2];

    //create temporal virtual camera
    vtkSmartPointer<vtkCamera> cam_tmp = vtkSmartPointer<vtkCamera>::New ();
    cam_tmp->SetViewAngle (view_angle);

    Eigen::Vector3f cam_pos_3f (static_cast<float> (cam_pos[0]), static_cast<float> (cam_pos[1]), static_cast<float> (cam_pos[2]));
    cam_pos_3f = cam_pos_3f.normalized ();
    Eigen::Vector3f test = Eigen::Vector3f::UnitY ();

    //If the view up is parallel to ray cam_pos - focalPoint then the transformation
    //is singular and no points are rendered...
    //make sure it is perpendicular
    if (std::abs (cam_pos_3f.dot (test)) == 1)
    {
      //parallel, create
      test = cam_pos_3f.cross (Eigen::Vector3f::UnitX ());
    }

    cam_tmp->SetViewUp (test[0], test[1], test[2]);

    for (double &c_pos : cam_pos)
    {
      c_pos *= camera_radius;
    }

    cam_tmp->SetPosition (cam_pos);
    cam_tmp->SetFocalPoint (0, 0, 0);
    cam_tmp->Modified ();

    //rotate model so it looks the same as if we would look from the new position
    vtkSmartPointer<vtkMatrix4x4> view_trans_inverted = vtkSmartPointer<vtkMatrix4x4>::New ();
    vtkMatrix4x4::Invert (cam->GetViewTransformMatrix (), view_trans_inverted);
    vtkSmartPointer<vtkTransform> trans_rot_pose = vtkSmartPointer<vtkTransform>::New ();
    trans_rot_pose->Identity ();
    trans_rot_pose->Concatenate (view_trans_inverted);
    trans_rot_pose->Concatenate (cam_tmp->GetViewTransformMatrix ());
    vtkSmartPointer<vtkTransformFilter> trans_rot_pose_filter = vtkSmartPointer<vtkTransformFilter>::New ();
    trans_rot_pose_filter->SetTransform (trans_rot_pose);
    trans_rot_pose_filter->SetInputConnection (trans_filter_scale->GetOutputPort ());

    //translate model so we can place camera at (0,0,0)
    vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New ();
    translation->Translate (first_cam_pos[0] * -1, first_cam_pos[1] * -1, first_cam_pos[2] * -1);
    vtkSmartPointer<vtkTransformFilter> translation_filter = vtkSmartPointer<vtkTransformFilter>::New ();
    translation_filter->SetTransform (translation);
    translation_filter->SetInputConnection (trans_rot_pose_filter->GetOutputPort ());

    //modify camera
    cam_tmp->SetPosition (0, 0, 0);
    cam_tmp->SetFocalPoint (first_cam_pos[0] * -1, first_cam_pos[1] * -1, first_cam_pos[2] * -1);
    cam_tmp->Modified ();

    //notice transformations for final pose
    vtkSmartPointer<vtkMatrix4x4> matrixRotModel = trans_rot_pose->GetMatrix ();
    vtkSmartPointer<vtkMatrix4x4> matrixTranslation = translation->GetMatrix ();

    mapper->SetInputConnection (translation_filter->GetOutputPort ());
    mapper->Update ();

    //render view
    vtkSmartPointer<vtkActor> actor_view = vtkSmartPointer<vtkActor>::New ();
    actor_view->SetMapper (mapper);
    renderer->SetActiveCamera (cam_tmp);
    renderer->AddActor (actor_view);
    renderer->Modified ();
    //renderer->ResetCameraClippingRange ();
    render_win->Render ();

    //back to real scale transform
    vtkSmartPointer<vtkTransform> backToRealScale = vtkSmartPointer<vtkTransform>::New ();
    backToRealScale->PostMultiply ();
    backToRealScale->Identity ();
    backToRealScale->Concatenate (matrixScale);
    backToRealScale->Concatenate (matrixTranslation);
    backToRealScale->Inverse ();
    backToRealScale->Modified ();
    backToRealScale->Concatenate (matrixTranslation);
    backToRealScale->Modified ();

    Eigen::Matrix4f backToRealScale_eigen;
    backToRealScale_eigen.setIdentity ();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        backToRealScale_eigen (x, y) = static_cast<float> (backToRealScale->GetMatrix ()->GetElement (x, y));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    cloud->points.resize (xres * yres);
    cloud->width = xres * yres;
    cloud->height = 1;

    double coords[3];
    float * depth = new float[xres * yres];
    render_win->GetZbufferData (0, 0, xres - 1, yres - 1, &(depth[0]));

    int count_valid_depth_pixels = 0;
    std::size_t xresolution (xres);
    std::size_t yresolution (yres);
    for (std::size_t x = 0; x < xresolution; x++)
    {
      for (std::size_t y = 0; y < yresolution; y++)
      {
        float value = depth[y * xres + x];
        if (value == 1.0)
          continue;

        worldPicker->Pick (static_cast<double> (x), static_cast<double> (y), value, renderer);
        worldPicker->GetPickPosition (coords);
        (*cloud)[count_valid_depth_pixels].x = static_cast<float> (coords[0]);
        (*cloud)[count_valid_depth_pixels].y = static_cast<float> (coords[1]);
        (*cloud)[count_valid_depth_pixels].z = static_cast<float> (coords[2]);
        (*cloud)[count_valid_depth_pixels].getVector4fMap () = backToRealScale_eigen
            * (*cloud)[count_valid_depth_pixels].getVector4fMap ();
        count_valid_depth_pixels++;
      }
    }

    delete[] depth;

    //////////////////////////////
    // * Compute area of the mesh
    //////////////////////////////

    vtkSmartPointer<vtkPolyData> polydata = mapper->GetInput ();
    polydata->BuildCells ();

    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();
    vtkIdType npts = 0;
    vtkCellPtsPtr ptIds = nullptr;

    double p1[3], p2[3], p3[3], area, totalArea = 0;
    for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds);)
    {
      polydata->GetPoint (ptIds[0], p1);
      polydata->GetPoint (ptIds[1], p2);
      polydata->GetPoint (ptIds[2], p3);
      area = vtkTriangle::TriangleArea (p1, p2, p3);
      totalArea += area;
    }

    /////////////////////////////////////
    // * Select visible cells (triangles)
    /////////////////////////////////////
    vtkSmartPointer<vtkHardwareSelector> hardware_selector = vtkSmartPointer<vtkHardwareSelector>::New ();
    hardware_selector->ClearBuffers ();
    vtkSmartPointer<vtkSelection> hdw_selection = vtkSmartPointer<vtkSelection>::New ();
    hardware_selector->SetRenderer (renderer);
    hardware_selector->SetArea (0, 0, xres - 1, yres - 1);
    hardware_selector->SetFieldAssociation (vtkDataObject::FIELD_ASSOCIATION_CELLS);
    hdw_selection = hardware_selector->Select ();
    if (!hdw_selection || !hdw_selection->GetNode (0) || !hdw_selection->GetNode (0)->GetSelectionList ())
    {
      PCL_WARN ("[renderViewTesselatedSphere] Invalid selection, skipping!\n");
      continue;
    }

    vtkSmartPointer<vtkIdTypeArray> ids;
    ids = vtkIdTypeArray::SafeDownCast (hdw_selection->GetNode (0)->GetSelectionList ());
    if (!ids)
      return;
    double visible_area = 0;
    for (vtkIdType sel_id = 0; sel_id < (ids->GetNumberOfTuples ()); sel_id++)
    {
      int id_mesh = static_cast<int> (ids->GetValue (sel_id));
      vtkCell * cell = polydata->GetCell (id_mesh);
      auto* triangle = dynamic_cast<vtkTriangle*> (cell);
      if (!triangle)
      {
        PCL_WARN ("[renderViewTesselatedSphere] Invalid triangle %d, skipping!\n", id_mesh);
        continue;
      }

      double p0[3];
      double p1[3];
      double p2[3];
      triangle->GetPoints ()->GetPoint (0, p0);
      triangle->GetPoints ()->GetPoint (1, p1);
      triangle->GetPoints ()->GetPoint (2, p2);
      area = vtkTriangle::TriangleArea (p0, p1, p2);
      visible_area += area;
    }

    enthropies.push_back (static_cast<float> (visible_area / totalArea));

    cloud->points.resize (count_valid_depth_pixels);
    cloud->width = count_valid_depth_pixels;

    //transform cloud to give camera coordinates instead of world coordinates!
    vtkSmartPointer<vtkMatrix4x4> view_transform = cam_tmp->GetViewTransformMatrix ();
    Eigen::Matrix4f trans_view;
    trans_view.setIdentity ();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        trans_view (x, y) = static_cast<float> (view_transform->GetElement (x, y));

    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    //thus, the fliping in y and z
    for (auto &point : cloud->points)
    {
      point.getVector4fMap () = trans_view * point.getVector4fMap ();
      point.y *= -1.0f;
      point.z *= -1.0f;
    }

    renderer->RemoveActor (actor_view);

    clouds.push_back (*cloud);

    //create pose, from OBJECT coordinates to CAMERA coordinates!
    vtkSmartPointer<vtkTransform> transOCtoCC = vtkSmartPointer<vtkTransform>::New ();
    transOCtoCC->PostMultiply ();
    transOCtoCC->Identity ();
    transOCtoCC->Concatenate (matrixCenter);
    transOCtoCC->Concatenate (matrixRotModel);
    transOCtoCC->Concatenate (matrixTranslation);
    transOCtoCC->Concatenate (cam_tmp->GetViewTransformMatrix ());

    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    //thus, the fliping in y and z
    vtkSmartPointer<vtkMatrix4x4> cameraSTD = vtkSmartPointer<vtkMatrix4x4>::New ();
    cameraSTD->Identity ();
    cameraSTD->SetElement (0, 0, 1);
    cameraSTD->SetElement (1, 1, -1);
    cameraSTD->SetElement (2, 2, -1);

    transOCtoCC->Concatenate (cameraSTD);
    transOCtoCC->Modified ();

    Eigen::Matrix4f pose_view;
    pose_view.setIdentity ();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        pose_view (x, y) = static_cast<float> (transOCtoCC->GetMatrix ()->GetElement (x, y));

    poses.push_back (pose_view);

  }
}

///////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::renderView (int xres, int yres, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  if (rens_->GetNumberOfItems () > 1)
  {
    PCL_WARN ("[renderView] Method will render only the first viewport\n");
    return;
  }

  win_->SetSize (xres, yres);
  win_->Render ();

  float dwidth = 2.0f / float (xres),
        dheight = 2.0f / float (yres);

  cloud->points.resize (xres * yres);
  cloud->width = xres;
  cloud->height = yres;

  float *depth = new float[xres * yres];
  win_->GetZbufferData (0, 0, xres - 1, yres - 1, &(depth[0]));

  // Transform cloud to give camera coordinates instead of world coordinates!
  vtkRenderer *ren = rens_->GetFirstRenderer ();
  vtkCamera *camera = ren->GetActiveCamera ();
  vtkSmartPointer<vtkMatrix4x4> composite_projection_transform = camera->GetCompositeProjectionTransformMatrix (ren->GetTiledAspectRatio (), 0, 1);
  vtkSmartPointer<vtkMatrix4x4> view_transform = camera->GetViewTransformMatrix ();

  Eigen::Matrix4f mat1, mat2;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
    {
      mat1 (i, j) = static_cast<float> (composite_projection_transform->Element[i][j]);
      mat2 (i, j) = static_cast<float> (view_transform->Element[i][j]);
    }

  mat1 = mat1.inverse ().eval ();

  int ptr = 0;
  for (int y = 0; y < yres; ++y)
  {
    for (int x = 0; x < xres; ++x, ++ptr)
    {
      pcl::PointXYZ &pt = (*cloud)[ptr];

      if (depth[ptr] == 1.0)
      {
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        continue;
      }

      Eigen::Vector4f world_coords (dwidth  * float (x) - 1.0f,
                                    dheight * float (y) - 1.0f,
                                    depth[ptr],
                                    1.0f);
      world_coords = mat2 * mat1 * world_coords;

      float w3 = 1.0f / world_coords[3];
      world_coords[0] *= w3;
      // vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right), thus, the fliping in y and z
      world_coords[1] *= -w3;
      world_coords[2] *= -w3;

      pt.x = static_cast<float> (world_coords[0]);
      pt.y = static_cast<float> (world_coords[1]);
      pt.z = static_cast<float> (world_coords[2]);
    }
  }

  delete[] depth;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
    const GeometryHandlerConstPtr &geometry_handler,
    const ColorHandlerConstPtr &color_handler,
    const std::string &id,
    int viewport,
    const Eigen::Vector4f& sensor_origin,
    const Eigen::Quaternion<float>& sensor_orientation)
{
  if (!geometry_handler->isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid geometry handler (%s)!\n", id.c_str (), geometry_handler->getName ().c_str ());
    return (false);
  }

  if (!color_handler->isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid color handler (%s)!\n", id.c_str (), color_handler->getName ().c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkIdTypeArray> initcells;

  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata, initcells);

  // Get the colors from the handler
  bool has_colors = false;
  double minmax[2];
  if (auto scalars = color_handler->getColor ())
  {
    polydata->GetPointData ()->SetScalars (scalars);
    scalars->GetRange (minmax);
    has_colors = true;
  }

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);
  if (has_colors)
    actor->GetMapper ()->SetScalarRange (minmax);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor& cloud_actor = (*cloud_actor_map_)[id];
  cloud_actor.actor = actor;
  cloud_actor.cells = reinterpret_cast<vtkPolyDataMapper*>(actor->GetMapper ())->GetInput ()->GetVerts ()->GetData ();
  cloud_actor.geometry_handlers.push_back (geometry_handler);
  cloud_actor.color_handlers.push_back (color_handler);

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  convertToVtkMatrix (sensor_origin, sensor_orientation, transformation);
  cloud_actor.viewpoint_transformation_ = transformation;
  cloud_actor.actor->SetUserMatrix (transformation);
  cloud_actor.actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::updateCells (vtkSmartPointer<vtkIdTypeArray> &cells,
                                                vtkSmartPointer<vtkIdTypeArray> &initcells,
                                                vtkIdType nr_points)
{
  // If no init cells and cells has not been initialized...
  if (!cells)
    cells = vtkSmartPointer<vtkIdTypeArray>::New ();

  // If we have less values then we need to recreate the array
  if (cells->GetNumberOfTuples () < nr_points)
  {
    cells = vtkSmartPointer<vtkIdTypeArray>::New ();

    // If init cells is given, and there's enough data in it, use it
    if (initcells && initcells->GetNumberOfTuples () >= nr_points)
    {
      cells->DeepCopy (initcells);
      cells->SetNumberOfComponents (2);
      cells->SetNumberOfTuples (nr_points);
    }
    else
    {
      // If the number of tuples is still too small, we need to recreate the array
      cells->SetNumberOfComponents (2);
      cells->SetNumberOfTuples (nr_points);
      vtkIdType *cell = cells->GetPointer (0);
      for (vtkIdType i = 0; i < nr_points; ++i, cell += 2)
      {
        *cell = 1;
        *(cell + 1) = i;
      }

      // Save the results in initcells
      initcells = vtkSmartPointer<vtkIdTypeArray>::New ();
      initcells->DeepCopy (cells);
    }
  }
  else
  {
    // The assumption here is that the current set of cells has more data than needed
    cells->SetNumberOfComponents (2);
    cells->SetNumberOfTuples (nr_points);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::allocVtkPolyData (vtkSmartPointer<vtkAppendPolyData> &polydata)
{
  polydata = vtkSmartPointer<vtkAppendPolyData>::New ();
}
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::allocVtkPolyData (vtkSmartPointer<vtkPolyData> &polydata)
{
  polydata = vtkSmartPointer<vtkPolyData>::New ();
}
//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata)
{
  polydata = vtkSmartPointer<vtkUnstructuredGrid>::New ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::getTransformationMatrix (
    const Eigen::Vector4f &origin,
    const Eigen::Quaternion<float> &orientation,
    Eigen::Matrix4f &transformation)
{
  transformation.setIdentity ();
  transformation.block<3, 3> (0, 0) = orientation.toRotationMatrix ();
  transformation.block<3, 1> (0, 3) = origin.head (3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::convertToVtkMatrix (
    const Eigen::Vector4f &origin,
    const Eigen::Quaternion<float> &orientation,
    vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
  // set rotation
  Eigen::Matrix3f rot = orientation.toRotationMatrix ();
  for (int i = 0; i < 3; i++)
    for (int k = 0; k < 3; k++)
      vtk_matrix->SetElement (i, k, rot (i, k));

  // set translation
  vtk_matrix->SetElement (0, 3, origin (0));
  vtk_matrix->SetElement (1, 3, origin (1));
  vtk_matrix->SetElement (2, 3, origin (2));
  vtk_matrix->SetElement (3, 3, 1.0f);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::convertToVtkMatrix (
    const Eigen::Matrix4f &m,
    vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
  for (int i = 0; i < 4; i++)
    for (int k = 0; k < 4; k++)
      vtk_matrix->SetElement (i, k, m (i, k));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::convertToEigenMatrix (
    const vtkSmartPointer<vtkMatrix4x4> &vtk_matrix,
    Eigen::Matrix4f &m)
{
  for (int i = 0; i < 4; i++)
    for (int k = 0; k < 4; k++)
      m (i, k) = static_cast<float> (vtk_matrix->GetElement (i, k));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addPointCloud (
    const pcl::PCLPointCloud2::ConstPtr &,
    const GeometryHandlerConstPtr &geometry_handler,
    const ColorHandlerConstPtr &color_handler,
    const Eigen::Vector4f& sensor_origin,
    const Eigen::Quaternion<float>& sensor_orientation,
    const std::string &id, int viewport)
{
  // Check to see if this entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    am_it->second.geometry_handlers.push_back (geometry_handler);
    am_it->second.color_handlers.push_back (color_handler);
    return (true);
  }
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, sensor_origin, sensor_orientation));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addPointCloud (
    const pcl::PCLPointCloud2::ConstPtr &cloud,
    const GeometryHandlerConstPtr &geometry_handler,
    const Eigen::Vector4f& sensor_origin,
    const Eigen::Quaternion<float>& sensor_orientation,
    const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    am_it->second.geometry_handlers.push_back (geometry_handler);
    return (true);
  }

  PointCloudColorHandlerCustom<pcl::PCLPointCloud2>::Ptr color_handler (new PointCloudColorHandlerCustom<pcl::PCLPointCloud2> (cloud, 255, 255, 255));
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, sensor_origin, sensor_orientation));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::addPointCloud (
    const pcl::PCLPointCloud2::ConstPtr &cloud,
    const ColorHandlerConstPtr &color_handler,
    const Eigen::Vector4f& sensor_origin,
    const Eigen::Quaternion<float>& sensor_orientation,
    const std::string &id, int viewport)
{
  // Check to see if this entry already exists (has it been already added to the visualizer?)
  auto am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    am_it->second.color_handlers.push_back (color_handler);
    return (true);
  }

  PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>::Ptr geometry_handler (new PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud));
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, sensor_origin, sensor_orientation));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setFullScreen (bool mode)
{
  if (win_)
    win_->SetFullScreen (mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setWindowName (const std::string &name)
{
  if (win_)
    win_->SetWindowName (name.c_str ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setWindowBorders (bool mode)
{
  if (win_)
    win_->SetBorders (mode);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setPosition (int x, int y)
{
  if (win_)
  {
    win_->SetPosition (x, y);
    win_->Render ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setSize (int xw, int yw)
{
  if (win_)
  {
    win_->SetSize (xw, yw);
    win_->Render ();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::close ()
{
  stopped_ = true;
  // This tends to close the window...
  win_->Finalize ();
  if (interactor_)
    interactor_->TerminateApp ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::removeCorrespondences (
    const std::string &id, int viewport)
{ 
  removeShape (id, viewport);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PCLVisualizer::getColorHandlerIndex (const std::string &id)
{
  auto am_it = style_->getCloudActorMap ()->find (id);
  if (am_it == cloud_actor_map_->end ())
    return (-1);

  return (am_it->second.color_handler_index_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PCLVisualizer::getGeometryHandlerIndex (const std::string &id)
{
  auto am_it = style_->getCloudActorMap ()->find (id);
  if (am_it != cloud_actor_map_->end ())
    return (-1);

  return (am_it->second.geometry_handler_index_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizer::wasStopped () const
{
  if (interactor_)
    return (stopped_);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::resetStoppedFlag ()
{
  if (interactor_)
    stopped_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setUseVbos (bool use_vbos)
{
  PCL_WARN ("[PCLVisualizer::setUseVbos] Has no effect when OpenGL version is ≥ 2\n");
  pcl::utils::ignore(use_vbos);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::setLookUpTableID (const std::string id)
{
  style_->lut_actor_id_ = id;
  style_->updateLookUpTableDisplay (false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::ExitMainLoopTimerCallback::Execute (
    vtkObject*, unsigned long event_id, void* call_data)
{
  if (event_id != vtkCommand::TimerEvent)
    return;
  int timer_id = *static_cast<int*> (call_data);
  //PCL_WARN ("[pcl::visualization::PCLVisualizer::ExitMainLoopTimerCallback] Timer %d called.\n", timer_id);
  if (timer_id != right_timer_id)
    return;
  // Stop vtk loop and send notification to app to wake it up
  pcl_visualizer->interactor_->TerminateApp ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::ExitCallback::Execute (
    vtkObject*, unsigned long event_id, void*)
{
  if (event_id != vtkCommand::ExitEvent)
    return;
  pcl_visualizer->stopped_ = true;
  // This tends to close the window...
  if (pcl_visualizer->interactor_)
    pcl_visualizer->interactor_->TerminateApp ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizer::FPSCallback::Execute (
    vtkObject* caller, unsigned long, void*)
{
  auto *ren = reinterpret_cast<vtkRenderer *> (caller);
  last_fps = 1.0f / static_cast<float> (ren->GetLastRenderTimeInSeconds ());
  char buf[128];
  sprintf (buf, "%.1f FPS", last_fps);
  actor->SetInput (buf);
}

/////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::visualization::PCLVisualizer::textureFromTexMaterial (const pcl::TexMaterial& tex_mat,
                                                           vtkTexture* vtk_tex) const
{
  if (tex_mat.tex_file.empty())
  {
    PCL_WARN ("[PCLVisualizer::textureFromTexMaterial] No texture file given for material %s!\n",
               tex_mat.tex_name.c_str ());
    return (-1);
  }

  boost::filesystem::path full_path (tex_mat.tex_file.c_str ());
  if (!boost::filesystem::exists (full_path))
  {
    boost::filesystem::path parent_dir = full_path.parent_path ();
    std::string upper_filename = tex_mat.tex_file;
    boost::to_upper (upper_filename);
    std::string real_name;

    try
    {
      if (!boost::filesystem::exists (parent_dir))
      {
        PCL_WARN ("[PCLVisualizer::textureFromTexMaterial] Parent directory '%s' doesn't exist!\n",
                   parent_dir.string ().c_str ());
        return (-1);
      }

      if (!boost::filesystem::is_directory (parent_dir))
      {
        PCL_WARN ("[PCLVisualizer::textureFromTexMaterial] Parent '%s' is not a directory !\n",
                   parent_dir.string ().c_str ());
        return (-1);
      }

      using paths_vector = std::vector<boost::filesystem::path>;
      paths_vector paths;
      std::copy (boost::filesystem::directory_iterator (parent_dir),
                 boost::filesystem::directory_iterator (),
                 back_inserter (paths));

      for (const auto& path : paths)
      {
        if (boost::filesystem::is_regular_file (path))
        {
          std::string name = path.string ();
          boost::to_upper (name);
          if (name == upper_filename)
          {
            real_name = path.string ();
            break;
          }
        }
      }
      // Check texture file existence
      if (real_name.empty())
      {
        PCL_WARN ("[PCLVisualizer::textureFromTexMaterial] Can not find texture file %s!\n",
                   tex_mat.tex_file.c_str ());
        return (-1);
      }
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {

      PCL_WARN ("[PCLVisualizer::textureFromTexMaterial] Error %s when looking for file %s\n!",
                 ex.what (), tex_mat.tex_file.c_str ());
      return (-1);
    }

    //Save the real path
    full_path = real_name.c_str ();
  }

  std::string extension = full_path.extension ().string ();
  //!!! nizar 20131206 : The list is far from being exhaustive I am afraid.
  if ((extension == ".jpg") || (extension == ".JPG"))
  {
    vtkSmartPointer<vtkJPEGReader> jpeg_reader = vtkSmartPointer<vtkJPEGReader>::New ();
    jpeg_reader->SetFileName (full_path.string ().c_str ());
    jpeg_reader->Update ();
    vtk_tex->SetInputConnection (jpeg_reader->GetOutputPort ());
  }
  else if ((extension == ".bmp") || (extension == ".BMP"))
  {
    vtkSmartPointer<vtkBMPReader> bmp_reader = vtkSmartPointer<vtkBMPReader>::New ();
    bmp_reader->SetFileName (full_path.string ().c_str ());
    bmp_reader->Update ();
    vtk_tex->SetInputConnection (bmp_reader->GetOutputPort ());
  }
  else if ((extension == ".pnm") || (extension == ".PNM"))
  {
    vtkSmartPointer<vtkPNMReader> pnm_reader = vtkSmartPointer<vtkPNMReader>::New ();
    pnm_reader->SetFileName (full_path.string ().c_str ());
    pnm_reader->Update ();
    vtk_tex->SetInputConnection (pnm_reader->GetOutputPort ());
  }
  else if ((extension == ".png") || (extension == ".PNG"))
  {
    vtkSmartPointer<vtkPNGReader> png_reader = vtkSmartPointer<vtkPNGReader>::New ();
    png_reader->SetFileName (full_path.string ().c_str ());
    png_reader->Update ();
    vtk_tex->SetInputConnection (png_reader->GetOutputPort ());
  }
  else if ((extension == ".tiff") || (extension == ".TIFF"))
  {
    vtkSmartPointer<vtkTIFFReader> tiff_reader = vtkSmartPointer<vtkTIFFReader>::New ();
    tiff_reader->SetFileName (full_path.string ().c_str ());
    tiff_reader->Update ();
    vtk_tex->SetInputConnection (tiff_reader->GetOutputPort ());
  }
  else
  {
    PCL_WARN ("[PCLVisualizer::textureFromTexMaterial] Unhandled image %s for material %s!\n",
               full_path.c_str (), tex_mat.tex_name.c_str ());
    return (-1);
  }

  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::string
pcl::visualization::PCLVisualizer::getUniqueCameraFile (int argc, char **argv)
{
  std::vector<int> p_file_indices;
  std::ostringstream sstream;

  p_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (!p_file_indices.empty ())
  {
    boost::uuids::detail::sha1 sha1;    
    bool valid = false;

    // Calculate sha1 using canonical paths
    for (const int &p_file_index : p_file_indices)
    {
      boost::filesystem::path path (argv[p_file_index]);
      if (boost::filesystem::exists (path))
      {
        path = boost::filesystem::canonical (path);
        const auto pathStr = path.string ();
        sha1.process_bytes (pathStr.c_str(), pathStr.size());
        valid = true;
      }
    }

    // Build camera filename
    if (valid)
    {
      unsigned int digest[5];
      sha1.get_digest (digest);
      sstream << ".";
      sstream << std::hex << digest[0] << digest[1] << digest[2] << digest[3] << digest[4];
      sstream << ".cam";
    }
  }

  return (sstream.str ());
}
