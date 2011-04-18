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
 * $Id$
 *
 */

#include <list>
#include <pcl/visualization/common/io.h>
#include <pcl/visualization/interactor_style.h>
#include <vtkPolyData.h>
#include <vtkMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkAppendPolyData.h>
#include <vtkTextProperty.h>
#include <vtkAbstractPicker.h>
#include <vtkAbstractPropPicker.h>
#include <vtkPlanes.h>

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Initialization routine. Must be called before anything else. */
void
  pcl_visualization::PCLVisualizerInteractorStyle::Initialize ()
{
  // Set windows size (width, height) to unknown (-1)
  win_height_ = win_width_ = -1;
  win_pos_x_ = win_pos_y_ = 0;
  max_win_height_ = max_win_width_ = -1;

  // Grid is disabled by default
  grid_enabled_ = false;
  grid_actor_ = vtkSmartPointer<vtkLegendScaleActor>::New ();

  // LUT is disabled by default
  lut_enabled_ = false;
  lut_actor_ = vtkSmartPointer<vtkScalarBarActor>::New ();
  lut_actor_->SetTitle ("");
  lut_actor_->SetOrientationToHorizontal ();
  lut_actor_->SetPosition (0.05, 0.01);
  lut_actor_->SetWidth (0.9);
  lut_actor_->SetHeight (0.1);
  lut_actor_->SetNumberOfLabels (lut_actor_->GetNumberOfLabels () * 2);
  vtkSmartPointer<vtkTextProperty> prop = lut_actor_->GetLabelTextProperty ();
  prop->SetFontSize (10);
  lut_actor_->SetLabelTextProperty (prop);
  lut_actor_->SetTitleTextProperty (prop);

  // Create the image filter and PNG writer objects
  wif_ = vtkSmartPointer<vtkWindowToImageFilter>::New ();
  snapshot_writer_ = vtkSmartPointer<vtkPNGWriter>::New ();
  snapshot_writer_->SetInputConnection (wif_->GetOutputPort ());

  init_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Interactor style internal method. Zoom in. */
void
  pcl_visualization::PCLVisualizerInteractorStyle::zoomIn ()
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
  // Zoom in
  StartDolly ();
  double factor = 10.0 * 0.2 * .5;
  Dolly (pow (1.1, factor));
  EndDolly ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Interactor style internal method. Zoom out. */
void
  pcl_visualization::PCLVisualizerInteractorStyle::zoomOut ()
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
  // Zoom out
  StartDolly ();
  double factor = 10.0 * -0.2 * .5;
  Dolly (pow (1.1, factor));
  EndDolly ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Interactor style internal method. Gets called whenever a key is pressed. */
void
  pcl_visualization::PCLVisualizerInteractorStyle::OnChar ()
{
  if (!init_)
  {
    pcl::console::print_error ("[PCLVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
    return;
  }

  if (!rens_)
  {
    pcl::console::print_error ("[PCLVisualizerInteractorStyle] No renderer collection given! Use SetRendererCollection () before continuing.\n");
    return;
  }

  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

  if (wif_->GetInput () == NULL)
  {
    wif_->SetInput (Interactor->GetRenderWindow ());
    wif_->Modified ();
    snapshot_writer_->Modified ();
  }

  // Save the initial windows width/height
  if (win_height_ == -1 || win_width_ == -1)
  {
    int *win_size = Interactor->GetRenderWindow ()->GetSize ();
    win_height_ = win_size[0];
    win_width_  = win_size[1];
  }

  // Get the status of special keys (Cltr+Alt+Shift)
  //bool shift = Interactor->GetShiftKey   ();
  bool ctrl  = Interactor->GetControlKey ();
  bool alt   = Interactor->GetAltKey     ();

  //fprintf (stderr, "Key sym: %s\n", Interactor->GetKeySym ());
  // ---[ Check the rest of the key codes 

  // Switch between point color/geometry handlers
  if (Interactor->GetKeyCode () >= '0' && Interactor->GetKeyCode () <= '9')
  {
    CloudActorMap::iterator it;
    int index = Interactor->GetKeyCode () - '0' - 1;
    if (index == -1) index = 9;

    // Add 10 more for CTRL+0..9 keys
    if (ctrl)
      index += 10;

    // Geometry ?
    if (alt)
    {
      for (it = actors_->begin (); it != actors_->end (); ++it)
      {
        CloudActor *act = &(*it).second;
        if (index >= (int)act->geometry_handlers.size ())
          continue;

        // Save the geometry handler index for later usage
        act->geometry_handler_index_ = index;

        // Create the new geometry
        PointCloudGeometryHandler<sensor_msgs::PointCloud2>::ConstPtr geometry_handler = act->geometry_handlers[index];
        //pcl::console::print_debug ("[OnChar] Setting a new geometry handler (%s) for actor %s\n", geometry_handler->getFieldName ().c_str (), (*it).first.c_str ());

        // Use the handler to obtain the geometry
        vtkSmartPointer<vtkPoints> points;
        geometry_handler->getGeometry (points);

        // Set the vertices
        vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
        for (vtkIdType i = 0; i < (int)points->GetNumberOfPoints (); ++i)
          vertices->InsertNextCell ((vtkIdType)1, &i);

        // Create the data
        vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New ();
        data->SetPoints (points);
        data->SetVerts (vertices);
        // Modify the mapper
        vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(act->actor->GetMapper ());
        mapper->SetInput (data);
        // Modify the actor
        act->actor->SetMapper (mapper);
        act->actor->Modified ();
      }
    }
    else
    {
      for (it = actors_->begin (); it != actors_->end (); ++it)
      {
        CloudActor *act = &(*it).second;
        // Check for out of bounds
        if (index >= (int)act->color_handlers.size ())
          continue;

        // Save the color handler index for later usage
        act->color_handler_index_ = index;

        // Get the new color
        PointCloudColorHandler<sensor_msgs::PointCloud2>::ConstPtr color_handler = act->color_handlers[index];

        //pcl::console::print_debug ("[OnChar] Setting a new color handler (%s) for actor %s\n", color_handler->getFieldName ().c_str (), (*it).first.c_str ());

        vtkSmartPointer<vtkDataArray> scalars;
        color_handler->getColor (scalars);
        double minmax[2];
        scalars->GetRange (minmax);
        // Update the data
        vtkPolyData *data = static_cast<vtkPolyData*>(act->actor->GetMapper ()->GetInput ());
        data->GetPointData ()->SetScalars (scalars);
        data->Update ();
        // Modify the mapper
        vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(act->actor->GetMapper ());
        mapper->SetScalarRange (minmax);
        mapper->SetScalarModeToUsePointData ();
        mapper->SetInput (data);
        // Modify the actor
        act->actor->SetMapper (mapper);
        act->actor->Modified ();
      }
    }

    Interactor->Render ();
    return;
  }

  std::string key (Interactor->GetKeySym ());
  if (key.find ("XF86ZoomIn") != std::string::npos)
    zoomIn ();
  else if (key.find ("XF86ZoomOut") != std::string::npos)
    zoomOut ();

  switch (Interactor->GetKeyCode ())
  {
    case 'h': case 'H':
    {
      pcl::console::print_info ("| Help:\n"
                  "-------\n"
                  "          p, P   : switch to a point-based representation\n"
                  "          w, W   : switch to a wireframe-based representation (where available)\n"
                  "          s, S   : switch to a surface-based representation (where available)\n"
                  "\n"
                  "          j, J   : take a .PNG snapshot of the current window view\n"
                  "          c, C   : display current camera/window parameters\n"
                  "          f, F   : fly to point mode\n"
                  "\n"
                  "         + / -   : increment/decrement overall point size\n"
                  "\n"
                  "          g, G   : display scale grid (on/off)\n"
                  "          u, U   : display lookup table (on/off)\n"
                  "\n"
                  "    r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]\n"
                  "\n"
                  "    ALT + s, S   : turn stereo mode on/off\n"
                  "    ALT + f, F   : switch between maximized window mode and original size\n"
                  "\n"
                  "          l, L           : list all available geometric and color handlers for the current actor map\n"
                  "    ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)\n"
                  "          0..9 [+ CTRL]  : switch between different color handlers (where available)\n"
          );
      break;
    }

    // Get the list of available handlers
    case 'l': case 'L':
    {
      // Iterate over the entire actors list and extract the geomotry/color handlers list
      for (CloudActorMap::iterator it = actors_->begin (); it != actors_->end (); ++it)
      {
        std::list<std::string> geometry_handlers_list, color_handlers_list;
        CloudActor *act = &(*it).second;
        for (size_t i = 0; i < act->geometry_handlers.size (); ++i)
          geometry_handlers_list.push_back (act->geometry_handlers[i]->getFieldName ());
        for (size_t i = 0; i < act->color_handlers.size (); ++i)
          color_handlers_list.push_back (act->color_handlers[i]->getFieldName ());

        if (!geometry_handlers_list.empty ())
        {
          int i = 0;
          pcl::console::print_info ("List of available geometry handlers for actor "); pcl::console::print_value ("%s: ", (*it).first.c_str ());
          for (std::list<std::string>::iterator git = geometry_handlers_list.begin (); git != geometry_handlers_list.end (); ++git)
            pcl::console::print_value ("%s(%d) ", (*git).c_str (), ++i);
          pcl::console::print_info ("\n");
        }
        if (!color_handlers_list.empty ())
        {
          int i = 0;
          pcl::console::print_info ("List of available color handlers for actor "); pcl::console::print_value ("%s: ", (*it).first.c_str ());
          for (std::list<std::string>::iterator cit = color_handlers_list.begin (); cit != color_handlers_list.end (); ++cit)
            pcl::console::print_value ("%s(%d) ", (*cit).c_str (), ++i);
          pcl::console::print_info ("\n");
        }
      }
      
      break;
    }

    // Switch representation to points
    case 'p': case 'P':
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = (vtkActor*)path->GetLastNode ()->GetViewProp ();
          apart->GetProperty ()->SetRepresentationToPoints ();
        }
      }
      break;
    }
    // Save a PNG snapshot with the current screen
    case 'j': case 'J':
    {
      wif_->Modified ();      // Update the WindowToImageFilter
      snapshot_writer_->Modified ();
      char cam_fn[80], snapshot_fn[80];
      unsigned t = time (0);
      sprintf (snapshot_fn, "screenshot-%d.png" , t);
      snapshot_writer_->SetFileName (snapshot_fn);
      snapshot_writer_->Write ();

      sprintf (cam_fn, "screenshot-%d.cam", t);
      ofstream ofs_cam;
      ofs_cam.open (cam_fn);
      vtkSmartPointer<vtkCamera> cam = Interactor->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
      double clip[2], focal[3], pos[3], view[3];
      cam->GetClippingRange (clip);
      cam->GetFocalPoint (focal);
      cam->GetPosition (pos);
      cam->GetViewUp (view);
      int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
      int *win_size = Interactor->GetRenderWindow ()->GetSize ();
      ofs_cam << clip[0]  << "," << clip[1]  << "/" << focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
                 pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" << view[0]  << "," << view[1]  << "," << view[2] << "/" << 
                 win_size[0] << "," << win_size[1] << "/" << win_pos[0] << "," << win_pos[1]
              << endl;
      ofs_cam.close ();

      pcl::console::print_info ("Screenshot (%s) and camera information (%s) successfully captured.\n", snapshot_fn, cam_fn);
      break;
    }
    // display current camera settings/parameters
    case 'c': case 'C':
    {
      vtkSmartPointer<vtkCamera> cam = Interactor->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
      double clip[2], focal[3], pos[3], view[3];
      cam->GetClippingRange (clip);
      cam->GetFocalPoint (focal);
      cam->GetPosition (pos);
      cam->GetViewUp (view);
      int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
      int *win_size = Interactor->GetRenderWindow ()->GetSize ();
      std::cerr << clip[0]  << "," << clip[1]  << "/" << focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
                   pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" << view[0]  << "," << view[1]  << "," << view[2] << "/" <<
                   win_size[0] << "," << win_size[1] << "/" << win_pos[0] << "," << win_pos[1]
                << endl;
      break;
    }
    case 43:        // KEY_PLUS
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = (vtkActor*)path->GetLastNode ()->GetViewProp ();
          int psize = apart->GetProperty ()->GetPointSize ();
          if (psize < 63)
            apart->GetProperty ()->SetPointSize (psize + 1);
        }
      }
      break;
    }
    case 45:        // KEY_MINUS
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = (vtkActor*)path->GetLastNode ()->GetViewProp ();
          int psize = apart->GetProperty ()->GetPointSize ();
          if (psize > 1)
            apart->GetProperty ()->SetPointSize (psize - 1);
        }
      }
      break;
    }
    // Switch between maximize and original window size
    case 'f': case 'F':
    {
      if (alt)
      {
        // Get screen size
        int *scr_size = Interactor->GetRenderWindow ()->GetScreenSize ();
        // Get window size
        int *win_size = Interactor->GetRenderWindow ()->GetSize ();
        // Is window size = max?
        if (win_size[0] == max_win_height_ && win_size[1] == max_win_width_)
        {
          // Set the previously saved 'current' window size
          Interactor->GetRenderWindow ()->SetSize (win_height_, win_width_);
          // Set the previously saved window position
          Interactor->GetRenderWindow ()->SetPosition (win_pos_x_, win_pos_y_);
          Interactor->GetRenderWindow ()->Render ();
          Interactor->Render ();
        }
        // Set to max
        else
        {
          int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
          // Save the current window position
          win_pos_x_  = win_pos[0];
          win_pos_y_  = win_pos[1];
          // Save the current window size
          win_height_ = win_size[0];
          win_width_  = win_size[1];
          // Set the maximum window size
          Interactor->GetRenderWindow ()->SetSize (scr_size[0], scr_size[1]);
          Interactor->GetRenderWindow ()->Render ();
          Interactor->Render ();
          int *win_size = Interactor->GetRenderWindow ()->GetSize ();
          // Save the maximum window size
          max_win_height_ = win_size[0];
          max_win_width_  = win_size[1];
        }
      }
      else
      {
        AnimState = VTKIS_ANIM_ON;
        vtkAssemblyPath *path = NULL;
        Interactor->GetPicker ()->Pick (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1], 0.0, CurrentRenderer);
        vtkAbstractPropPicker *picker;
        if ((picker = vtkAbstractPropPicker::SafeDownCast (Interactor->GetPicker ())))
          path = picker->GetPath ();
        if (path != NULL)
          Interactor->FlyTo (CurrentRenderer, picker->GetPickPosition ());
        AnimState = VTKIS_ANIM_OFF;
      }
      break;
    }
    // 's'/'S' w/out ALT
    case 's': case 'S':
    {
      if (alt)
      {
        int stereo_render = Interactor->GetRenderWindow ()->GetStereoRender ();
        Interactor->GetRenderWindow ()->SetStereoRender (!stereo_render);
        Interactor->GetRenderWindow ()->Render ();
        Interactor->Render ();
      }
      else
        Superclass::OnChar ();
      break;
    } 

    // Display a grid/scale over the screen               
    case 'g': case 'G':
    {
      if (!grid_enabled_)
      {
        grid_actor_->TopAxisVisibilityOn ();
        CurrentRenderer->AddViewProp (grid_actor_);
        grid_enabled_ = true;
      }
      else
      { 
        CurrentRenderer->RemoveViewProp (grid_actor_);
        grid_enabled_ = false;
      }
      break;
    }

    // Display a LUT actor on screen               
    case 'u': case 'U':
    {
      CloudActorMap::iterator it;
      for (it = actors_->begin (); it != actors_->end (); ++it)
      {
        CloudActor *act = &(*it).second;

        vtkScalarsToColors* lut = act->actor->GetMapper ()->GetLookupTable ();
        lut_actor_->SetLookupTable (lut);
        lut_actor_->Modified ();
      }
      if (!lut_enabled_)
      {
        CurrentRenderer->AddActor (lut_actor_);
        lut_actor_->SetVisibility (true);
        lut_enabled_ = true;
      }
      else
      { 
        CurrentRenderer->RemoveActor (lut_actor_);
        lut_enabled_ = false;
      }
      CurrentRenderer->Render ();
      break;
    }

    // Overwrite the camera reset
    case 'r': case 'R':
    {
      if (!alt)
      {
        Superclass::OnChar ();
        break;
      }
      // Get all the data
      double bounds[6];
      double focal[3];
      CurrentRenderer->ComputeVisiblePropBounds (bounds);
      focal[0] = (bounds[0] + bounds[1]) / 2.0;
      focal[1] = (bounds[2] + bounds[3]) / 2.0;
      focal[2] = (bounds[4] + bounds[5]) / 2.0;

      vtkSmartPointer<vtkCamera> cam = CurrentRenderer->GetActiveCamera ();
      double view[3];
      cam->SetFocalPoint (focal);
      cam->SetPosition (0 - .25 * focal[0], 0 - .25 * focal[1], 0 - .25 * focal[2]);
      cam->GetViewUp (view);

      // Dataset negative on Z?
      if (focal[2] > 0)
        for (int i = 0; i < 3; i++) view[i] *= -1;
      cam->SetViewUp (view[0], view[1], view[2]);
      CurrentRenderer->SetActiveCamera (cam);
      CurrentRenderer->ResetCameraClippingRange (bounds);
      CurrentRenderer->Render ();
      break;
    }

    default:
    {
      Superclass::OnChar ();
      break;
    }
  }
  rens_->Render ();
  Interactor->Render ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Interactor style internal method. Gets called periodically if a timer is set. */
void
  pcl_visualization::PCLVisualizerInteractorStyle::OnTimer ()
{
  if (!init_)
  {
    pcl::console::print_error ("[PCLVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
    return;
  }

  if (!rens_)
  {
    pcl::console::print_error ("[PCLVisualizerInteractorStyle] No renderer collection given! Use SetRendererCollection () before continuing.\n");
    return;
  }
  rens_->Render ();
  Interactor->Render ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Initialization routine. Must be called before anything else. */
void
  pcl_visualization::PCLHistogramVisualizerInteractorStyle::Initialize ()
{
  // Set windows size (width, height) to unknown (-1)
  win_height_ = win_width_ = -1;
  win_pos_x_ = win_pos_y_ = 0;
  max_win_height_ = max_win_width_ = -1;

  // Create the image filter and PNG writer objects
  wif_ = vtkSmartPointer<vtkWindowToImageFilter>::New ();
  snapshot_writer_ = vtkSmartPointer<vtkPNGWriter>::New ();
  snapshot_writer_->SetInputConnection (wif_->GetOutputPort ());

  init_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Interactor style internal method. Gets called whenever a key is pressed. */
void
  pcl_visualization::PCLHistogramVisualizerInteractorStyle::OnChar ()
{
  if (!init_)
  {
    pcl::console::print_error ("[PCLHistogramVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
    return;
  }

/*  if (!wins_)
  {
    pcl::console::print_error ("[PCLHistogramVisualizerInteractorStyle] No renderer-window-interactor map given! Use setRenWinInteractMap () before continuing.\n");
    return;
  }*/

  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

  if (wif_->GetInput () == NULL)
  {
    wif_->SetInput (Interactor->GetRenderWindow ());
    wif_->Modified ();
    snapshot_writer_->Modified ();
  }

  // Save the initial windows width/height
  if (win_height_ == -1 || win_width_ == -1)
  {
    int *win_size = Interactor->GetRenderWindow ()->GetSize ();
    win_height_ = win_size[0];
    win_width_  = win_size[1];
  }

  // Get the status of special keys (Cltr+Alt+Shift)
  bool alt   = Interactor->GetAltKey     ();

  //fprintf (stderr, "Key sym: %s\n", Interactor->GetKeySym ());
  // ---[ Check the rest of the key codes 
  switch (Interactor->GetKeyCode ())
  {
    case 'h': case 'H':
    {
      pcl::console::print_info ("| Help:\n"
                  "-------\n"
                  "          p, P   : switch to a point-based representation\n"
                  "          w, W   : switch to a wireframe-based representation (where available)\n"
                  "          s, S   : switch to a surface-based representation (where available)\n"
                  "\n"
                  "          j, J   : take a .PNG snapshot of the current window view\n"
                  "          c, C   : display current camera/window parameters\n"
                  "\n"
                  "         + / -   : increment/decrement overall point size\n"
                  "\n"
                  "    ALT + f, F   : switch between maximized window mode and original size\n"
          );
      break;
    }

    // Switch representation to points
    case 'p': case 'P':
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = (vtkActor*)path->GetLastNode ()->GetViewProp ();
          apart->GetProperty ()->SetRepresentationToPoints ();
        }
      }
      break;
    }
    // Save a PNG snapshot with the current screen
    case 'j': case 'J':
    {
      char cam_fn[80], snapshot_fn[80];
      unsigned t = time (0);
      sprintf (snapshot_fn, "screenshot-%d.png" , t);
      snapshot_writer_->SetFileName (snapshot_fn);
      snapshot_writer_->Write ();

      sprintf (cam_fn, "screenshot-%d.cam", t);
      ofstream ofs_cam;
      ofs_cam.open (cam_fn);
      vtkSmartPointer<vtkCamera> cam = Interactor->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
      double clip[2], focal[3], pos[3], view[3];
      cam->GetClippingRange (clip);
      cam->GetFocalPoint (focal);
      cam->GetPosition (pos);
      cam->GetViewUp (view);
      int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
      int *win_size = Interactor->GetRenderWindow ()->GetSize ();
      ofs_cam << clip[0]  << "," << clip[1]  << "/" << focal[0] << "," << focal[1] << "," << focal[2] << "/" <<
                 pos[0]   << "," << pos[1]   << "," << pos[2]   << "/" << view[0]  << "," << view[1]  << "," << view[2] << "/" << 
                 win_size[0] << "," << win_size[1] << "/" << win_pos[0] << "," << win_pos[1]
              << endl;
      ofs_cam.close ();

      pcl::console::print_info ("Screenshot (%s) and camera information (%s) successfully captured.\n", snapshot_fn, cam_fn);
      break;
    }
    case 43:        // KEY_PLUS
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = (vtkActor*)path->GetLastNode ()->GetViewProp ();
          int psize = apart->GetProperty ()->GetPointSize ();
          if (psize < 63)
            apart->GetProperty ()->SetPointSize (psize + 1);
        }
      }
      break;
    }
    case 45:        // KEY_MINUS
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = (vtkActor*)path->GetLastNode ()->GetViewProp ();
          int psize = apart->GetProperty ()->GetPointSize ();
          if (psize > 1)
            apart->GetProperty ()->SetPointSize (psize - 1);
        }
      }
      break;
    }
    // Switch between maximize and original window size
    case 'f': case 'F':
    {
      if (alt)
      {
        // Get screen size
        int *scr_size = Interactor->GetRenderWindow ()->GetScreenSize ();
        // Get window size
        int *win_size = Interactor->GetRenderWindow ()->GetSize ();
        // Is window size = max?
        if (win_size[0] == max_win_height_ && win_size[1] == max_win_width_)
        {
          // Set the previously saved 'current' window size
          Interactor->GetRenderWindow ()->SetSize (win_height_, win_width_);
          // Set the previously saved window position
          Interactor->GetRenderWindow ()->SetPosition (win_pos_x_, win_pos_y_);
          Interactor->GetRenderWindow ()->Render ();
          Interactor->Render ();
        }
        // Set to max
        else
        {
          int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
          // Save the current window position
          win_pos_x_  = win_pos[0];
          win_pos_y_  = win_pos[1];
          // Save the current window size
          win_height_ = win_size[0];
          win_width_  = win_size[1];
          // Set the maximum window size
          Interactor->GetRenderWindow ()->SetSize (scr_size[0], scr_size[1]);
          Interactor->GetRenderWindow ()->Render ();
          Interactor->Render ();
          int *win_size = Interactor->GetRenderWindow ()->GetSize ();
          // Save the maximum window size
          max_win_height_ = win_size[0];
          max_win_width_  = win_size[1];
        }
      }
      else
        Superclass::OnChar ();
      break;
    }
    // Switch representation to wireframe
    default:
    {
      Superclass::OnChar ();
    }
  }
  Interactor->Render ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Interactor style internal method. Gets called periodically if a timer is set. */
void
  pcl_visualization::PCLHistogramVisualizerInteractorStyle::OnTimer ()
{
  if (!init_)
  {
    pcl::console::print_error ("[PCLHistogramVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
    return;
  }

  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
    (*am_it).second.ren_->Render ();
}

namespace pcl_visualization
{
  // Standard VTK macro for *New ()
  vtkStandardNewMacro (PCLVisualizerInteractorStyle);
  vtkStandardNewMacro (PCLHistogramVisualizerInteractorStyle);
}

