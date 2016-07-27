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

#include <list>
#include <pcl/visualization/common/io.h>
#include <pcl/visualization/interactor_style.h>
#include <vtkVersion.h>
#include <vtkLODActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkCellArray.h>
#include <vtkTextProperty.h>
#include <vtkAbstractPropPicker.h>
#include <vtkCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkScalarBarActor.h>
#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRendererCollection.h>
#include <vtkActorCollection.h>
#include <vtkLegendScaleActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkObjectFactory.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkAssemblyPath.h>
#include <vtkAbstractPicker.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>

#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>
#endif

#define ORIENT_MODE 0
#define SELECT_MODE 1

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::Initialize ()
{
  modifier_ = pcl::visualization::INTERACTOR_KB_MOD_ALT;
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
  wif_->ReadFrontBufferOff ();
  snapshot_writer_ = vtkSmartPointer<vtkPNGWriter>::New ();
  snapshot_writer_->SetInputConnection (wif_->GetOutputPort ());

  init_ = true;

  stereo_anaglyph_mask_default_ = true;

  // Start in orient mode
  Superclass::CurrentMode = ORIENT_MODE;

  // Add our own mouse callback before any user callback. Used for accurate point picking.
  mouse_callback_ = vtkSmartPointer<pcl::visualization::PointPickingCallback>::New ();
  AddObserver (vtkCommand::LeftButtonPressEvent, mouse_callback_);
  AddObserver (vtkCommand::LeftButtonReleaseEvent, mouse_callback_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::saveScreenshot (const std::string &file)
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
  wif_->SetInput (Interactor->GetRenderWindow ());
  wif_->Modified ();      // Update the WindowToImageFilter
  snapshot_writer_->Modified ();
  snapshot_writer_->SetFileName (file.c_str ());
  snapshot_writer_->Write ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizerInteractorStyle::saveCameraParameters (const std::string &file)
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

  ofstream ofs_cam (file.c_str ());
  if (!ofs_cam.is_open ())
  {
    return (false);
  }

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
             cam->GetViewAngle () / 180.0 * M_PI  << "/" << win_size[0] << "," << win_size[1] << "/" << win_pos[0] << "," << win_pos[1]
          << endl;
  ofs_cam.close ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::getCameraParameters (pcl::visualization::Camera &camera)
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

  vtkSmartPointer<vtkCamera> cam = Interactor->GetRenderWindow ()->GetRenderers ()->GetFirstRenderer ()->GetActiveCamera ();
  cam->GetClippingRange (camera.clip);
  cam->GetFocalPoint (camera.focal);
  cam->GetPosition (camera.pos);
  cam->GetViewUp (camera.view);
  camera.fovy = cam->GetViewAngle () / 180.0 * M_PI;
  int *win_pos = Interactor->GetRenderWindow ()->GetPosition ();
  int *win_size = Interactor->GetRenderWindow ()->GetSize ();
  camera.window_pos[0] = win_pos[0];
  camera.window_pos[1] = win_pos[1];
  camera.window_size[0] = win_size[0];
  camera.window_size[1] = win_size[1];
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizerInteractorStyle::loadCameraParameters (const std::string &file)
{
  std::ifstream fs;
  std::string line;
  std::vector<std::string> camera;
  bool ret;

  fs.open (file.c_str ());
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line == "")
      continue;

    boost::split (camera, line, boost::is_any_of ("/"), boost::token_compress_on);
    break;
  }
  fs.close ();

  ret = getCameraParameters (camera);
  if (ret)
  {
    camera_file_ = file;
  }

  return (ret);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::setCameraParameters (const Eigen::Matrix3f &intrinsics,
                                                                       const Eigen::Matrix4f &extrinsics,
                                                                       int viewport)
{
  // Position = extrinsic translation
  Eigen::Vector3f pos_vec = extrinsics.block<3, 1> (0, 3);

  // Rotate the view vector
  Eigen::Matrix3f rotation = extrinsics.block<3, 3> (0, 0);
  Eigen::Vector3f y_axis (0.f, 1.f, 0.f);
  Eigen::Vector3f up_vec (rotation * y_axis);

  // Compute the new focal point
  Eigen::Vector3f z_axis (0.f, 0.f, 1.f);
  Eigen::Vector3f focal_vec = pos_vec + rotation * z_axis;

  // Get the width and height of the image - assume the calibrated centers are at the center of the image
  Eigen::Vector2i window_size;
  window_size[0] = static_cast<int> (intrinsics (0, 2));
  window_size[1] = static_cast<int> (intrinsics (1, 2));

  // Compute the vertical field of view based on the focal length and image heigh
  double fovy = 2 * atan (window_size[1] / (2. * intrinsics (1, 1))) * 180.0 / M_PI;


  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Modify all renderer's cameras
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
      cam->SetPosition (pos_vec[0], pos_vec[1], pos_vec[2]);
      cam->SetFocalPoint (focal_vec[0], focal_vec[1], focal_vec[2]);
      cam->SetViewUp (up_vec[0], up_vec[1], up_vec[2]);
      cam->SetUseHorizontalViewAngle (0);
      cam->SetViewAngle (fovy);
      cam->SetClippingRange (0.01, 1000.01);
      win_->SetSize (window_size[0], window_size[1]);
    }
    ++i;
  }
  win_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::setCameraParameters (const pcl::visualization::Camera &camera, int viewport)
{
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 0;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Modify all renderer's cameras
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkCamera> cam = renderer->GetActiveCamera ();
      cam->SetPosition (camera.pos[0], camera.pos[1], camera.pos[2]);
      cam->SetFocalPoint (camera.focal[0], camera.focal[1], camera.focal[2]);
      cam->SetViewUp (camera.view[0], camera.view[1], camera.view[2]);
      cam->SetClippingRange (camera.clip);
      cam->SetUseHorizontalViewAngle (0);
      cam->SetViewAngle (camera.fovy * 180.0 / M_PI);

      win_->SetSize (static_cast<int> (camera.window_size[0]),
                     static_cast<int> (camera.window_size[1]));
    }
    ++i;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::zoomIn ()
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
  // Zoom in
  StartDolly ();
  double factor = 10.0 * 0.2 * .5;
  Dolly (pow (1.1, factor));
  EndDolly ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::zoomOut ()
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
  // Zoom out
  StartDolly ();
  double factor = 10.0 * -0.2 * .5;
  Dolly (pow (1.1, factor));
  EndDolly ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::PCLVisualizerInteractorStyle::getCameraParameters (const std::vector<std::string> &camera)
{
  pcl::visualization::Camera camera_temp;

  // look for '/' as a separator
  if (camera.size () != 7)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Camera parameters given, but with an invalid number of options (%lu vs 7)!\n", static_cast<unsigned long> (camera.size ()));
    return (false);
  }

  std::string clip_str  = camera.at (0);
  std::string focal_str = camera.at (1);
  std::string pos_str   = camera.at (2);
  std::string view_str  = camera.at (3);
  std::string fovy_str  = camera.at (4);
  std::string win_size_str = camera.at (5);
  std::string win_pos_str  = camera.at (6);

  // Get each camera setting separately and parse for ','
  std::vector<std::string> clip_st;
  boost::split (clip_st, clip_str, boost::is_any_of (","), boost::token_compress_on);
  if (clip_st.size () != 2)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera clipping angle!\n");
    return (false);
  }
  camera_temp.clip[0] = atof (clip_st.at (0).c_str ());
  camera_temp.clip[1] = atof (clip_st.at (1).c_str ());

  std::vector<std::string> focal_st;
  boost::split (focal_st, focal_str, boost::is_any_of (","), boost::token_compress_on);
  if (focal_st.size () != 3)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera focal point!\n");
    return (false);
  }
  camera_temp.focal[0] = atof (focal_st.at (0).c_str ());
  camera_temp.focal[1] = atof (focal_st.at (1).c_str ());
  camera_temp.focal[2] = atof (focal_st.at (2).c_str ());

  std::vector<std::string> pos_st;
  boost::split (pos_st, pos_str, boost::is_any_of (","), boost::token_compress_on);
  if (pos_st.size () != 3)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera position!\n");
    return (false);
  }
  camera_temp.pos[0] = atof (pos_st.at (0).c_str ());
  camera_temp.pos[1] = atof (pos_st.at (1).c_str ());
  camera_temp.pos[2] = atof (pos_st.at (2).c_str ());

  std::vector<std::string> view_st;
  boost::split (view_st, view_str, boost::is_any_of (","), boost::token_compress_on);
  if (view_st.size () != 3)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for camera viewup!\n");
    return (false);
  }
  camera_temp.view[0] = atof (view_st.at (0).c_str ());
  camera_temp.view[1] = atof (view_st.at (1).c_str ());
  camera_temp.view[2] = atof (view_st.at (2).c_str ());

  std::vector<std::string> fovy_size_st;
  boost::split (fovy_size_st, fovy_str, boost::is_any_of (","), boost::token_compress_on);
  if (fovy_size_st.size () != 1)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for field of view angle!\n");
    return (false);
  }
  camera_temp.fovy = atof (fovy_size_st.at (0).c_str ());

  std::vector<std::string> win_size_st;
  boost::split (win_size_st, win_size_str, boost::is_any_of (","), boost::token_compress_on);
  if (win_size_st.size () != 2)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for window size!\n");
    return (false);
  }
  camera_temp.window_size[0] = atof (win_size_st.at (0).c_str ());
  camera_temp.window_size[1] = atof (win_size_st.at (1).c_str ());

  std::vector<std::string> win_pos_st;
  boost::split (win_pos_st, win_pos_str, boost::is_any_of (","), boost::token_compress_on);
  if (win_pos_st.size () != 2)
  {
    pcl::console::print_error ("[PCLVisualizer::getCameraParameters] Invalid parameters given for window position!\n");
    return (false);
  }
  camera_temp.window_pos[0] = atof (win_pos_st.at (0).c_str ());
  camera_temp.window_pos[1] = atof (win_pos_st.at (1).c_str ());

  setCameraParameters (camera_temp);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnChar ()
{
  // Make sure we ignore the same events we handle in OnKeyDown to avoid calling things twice
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
  if (Interactor->GetKeyCode () >= '0' && Interactor->GetKeyCode () <= '9')
    return;
  std::string key (Interactor->GetKeySym ());
  if (key.find ("XF86ZoomIn") != std::string::npos)
    zoomIn ();
  else if (key.find ("XF86ZoomOut") != std::string::npos)
    zoomOut ();

  bool keymod = false;
  switch (modifier_)
  {
    case INTERACTOR_KB_MOD_ALT:
    {
      keymod = Interactor->GetAltKey ();
      break;
    }
    case INTERACTOR_KB_MOD_CTRL:
    {
      keymod = Interactor->GetControlKey ();
      break;
    }
    case INTERACTOR_KB_MOD_SHIFT:
    {
      keymod = Interactor->GetShiftKey ();
      break;
    }
  }

  switch (Interactor->GetKeyCode ())
  {
    // All of the options below simply exit
    case 'h': case 'H':
    case 'l': case 'L':
    case 'p': case 'P':
    case 'j': case 'J':
    case 'c': case 'C':
    case 43:        // KEY_PLUS
    case 45:        // KEY_MINUS
    case 'f': case 'F':
    case 'g': case 'G':
    case 'o': case 'O':
    case 'u': case 'U':
    case 'q': case 'Q':
    case 'x': case 'X':
    case 'r': case 'R':
    {
      break;
    }
    // S have special !ALT case
    case 's': case 'S':
    {
      if (!keymod)
        Superclass::OnChar ();
      break;
    }
    default:
    {
      Superclass::OnChar ();
      break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizerInteractorStyle::registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> callback)
{
  return (mouse_signal_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizerInteractorStyle::registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> callback)
{
  return (keyboard_signal_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizerInteractorStyle::registerPointPickingCallback (boost::function<void (const pcl::visualization::PointPickingEvent&)> callback)
{
  return (point_picking_signal_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection
pcl::visualization::PCLVisualizerInteractorStyle::registerAreaPickingCallback (boost::function<void (const pcl::visualization::AreaPickingEvent&)> callback)
{
  return (area_picking_signal_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnKeyDown ()
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
  bool shift = Interactor->GetShiftKey   ();
  bool ctrl  = Interactor->GetControlKey ();
  bool alt   = Interactor->GetAltKey ();

  bool keymod = false;
  switch (modifier_)
  {
    case INTERACTOR_KB_MOD_ALT:
    {
      keymod = alt;
      break;
    }
    case INTERACTOR_KB_MOD_CTRL:
    {
      keymod = ctrl;
      break;
    }
    case INTERACTOR_KB_MOD_SHIFT:
    {
      keymod = shift;
      break;
    }
  }

  // ---[ Check the rest of the key codes

  // Save camera parameters
  if ((Interactor->GetKeySym ()[0] == 'S' || Interactor->GetKeySym ()[0] == 's') && ctrl && !alt && !shift)
  {
    if (camera_file_.empty ())
    {
      getCameraParameters (camera_);
      camera_saved_ = true;
      pcl::console::print_info ("Camera parameters saved, you can press CTRL + R to restore.\n");
    }
    else
    {
      if (saveCameraParameters (camera_file_))
      {
        pcl::console::print_info ("Save camera parameters to %s, you can press CTRL + R to restore.\n", camera_file_.c_str ());
      }
      else
      {
        pcl::console::print_error ("[PCLVisualizerInteractorStyle] Can't save camera parameters to file: %s.\n", camera_file_.c_str ());
      }
    }
  }

  // Restore camera parameters
  if ((Interactor->GetKeySym ()[0] == 'R' || Interactor->GetKeySym ()[0] == 'r') && ctrl && !alt && !shift)
  {
    if (camera_file_.empty ())
    {
      if (camera_saved_)
      {
        setCameraParameters (camera_);
        pcl::console::print_info ("Camera parameters restored.\n");
      }
      else
      {
        pcl::console::print_info ("No camera parameters saved for restoring.\n");
      }
    }
    else
    {
      if (boost::filesystem::exists (camera_file_))
      {
        if (loadCameraParameters (camera_file_))
        {
          pcl::console::print_info ("Restore camera parameters from %s.\n", camera_file_.c_str ());
        }
        else
        {
          pcl::console::print_error ("Can't restore camera parameters from file: %s.\n", camera_file_.c_str ());
        }
      }
      else
      {
        pcl::console::print_info ("No camera parameters saved in %s for restoring.\n", camera_file_.c_str ());
      }
    }
  }

  // Switch between point color/geometry handlers
  if (Interactor->GetKeySym () && Interactor->GetKeySym ()[0]  >= '0' && Interactor->GetKeySym ()[0] <= '9')
  {
    CloudActorMap::iterator it;
    int index = Interactor->GetKeySym ()[0] - '0' - 1;
    if (index == -1) index = 9;

    // Add 10 more for CTRL+0..9 keys
    if (ctrl)
      index += 10;

    // Geometry ?
    if (keymod)
    {
      for (it = cloud_actors_->begin (); it != cloud_actors_->end (); ++it)
      {
        CloudActor *act = &(*it).second;
        if (index >= static_cast<int> (act->geometry_handlers.size ()))
          continue;

        // Save the geometry handler index for later usage
        act->geometry_handler_index_ = index;

        // Create the new geometry
        PointCloudGeometryHandler<pcl::PCLPointCloud2>::ConstPtr geometry_handler = act->geometry_handlers[index];

        // Use the handler to obtain the geometry
        vtkSmartPointer<vtkPoints> points;
        geometry_handler->getGeometry (points);

        // Set the vertices
        vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
        for (vtkIdType i = 0; i < static_cast<vtkIdType> (points->GetNumberOfPoints ()); ++i)
          vertices->InsertNextCell (static_cast<vtkIdType>(1), &i);

        // Create the data
        vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New ();
        data->SetPoints (points);
        data->SetVerts (vertices);
        // Modify the mapper
#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
        if (use_vbos_)
        {
          vtkVertexBufferObjectMapper* mapper = static_cast<vtkVertexBufferObjectMapper*>(act->actor->GetMapper ());
          mapper->SetInput (data);
          // Modify the actor
          act->actor->SetMapper (mapper);
        }
        else
#endif
        {
          vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(act->actor->GetMapper ());
#if VTK_MAJOR_VERSION < 6
          mapper->SetInput (data);
#else
          mapper->SetInputData (data);
#endif
          // Modify the actor
          act->actor->SetMapper (mapper);
        }
        act->actor->Modified ();
      }
    }
    else
    {
      for (it = cloud_actors_->begin (); it != cloud_actors_->end (); ++it)
      {
        CloudActor *act = &(*it).second;
        // Check for out of bounds
        if (index >= static_cast<int> (act->color_handlers.size ()))
          continue;

        // Save the color handler index for later usage
        act->color_handler_index_ = index;

        // Get the new color
        PointCloudColorHandler<pcl::PCLPointCloud2>::ConstPtr color_handler = act->color_handlers[index];

        vtkSmartPointer<vtkDataArray> scalars;
        color_handler->getColor (scalars);
        double minmax[2];
        scalars->GetRange (minmax);
        // Update the data
        vtkPolyData *data = static_cast<vtkPolyData*>(act->actor->GetMapper ()->GetInput ());
        data->GetPointData ()->SetScalars (scalars);
        // Modify the mapper
#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
        if (use_vbos_)
        {
          vtkVertexBufferObjectMapper* mapper = static_cast<vtkVertexBufferObjectMapper*>(act->actor->GetMapper ());
          mapper->SetScalarRange (minmax);
          mapper->SetScalarModeToUsePointData ();
          mapper->SetInput (data);
          // Modify the actor
          act->actor->SetMapper (mapper);
        }
        else
#endif
        {
          vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(act->actor->GetMapper ());
          mapper->SetScalarRange (minmax);
          mapper->SetScalarModeToUsePointData ();
#if VTK_MAJOR_VERSION < 6
          mapper->SetInput (data);
#else
          mapper->SetInputData (data);
#endif
          // Modify the actor
          act->actor->SetMapper (mapper);
        }
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
                  "          e, E   : exit the interactor\n"
                  "          q, Q   : stop and call VTK's TerminateApp\n"
                  "\n"
                  "           +/-   : increment/decrement overall point size\n"
                  "     +/- [+ ALT] : zoom in/out \n"
                  "\n"
                  "          g, G   : display scale grid (on/off)\n"
                  "          u, U   : display lookup table (on/off)\n"
                  "\n"
                  "    o, O         : switch between perspective/parallel projection (default = perspective)\n"
                  "    r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]\n"
                  "    CTRL + s, S  : save camera parameters\n"
                  "    CTRL + r, R  : restore camera parameters\n"
                  "\n"
                  "    ALT + s, S   : turn stereo mode on/off\n"
                  "    ALT + f, F   : switch between maximized window mode and original size\n"
                  "\n"
                  "          l, L           : list all available geometric and color handlers for the current actor map\n"
                  "    ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)\n"
                  "          0..9 [+ CTRL]  : switch between different color handlers (where available)\n"
                  "\n"
                  "    SHIFT + left click   : select a point (start with -use_point_picking)\n"
                  "\n"
                  "          x, X   : toggle rubber band selection mode for left mouse button\n"
          );
      break;
    }

    // Get the list of available handlers
    case 'l': case 'L':
    {
      // Iterate over the entire actors list and extract the geomotry/color handlers list
      for (CloudActorMap::iterator it = cloud_actors_->begin (); it != cloud_actors_->end (); ++it)
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
          vtkSmartPointer<vtkActor> apart = reinterpret_cast <vtkActor*> (path->GetLastNode ()->GetViewProp ());
          apart->GetProperty ()->SetRepresentationToPoints ();
        }
      }
      break;
    }

    // Switch representation to wireframe (override default behavior)
    case 'w': case 'W':
    {
      vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
      vtkCollectionSimpleIterator ait;
      for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
      {
        for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
        {
          vtkSmartPointer<vtkActor> apart = reinterpret_cast <vtkActor*> (path->GetLastNode ()->GetViewProp ());
          apart->GetProperty ()->SetRepresentationToWireframe ();
          apart->GetProperty ()->SetLighting (false);
        }
      }
      break;
    }

    // Save a PNG snapshot with the current screen
    case 'j': case 'J':
    {
      char cam_fn[80], snapshot_fn[80];
      unsigned t = static_cast<unsigned> (time (0));
      sprintf (snapshot_fn, "screenshot-%d.png" , t);
      saveScreenshot (snapshot_fn);

      sprintf (cam_fn, "screenshot-%d.cam", t);
      saveCameraParameters (cam_fn);

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
      std::cerr <<  "Clipping plane [near,far] "  << clip[0] << ", " << clip[1] << endl <<
                    "Focal point [x,y,z] " << focal[0] << ", " << focal[1] << ", " << focal[2] << endl <<
                    "Position [x,y,z] " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl <<
                    "View up [x,y,z] " << view[0]  << ", " << view[1]  << ", " << view[2] << endl <<
                    "Camera view angle [degrees] " << cam->GetViewAngle () << endl <<
                    "Window size [x,y] " << win_size[0] << ", " << win_size[1] << endl <<
                    "Window position [x,y] " << win_pos[0] << ", " << win_pos[1] << endl;
      break;
    }
    case '=':
    {
      zoomIn();
      break;
    }
    case 43:        // KEY_PLUS
    {
      if(alt)
        zoomIn ();
      else
      {
        vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
        vtkCollectionSimpleIterator ait;
        for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
        {
          for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
          {
            vtkSmartPointer<vtkActor> apart = reinterpret_cast <vtkActor*> (path->GetLastNode ()->GetViewProp ());
            float psize = apart->GetProperty ()->GetPointSize ();
            if (psize < 63.0f)
              apart->GetProperty ()->SetPointSize (psize + 1.0f);
          }
        }
      }
      break;
    }
    case 45:        // KEY_MINUS
    {
      if(alt)
        zoomOut ();
      else
      {
        vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors ();
        vtkCollectionSimpleIterator ait;
        for (ac->InitTraversal (ait); vtkActor* actor = ac->GetNextActor (ait); )
        {
          for (actor->InitPathTraversal (); vtkAssemblyPath* path = actor->GetNextPath (); )
          {
            vtkSmartPointer<vtkActor> apart = static_cast<vtkActor*> (path->GetLastNode ()->GetViewProp ());
            float psize = apart->GetProperty ()->GetPointSize ();
            if (psize > 1.0f)
              apart->GetProperty ()->SetPointSize (psize - 1.0f);
          }
        }
      }
      break;
    }
    // Switch between maximize and original window size
    case 'f': case 'F':
    {
      if (keymod)
      {
        // Get screen size
        int *temp = Interactor->GetRenderWindow ()->GetScreenSize ();
        int scr_size[2]; scr_size[0] = temp[0]; scr_size[1] = temp[1];

        // Get window size
        temp = Interactor->GetRenderWindow ()->GetSize ();
        int win_size[2]; win_size[0] = temp[0]; win_size[1] = temp[1];
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
      if (keymod)
      {
        int stereo_render = Interactor->GetRenderWindow ()->GetStereoRender ();
        if (!stereo_render)
        {
          if (stereo_anaglyph_mask_default_)
          {
            Interactor->GetRenderWindow ()->SetAnaglyphColorMask (4, 3);
            stereo_anaglyph_mask_default_ = false;
          }
          else
          {
            Interactor->GetRenderWindow ()->SetAnaglyphColorMask (2, 5);
            stereo_anaglyph_mask_default_ = true;
          }
        }
        Interactor->GetRenderWindow ()->SetStereoRender (!stereo_render);
        Interactor->GetRenderWindow ()->Render ();
        Interactor->Render ();
      }
      else
      {
        Superclass::OnKeyDown();
        vtkSmartPointer<vtkActorCollection> ac = CurrentRenderer->GetActors();
        vtkCollectionSimpleIterator ait;
        for (ac->InitTraversal(ait); vtkActor* actor = ac->GetNextActor(ait);)
        {
          for (actor->InitPathTraversal(); vtkAssemblyPath* path = actor->GetNextPath();)
          {
            vtkSmartPointer<vtkActor> apart = reinterpret_cast<vtkActor*>(path->GetLastNode()->GetViewProp());
            apart->GetProperty()->SetRepresentationToSurface();
            apart->GetProperty()->SetLighting(true);
          }
        }
      }
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

    case 'o': case 'O':
    {
      vtkSmartPointer<vtkCamera> cam = CurrentRenderer->GetActiveCamera ();
      int flag = cam->GetParallelProjection ();
      cam->SetParallelProjection (!flag);

      CurrentRenderer->SetActiveCamera (cam);
      CurrentRenderer->Render ();
      break;
    }
    // Display a LUT actor on screen
    case 'u': case 'U':
    {
      updateLookUpTableDisplay (true);
      break;
    }

    // Overwrite the camera reset
    case 'r': case 'R':
    {
      if (!keymod)
      {
        FindPokedRenderer(Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);
        if(CurrentRenderer != 0)
          CurrentRenderer->ResetCamera ();
        else
          PCL_WARN ("no current renderer on the interactor style.");

        CurrentRenderer->Render ();
        break;
      }

      vtkSmartPointer<vtkCamera> cam = CurrentRenderer->GetActiveCamera ();
      
      static CloudActorMap::iterator it = cloud_actors_->begin ();
      // it might be that some actors don't have a valid transformation set -> we skip them to avoid a seg fault.
      bool found_transformation = false;
      for (unsigned idx = 0; idx < cloud_actors_->size (); ++idx, ++it)
      {
        if (it == cloud_actors_->end ())
          it = cloud_actors_->begin ();
        
        const CloudActor& actor = it->second;
        if (actor.viewpoint_transformation_.GetPointer ())
        {
          found_transformation = true;
          break;
        }
      }
      
      // if a valid transformation was found, use it otherwise fall back to default view point.
      if (found_transformation)
      {
        const CloudActor& actor = it->second;
        cam->SetPosition (actor.viewpoint_transformation_->GetElement (0, 3),
                          actor.viewpoint_transformation_->GetElement (1, 3),
                          actor.viewpoint_transformation_->GetElement (2, 3));

        cam->SetFocalPoint (actor.viewpoint_transformation_->GetElement (0, 3) - actor.viewpoint_transformation_->GetElement (0, 2),
                            actor.viewpoint_transformation_->GetElement (1, 3) - actor.viewpoint_transformation_->GetElement (1, 2),
                            actor.viewpoint_transformation_->GetElement (2, 3) - actor.viewpoint_transformation_->GetElement (2, 2));

        cam->SetViewUp (actor.viewpoint_transformation_->GetElement (0, 1),
                        actor.viewpoint_transformation_->GetElement (1, 1),
                        actor.viewpoint_transformation_->GetElement (2, 1));
      }
      else
      {
        cam->SetPosition (0, 0, 0);
        cam->SetFocalPoint (0, 0, 1);
        cam->SetViewUp (0, -1, 0);
      }

      // go to the next actor for the next key-press event.
      if (it != cloud_actors_->end ())
        ++it;
      else
        it = cloud_actors_->begin ();
      
      CurrentRenderer->SetActiveCamera (cam);
      CurrentRenderer->ResetCameraClippingRange ();
      CurrentRenderer->Render ();
      break;
    }

    case 'x' : case 'X' :
    {
      CurrentMode = (CurrentMode == ORIENT_MODE) ? SELECT_MODE : ORIENT_MODE;
      if (CurrentMode == SELECT_MODE)
      {
        // Save the point picker
        point_picker_ = static_cast<vtkPointPicker*> (Interactor->GetPicker ());
        // Switch for an area picker
        vtkSmartPointer<vtkAreaPicker> area_picker = vtkSmartPointer<vtkAreaPicker>::New ();
        Interactor->SetPicker (area_picker);
      }
      else
      {
        // Restore point picker
        Interactor->SetPicker (point_picker_);
      }
      break;
    }

    case 'q': case 'Q':
    {
      Interactor->ExitCallback ();
      return;
    }
    default:
    {
      Superclass::OnKeyDown ();
      break;
    }
  }

  KeyboardEvent event (true, Interactor->GetKeySym (), Interactor->GetKeyCode (), Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey ());
  keyboard_signal_ (event);

  rens_->Render ();
  Interactor->Render ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Update the look up table displayed when 'u' is pressed
void
pcl::visualization::PCLVisualizerInteractorStyle::updateLookUpTableDisplay (bool add_lut)
{
  CloudActorMap::iterator am_it;
  ShapeActorMap::iterator sm_it;
  bool actor_found = false;

  if (!lut_enabled_ && !add_lut)
    return;

  if (lut_actor_id_ != "")  // Search if provided actor id is in CloudActorMap or ShapeActorMap
  {
    am_it = cloud_actors_->find (lut_actor_id_);
    if (am_it == cloud_actors_->end ())
    {
      sm_it = shape_actors_->find (lut_actor_id_);
      if (sm_it == shape_actors_->end ())
      {
        PCL_WARN ("[updateLookUpTableDisplay] Could not find any actor with id <%s>!\n", lut_actor_id_.c_str ());
        if (lut_enabled_)
        {  // Remove LUT and exit
          CurrentRenderer->RemoveActor (lut_actor_);
          lut_enabled_ = false;
        }
        return;
      }

      // ShapeActor found
      vtkSmartPointer<vtkProp> *act = & (*sm_it).second;
      vtkSmartPointer<vtkActor> actor = vtkActor::SafeDownCast (*act);
      if (!actor || !actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
      {
        PCL_WARN ("[updateLookUpTableDisplay] id <%s> does not hold any color information!\n", lut_actor_id_.c_str ());
        if (lut_enabled_)
        {  // Remove LUT and exit
          CurrentRenderer->RemoveActor (lut_actor_);
          lut_enabled_ = false;
        }
        return;
      }

      lut_actor_->SetLookupTable (actor->GetMapper ()->GetLookupTable ());
      lut_actor_->Modified ();
      actor_found = true;
    }
    else
    {
      // CloudActor
      CloudActor *act = & (*am_it).second;
      if (!act->actor->GetMapper ()->GetLookupTable () && !act->actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
      {
        PCL_WARN ("[updateLookUpTableDisplay] id <%s> does not hold any color information!\n", lut_actor_id_.c_str ());
        if (lut_enabled_)
        {  // Remove LUT and exit
          CurrentRenderer->RemoveActor (lut_actor_);
          lut_enabled_ = false;
        }
        return;
      }

      vtkScalarsToColors* lut = act->actor->GetMapper ()->GetLookupTable ();
      lut_actor_->SetLookupTable (lut);
      lut_actor_->Modified ();
      actor_found = true;
    }
  }
  else  // lut_actor_id_ == "", the user did not specify which cloud/shape LUT should be displayed
  // Circling through all clouds/shapes and displaying first LUT found
  {
    for (am_it = cloud_actors_->begin (); am_it != cloud_actors_->end (); ++am_it)
    {
      CloudActor *act = & (*am_it).second;
      if (!act->actor->GetMapper ()->GetLookupTable ())
        continue;

      if (!act->actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())
        continue;

      vtkScalarsToColors* lut = act->actor->GetMapper ()->GetLookupTable ();
      lut_actor_->SetLookupTable (lut);
      lut_actor_->Modified ();
      actor_found = true;
      break;
    }

    if (!actor_found)
    {
      for (sm_it = shape_actors_->begin (); sm_it != shape_actors_->end (); ++sm_it)
      {
        vtkSmartPointer<vtkProp> *act = & (*sm_it).second;
        vtkSmartPointer<vtkActor> actor = vtkActor::SafeDownCast (*act);
        if (!actor)
          continue;

        if (!actor->GetMapper ()->GetInput ()->GetPointData ()->GetScalars ())  // Check if actor has scalars
          continue;
        lut_actor_->SetLookupTable (actor->GetMapper ()->GetLookupTable ());
        lut_actor_->Modified ();
        actor_found = true;
        break;
      }
    }
  }

  if ( (!actor_found && lut_enabled_) || (lut_enabled_ && add_lut))  // Remove actor
  {
    CurrentRenderer->RemoveActor (lut_actor_);
    lut_enabled_ = false;
  }
  else if (!lut_enabled_ && add_lut && actor_found)  // Add actor
  {
    CurrentRenderer->AddActor (lut_actor_);
    lut_actor_->SetVisibility (true);
    lut_enabled_ = true;
  }
  else if (lut_enabled_)  // Update actor (if displayed)
  {
    CurrentRenderer->RemoveActor (lut_actor_);
    CurrentRenderer->AddActor (lut_actor_);
  }
  else
    return;

  CurrentRenderer->Render ();
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnKeyUp ()
{
  KeyboardEvent event (false, Interactor->GetKeySym (), Interactor->GetKeyCode (), Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey ());
  keyboard_signal_ (event);
  Superclass::OnKeyUp ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnMouseMove ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseMove, MouseEvent::NoButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
  mouse_signal_ (event);
  Superclass::OnMouseMove ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnLeftButtonDown ()
{

  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];

  if (Interactor->GetRepeatCount () == 0)
  {
    MouseEvent event (MouseEvent::MouseButtonPress, MouseEvent::LeftButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
    mouse_signal_ (event);
  }
  else
  {
    MouseEvent event (MouseEvent::MouseDblClick, MouseEvent::LeftButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
    mouse_signal_ (event);
  }
  Superclass::OnLeftButtonDown ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnLeftButtonUp ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseButtonRelease, MouseEvent::LeftButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
  mouse_signal_ (event);
  Superclass::OnLeftButtonUp ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnMiddleButtonDown ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  if (Interactor->GetRepeatCount () == 0)
  {
    MouseEvent event (MouseEvent::MouseButtonPress, MouseEvent::MiddleButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
    mouse_signal_ (event);
  }
  else
  {
    MouseEvent event (MouseEvent::MouseDblClick, MouseEvent::MiddleButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
    mouse_signal_ (event);
  }
  Superclass::OnMiddleButtonDown ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnMiddleButtonUp ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseButtonRelease, MouseEvent::MiddleButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
  mouse_signal_ (event);
  Superclass::OnMiddleButtonUp ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnRightButtonDown ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  if (Interactor->GetRepeatCount () == 0)
  {
    MouseEvent event (MouseEvent::MouseButtonPress, MouseEvent::RightButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
    mouse_signal_ (event);
  }
  else
  {
    MouseEvent event (MouseEvent::MouseDblClick, MouseEvent::RightButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
    mouse_signal_ (event);
  }
  Superclass::OnRightButtonDown ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnRightButtonUp ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseButtonRelease, MouseEvent::RightButton, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
  mouse_signal_ (event);
  Superclass::OnRightButtonUp ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnMouseWheelForward ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseScrollUp, MouseEvent::VScroll, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
  mouse_signal_ (event);
  if (Interactor->GetRepeatCount ())
    mouse_signal_ (event);
  
  if (Interactor->GetAltKey ())
  {
    // zoom
    vtkSmartPointer<vtkCamera> cam = CurrentRenderer->GetActiveCamera ();
    double opening_angle = cam->GetViewAngle ();
    if (opening_angle > 15.0)
      opening_angle -= 1.0;
    
    cam->SetViewAngle (opening_angle);
    cam->Modified ();
    CurrentRenderer->SetActiveCamera (cam);
    CurrentRenderer->ResetCameraClippingRange ();
    CurrentRenderer->Modified ();    
    CurrentRenderer->Render ();
    rens_->Render ();
    Interactor->Render ();
  }
  else
  Superclass::OnMouseWheelForward ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnMouseWheelBackward ()
{
  int x = this->Interactor->GetEventPosition()[0];
  int y = this->Interactor->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseScrollDown, MouseEvent::VScroll, x, y, Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey (), Superclass::CurrentMode);
  mouse_signal_ (event);
  if (Interactor->GetRepeatCount ())
    mouse_signal_ (event);
  
  if (Interactor->GetAltKey ())
  {
    // zoom
    vtkSmartPointer<vtkCamera> cam = CurrentRenderer->GetActiveCamera ();
    double opening_angle = cam->GetViewAngle ();
    if (opening_angle < 170.0)
      opening_angle += 1.0;
    
    cam->SetViewAngle (opening_angle);
    cam->Modified ();
    CurrentRenderer->SetActiveCamera (cam);
    CurrentRenderer->ResetCameraClippingRange ();
    CurrentRenderer->Modified ();
    CurrentRenderer->Render ();
    rens_->Render ();
    Interactor->Render ();
  }
  else
    Superclass::OnMouseWheelBackward ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLVisualizerInteractorStyle::OnTimer ()
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
void
pcl::visualization::PCLHistogramVisualizerInteractorStyle::Initialize ()
{
  init_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizerInteractorStyle::OnKeyDown ()
{
  if (!init_)
  {
    pcl::console::print_error ("[PCLHistogramVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
    return;
  }

  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

  //fprintf (stderr, "Key sym: %s\n", Interactor->GetKeySym ());
  // ---[ Check the rest of the key codes
  switch (Interactor->GetKeyCode ())
  {
    case 'q': case 'Q':
    {
      Interactor->ExitCallback ();
      return;
    }
    // Switch representation to wireframe
    default:
    {
      Superclass::OnKeyDown ();
    }
  }
  Interactor->Render ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::PCLHistogramVisualizerInteractorStyle::OnTimer ()
{
  if (!init_)
  {
    pcl::console::print_error ("[PCLHistogramVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
    return;
  }

  for (RenWinInteractMap::iterator am_it = wins_.begin (); am_it != wins_.end (); ++am_it)
    (*am_it).second.ren_->Render ();
}

namespace pcl
{
  namespace visualization
  {
    // Standard VTK macro for *New ()
    vtkStandardNewMacro (PCLVisualizerInteractorStyle);
    vtkStandardNewMacro (PCLHistogramVisualizerInteractorStyle);
  }
}

