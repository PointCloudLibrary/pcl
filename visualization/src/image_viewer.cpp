/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id$
 */

#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/common/time.h>
#include <pcl/visualization/vtk/vtkRenderWindowInteractorFix.h>

//////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewer::ImageViewer (const std::string& window_title)
  : interactor_ ()
  , mouse_command_ (vtkSmartPointer<vtkCallbackCommand>::New ())
  , keyboard_command_ (vtkSmartPointer<vtkCallbackCommand>::New ())
  , exit_main_loop_timer_callback_ ()
  , exit_callback_ ()
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION < 10))
  , image_viewer_ (vtkSmartPointer<vtkImageViewer>::New ())
#else
  , win_ (vtkSmartPointer<vtkRenderWindow>::New ())
  , ren_ (vtkSmartPointer<vtkRenderer>::New ())
  , slice_ (vtkSmartPointer<vtkImageSlice>::New ())
#endif
  , interactor_style_ (vtkSmartPointer<ImageViewerInteractorStyle>::New ())
  , data_ ()
  , data_size_ (0)
  , stopped_ ()
  , timer_id_ ()
  , blend_ (vtkSmartPointer<vtkImageBlend>::New ())
  , layer_map_ ()
  , algo_ (vtkSmartPointer<vtkImageFlip>::New ())
{
  interactor_ = vtkSmartPointer <vtkRenderWindowInteractor>::Take (vtkRenderWindowInteractorFixNew ());
  // Prepare for image flip
  algo_->SetInterpolationModeToCubic ();
  algo_->PreserveImageExtentOn ();
  algo_->FlipAboutOriginOn ();
  algo_->SetFilteredAxis (1);

  blend_->SetBlendModeToNormal ();
  blend_->SetNumberOfThreads (1);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION < 10))
  image_viewer_->SetColorLevel (127.5);
  image_viewer_->SetColorWindow (255);
#endif

  // Set the mouse/keyboard callbacks
  mouse_command_->SetClientData (this);
  mouse_command_->SetCallback (ImageViewer::MouseCallback);
  
  keyboard_command_->SetClientData (this);
  keyboard_command_->SetCallback (ImageViewer::KeyboardCallback);
 
  // Create our own  interactor and set the window title
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION < 10))
  image_viewer_->SetupInteractor (interactor_);
  image_viewer_->GetRenderWindow ()->SetWindowName (window_title.c_str ());
  image_viewer_->GetRenderWindow ()->DoubleBufferOn ();
  image_viewer_->GetRenderWindow ()->EraseOff ();
  image_viewer_->GetRenderWindow ()->SetSize (640, 480);
  ren_ = image_viewer_->GetRenderer ();
  win_ = image_viewer_->GetRenderWindow ();
  interactor_ = win_->GetInteractor ();
#else
  win_->SetSize (640, 480);
  win_->AddRenderer (ren_);
  win_->SetWindowName (window_title.c_str ());
  interactor_->SetRenderWindow (win_);

  vtkSmartPointer<vtkImageData> empty_image = vtkSmartPointer<vtkImageData>::New ();
  vtkSmartPointer<vtkImageSliceMapper> map = vtkSmartPointer<vtkImageSliceMapper>::New ();
  map->SetInput (empty_image);
  slice_->SetMapper (map);
  ren_->AddViewProp (slice_);
  interactor_->SetInteractorStyle (interactor_style_);
#endif

  // Initialize and create timer
  interactor_->Initialize ();
  timer_id_ = interactor_->CreateRepeatingTimer (0);
  
  // Set the exit callbacks
  exit_main_loop_timer_callback_ = vtkSmartPointer<ExitMainLoopTimerCallback>::New ();
  exit_main_loop_timer_callback_->window = this;
  exit_main_loop_timer_callback_->right_timer_id = -1;
  interactor_->AddObserver (vtkCommand::TimerEvent, exit_main_loop_timer_callback_);

  exit_callback_ = vtkSmartPointer<ExitCallback>::New ();
  exit_callback_->window = this;
  interactor_->AddObserver (vtkCommand::ExitEvent, exit_callback_);

  // Reset camera (flip it vertically)
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  //ren_->GetActiveCamera ()->SetViewUp (0.0, -1.0, 0.0);
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New ();
  transform->Scale (1.0, -1.0, 1.0);
  ren_->GetActiveCamera ()->SetUserTransform (transform);
  ren_->GetActiveCamera ()->ParallelProjectionOn ();
  ren_->ResetCamera ();
  ren_->ResetCameraClippingRange ();
#endif
  resetStoppedFlag ();
    
  PCL_DEBUG ("[pcl::visualization::ImageViewer] VTK version found: %d.%d\n", VTK_MAJOR_VERSION, VTK_MINOR_VERSION);
}

//////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewer::~ImageViewer ()
{
   interactor_->DestroyTimer (timer_id_);
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addRGBImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  if (unsigned (getSize ()[0]) != width || 
      unsigned (getSize ()[1]) != height)
    setSize (width, height);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addRGBImage] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, width, height, opacity, false);
  }

  void* data = const_cast<void*> (reinterpret_cast<const void*> (rgb_data));
  
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New ();
  image->SetExtent (0, width - 1, 0, height - 1, 0, 0);
  image->SetScalarTypeToUnsignedChar ();
  image->SetNumberOfScalarComponents (3);
  image->AllocateScalars ();
  image->GetPointData ()->GetScalars ()->SetVoidArray (data, 3 * width * height, 1);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION < 10))
  // Now create filter and set previously created transformation
  algo_->SetInput (image);
  algo_->Update ();
#  if (VTK_MINOR_VERSION <= 6)
    image_viewer_->SetInput (algo_->GetOutput ());
#  else
    blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), algo_->GetOutputPort ());
    image_viewer_->SetInputConnection (blend_->GetOutputPort ());
#  endif
#else
  blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), image->GetProducerPort ());
  slice_->GetMapper ()->SetInput (blend_->GetOutput ());

  interactor_style_->adjustCamera (image, ren_);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showRGBImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  addRGBImage (rgb_data, width, height, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addMonoImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  if (unsigned (getSize ()[0]) != width || 
      unsigned (getSize ()[1]) != height)
    setSize (width, height);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::showMonoImage] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, width, height, opacity, false);
  }

  void* data = const_cast<void*> (reinterpret_cast<const void*> (rgb_data));
  
  vtkSmartPointer<vtkImageData> image = vtkSmartPointer<vtkImageData>::New ();
  image->SetExtent (0, width - 1, 0, height - 1, 0, 0);
  image->SetScalarTypeToUnsignedChar ();
  image->SetNumberOfScalarComponents (1);
  image->AllocateScalars ();
  image->GetPointData ()->GetScalars ()->SetVoidArray (data, width * height, 1);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION < 10))
  // Now create filter and set previously created transformation
  algo_->SetInput (image);
  algo_->Update ();
#  if ((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 6))
    image_viewer_->SetInput (algo_->GetOutput ());
#  else
    blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), algo_->GetOutputPort ());
    image_viewer_->SetInputConnection (blend_->GetOutputPort ());
#  endif
#else
  blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), image->GetProducerPort ());
  slice_->GetMapper ()->SetInput (blend_->GetOutput ());
  
  interactor_style_->adjustCamera (image, ren_);
#endif
}


//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showMonoImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  addMonoImage (rgb_data, width, height, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addMonoImage (
    const pcl::PointCloud<pcl::Intensity> &cloud,
    const std::string &layer_id, double opacity)
{
  if (data_size_ < cloud.width * cloud.height)
  {
    data_size_ = cloud.width * cloud.height * 3;
    data_.reset (new unsigned char[data_size_]);
  }

  convertIntensityCloudToUChar (cloud, data_);

  return (addMonoImage (data_.get (), cloud.width, cloud.height, layer_id, opacity));
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showMonoImage (
    const pcl::PointCloud<pcl::Intensity> &cloud,
    const std::string &layer_id, double opacity)
{
  addMonoImage (cloud, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addMonoImage (
    const pcl::PointCloud<pcl::Intensity8u> &cloud,
    const std::string &layer_id, double opacity)
{
  if (data_size_ < cloud.width * cloud.height)
  {
    data_size_ = cloud.width * cloud.height * 3;
    data_.reset (new unsigned char[data_size_]);
  }

  convertIntensityCloud8uToUChar (cloud, data_);

  return (addMonoImage (data_.get (), cloud.width, cloud.height, layer_id, opacity));
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showMonoImage (
    const pcl::PointCloud<pcl::Intensity8u> &cloud,
    const std::string &layer_id, double opacity)
{
  addMonoImage (cloud, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addFloatImage (
    const float* float_image, unsigned int width, unsigned int height,
    float min_value, float max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (float_image, width, height,
                                                              min_value, max_value, grayscale);
  addRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;  
 }

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showFloatImage (
    const float* float_image, unsigned int width, unsigned int height,
    float min_value, float max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  addFloatImage (float_image, width, height, min_value, max_value, grayscale, layer_id, opacity);
  render ();
 }
 
//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::addAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualAngleImage (angle_image, width, height);
  showRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  addAngleImage (angle_image, width, height, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::addHalfAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualHalfAngleImage (angle_image, width, height);
  addRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showHalfAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  addHalfAngleImage (angle_image, width, height, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::addShortImage (
    const unsigned short* short_image, unsigned int width, unsigned int height, 
    unsigned short min_value, unsigned short max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (short_image, width, height,
                                                              min_value, max_value, grayscale);
  addRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showShortImage (
    const unsigned short* short_image, unsigned int width, unsigned int height, 
    unsigned short min_value, unsigned short max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  addShortImage (short_image, width, height, min_value, max_value, grayscale, layer_id, opacity);
  render ();
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::spin ()
{
  if (stopped_)
    return (false);

  render ();
  resetStoppedFlag ();
  // Render the window before we start the interactor
  //interactor_->Render ();
  interactor_->Start ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::spinOnce (int time, bool force_redraw)
{
  if (stopped_)
    return (false);

  resetStoppedFlag ();
  if (force_redraw)
  {
    render ();
    //interactor_->Render ();
  }

  if (time <= 0)
    time = 1;
  
  DO_EVERY (1.0 / interactor_->GetDesiredUpdateRate (),
    exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer (time);
    interactor_->Start ();
    interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
  );

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection 
pcl::visualization::ImageViewer::registerMouseCallback (
    boost::function<void (const pcl::visualization::MouseEvent&)> callback)
{
  // just add observer at first time when a callback is registered
  if (mouse_signal_.empty ())
  {
    interactor_->AddObserver (vtkCommand::MouseMoveEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::MiddleButtonPressEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::MiddleButtonReleaseEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::MouseWheelBackwardEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::MouseWheelForwardEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::LeftButtonPressEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::LeftButtonReleaseEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::RightButtonPressEvent, mouse_command_);
    interactor_->AddObserver (vtkCommand::RightButtonReleaseEvent, mouse_command_);
  }
  return (mouse_signal_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////
boost::signals2::connection 
pcl::visualization::ImageViewer::registerKeyboardCallback (
    boost::function<void (const pcl::visualization::KeyboardEvent&)> callback)
{
  // just add observer at first time when a callback is registered
  if (keyboard_signal_.empty ())
  {
    interactor_->AddObserver (vtkCommand::KeyPressEvent, keyboard_command_);
    interactor_->AddObserver (vtkCommand::KeyReleaseEvent, keyboard_command_);
  }
  
  return (keyboard_signal_.connect (callback));
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::emitMouseEvent (unsigned long event_id)
{
  //interactor_->GetMousePosition (&x, &y);
  int x = this->interactor_->GetEventPosition()[0];
  int y = this->interactor_->GetEventPosition()[1];
  MouseEvent event (MouseEvent::MouseMove, MouseEvent::NoButton, x, y, 
                    interactor_->GetAltKey (), 
                    interactor_->GetControlKey (), 
                    interactor_->GetShiftKey ());
  bool repeat = false;
  switch (event_id)
  {
    case vtkCommand::MouseMoveEvent :
      event.setType (MouseEvent::MouseMove);
      break;
    
    case vtkCommand::LeftButtonPressEvent :
      event.setButton (MouseEvent::LeftButton);
      if (interactor_->GetRepeatCount () == 0)
        event.setType (MouseEvent::MouseButtonPress);
      else
        event.setType (MouseEvent::MouseDblClick);
      break;
      
    case vtkCommand::LeftButtonReleaseEvent :
      event.setButton (MouseEvent::LeftButton);
      event.setType (MouseEvent::MouseButtonRelease);
      break;
      
    case vtkCommand::RightButtonPressEvent :
      event.setButton (MouseEvent::RightButton);
      if (interactor_->GetRepeatCount () == 0)
        event.setType (MouseEvent::MouseButtonPress);
      else
        event.setType (MouseEvent::MouseDblClick);
      break;
      
    case vtkCommand::RightButtonReleaseEvent :
      event.setButton (MouseEvent::RightButton);
      event.setType (MouseEvent::MouseButtonRelease);
      break;
      
    case vtkCommand::MiddleButtonPressEvent :
      event.setButton (MouseEvent::MiddleButton);
      if (interactor_->GetRepeatCount () == 0)
        event.setType (MouseEvent::MouseButtonPress);
      else
        event.setType (MouseEvent::MouseDblClick);
      break;
      
    case vtkCommand::MiddleButtonReleaseEvent :
      event.setButton (MouseEvent::MiddleButton);
      event.setType (MouseEvent::MouseButtonRelease);
      break;
      
    case vtkCommand::MouseWheelBackwardEvent :
      event.setButton (MouseEvent::VScroll);
      event.setType (MouseEvent::MouseScrollDown);
      if (interactor_->GetRepeatCount () != 0)
        repeat = true;
      break;
      
    case vtkCommand::MouseWheelForwardEvent :
      event.setButton (MouseEvent::VScroll);
      event.setType (MouseEvent::MouseScrollUp);
      if (interactor_->GetRepeatCount () != 0)
        repeat = true;
      break;
    default:
      return;
  }
  
  mouse_signal_ (event);
  if (repeat)
    mouse_signal_ (event);
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::emitKeyboardEvent (unsigned long event_id)
{
  KeyboardEvent event (bool(event_id == vtkCommand::KeyPressEvent), interactor_->GetKeySym (),  interactor_->GetKeyCode (), interactor_->GetAltKey (), interactor_->GetControlKey (), interactor_->GetShiftKey ());
  keyboard_signal_ (event);
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::MouseCallback (vtkObject*, unsigned long eid, void* clientdata, void*)
{
  ImageViewer* window = reinterpret_cast<ImageViewer*> (clientdata);
  window->emitMouseEvent (eid);
}

//////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::KeyboardCallback (vtkObject*, unsigned long eid, void* clientdata, void*)
{
  ImageViewer* window = reinterpret_cast<ImageViewer*> (clientdata);
  window->emitKeyboardEvent (eid);
}

//////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewer::LayerMap::iterator
pcl::visualization::ImageViewer::createLayer (
    const std::string &layer_id, int width, int height, double opacity, bool fill_box)
{
  Layer l;
  l.layer_name = layer_id;
  l.opacity = opacity;
  // Create a new layer
  l.canvas = vtkSmartPointer<PCLImageCanvasSource2D>::New ();
  l.canvas->SetScalarTypeToUnsignedChar ();
  l.canvas->SetExtent (0, width, 0, height, 0, 0);
  l.canvas->SetNumberOfScalarComponents (4);
  if (fill_box)
  {
    l.canvas->SetDrawColor (0.0, 0.0, 0.0, 0.0); //opacity * 255.0);
    l.canvas->FillBox (0, width, 0, height);
    l.canvas->Update ();
    l.canvas->Modified ();
  }
  blend_->AddInputConnection (l.canvas->GetOutputPort ());
  blend_->SetOpacity (blend_->GetNumberOfInputs () - 1, opacity);

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  slice_->GetMapper ()->SetInput (blend_->GetOutput ());
  slice_->GetProperty ()->SetColorLevel (127.5);
  slice_->GetProperty ()->SetColorWindow (255);

  if (!ren_->HasViewProp (slice_))
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::createLayer] Global actor not found. Creating a new one (layer ID='%s').\n", layer_id.c_str ());
    ren_->AddViewProp (slice_);
  }
#else
  image_viewer_->SetInputConnection (blend_->GetOutputPort ());
#endif

  // Add another element
  layer_map_.push_back (l);

  return (layer_map_.end () - 1);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addLayer (
    const std::string &layer_id, int width, int height, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it != layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addLayer] Layer with ID='%s' already exists!\n", layer_id.c_str ());
    return (false);
  }

  createLayer (layer_id, width, height, opacity, false);

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::removeLayer (const std::string &layer_id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::removeLayer] No layer with ID='%s' found.\n", layer_id.c_str ());
    return;
  }

  // Remove the layers that we don't want anymore
  layer_map_.erase (layer_map_.begin () + int (am_it - layer_map_.begin ()));

  // Clear the blender
  blend_->RemoveAllInputs ();

  // Add one empty black layer
  unsigned width  = unsigned (getSize ()[0]);
  unsigned height = unsigned (getSize ()[1]);
  // Create a new layer
  vtkSmartPointer<PCLImageCanvasSource2D> canvas = vtkSmartPointer<PCLImageCanvasSource2D>::New ();
  canvas->SetScalarTypeToUnsignedChar ();
  canvas->SetExtent (0, width, 0, height, 0, 0);
  canvas->SetNumberOfScalarComponents (4);
  canvas->SetDrawColor (0.0, 0.0, 0.0, 255.0);
  canvas->FillBox (0, width, 0, height);
  canvas->Update ();
  canvas->Modified ();
  blend_->AddInputConnection (canvas->GetOutputPort ());
  blend_->SetOpacity (blend_->GetNumberOfInputs () - 1, 1.0);

  if (layer_map_.size () > 0)
  {
    // Add the remaining layers back to the blender
    for (size_t i = 0; i < layer_map_.size (); ++i)
    {
      blend_->AddInputConnection (layer_map_[i].canvas->GetOutputPort ());
      blend_->SetOpacity (blend_->GetNumberOfInputs () - 1, layer_map_[i].opacity);
    }
  }
  blend_->Modified ();
  blend_->Update ();

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  slice_->GetMapper ()->SetInput (blend_->GetOutput ());
  slice_->GetMapper ()->Modified ();
#else
  image_viewer_->SetInputConnection (blend_->GetOutputPort ());
  image_viewer_->Modified ();
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addCircle (
    unsigned int x, unsigned int y, double radius, double r, double g, double b, 
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addCircle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawCircle (x, y, radius);
#else
  am_it->canvas->DrawCircle (x, getSize ()[1] - y, radius);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addCircle (unsigned int x, unsigned int y, double radius, 
                                            const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addCircle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (0.0, 255.0, 0.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawCircle (x, y, radius);
#else
  am_it->canvas->DrawCircle (x, getSize ()[1] - y, radius);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addFilledRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    double r, double g, double b, const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->FillBox (x_min, x_max, y_min, y_max);
#else
  am_it->canvas->FillBox (x_min, x_max, getSize ()[1] - y_max, getSize ()[1] - y_min);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addFilledRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (0.0, 255.0, 0.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->FillBox (x_min, x_max, y_min, y_max);
#else
  am_it->canvas->FillBox (x_min, x_max, getSize ()[1] - y_max, getSize ()[1] - y_min);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    double r, double g, double b, const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addRectangle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawSegment (x_min, y_min, x_min, y_max);
  am_it->canvas->DrawSegment (x_min, y_max, x_max, y_max);
  am_it->canvas->DrawSegment (x_max, y_max, x_max, y_min);
  am_it->canvas->DrawSegment (x_max, y_min, x_min, y_min);
#else
  am_it->canvas->DrawSegment (x_min, getSize ()[1] - y_min, x_min, getSize ()[1] - y_max);
  am_it->canvas->DrawSegment (x_min, getSize ()[1] - y_max, x_max, getSize ()[1] - y_max);
  am_it->canvas->DrawSegment (x_max, getSize ()[1] - y_max, x_max, getSize ()[1] - y_min);
  am_it->canvas->DrawSegment (x_max, getSize ()[1] - y_min, x_min, getSize ()[1] - y_min);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addRectangle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (0.0, 255.0, 0.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawSegment (x_min, y_min, x_min, y_max);
  am_it->canvas->DrawSegment (x_min, y_max, x_max, y_max);
  am_it->canvas->DrawSegment (x_max, y_max, x_max, y_min);
  am_it->canvas->DrawSegment (x_max, y_min, x_min, y_min);
#else
  am_it->canvas->DrawSegment (x_min, getSize ()[1] - y_min, x_min, getSize ()[1] - y_max);
  am_it->canvas->DrawSegment (x_min, getSize ()[1] - y_max, x_max, getSize ()[1] - y_max);
  am_it->canvas->DrawSegment (x_max, getSize ()[1] - y_max, x_max, getSize ()[1] - y_min);
  am_it->canvas->DrawSegment (x_max, getSize ()[1] - y_min, x_min, getSize ()[1] - y_min);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    const pcl::PointXY &min_pt, const pcl::PointXY &max_pt,
    double r, double g, double b, const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addRectangle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawSegment (int (min_pt.x), int (min_pt.y), int (min_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (min_pt.x), int (max_pt.y), int (max_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (max_pt.y), int (max_pt.x), int (min_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (min_pt.y), int (min_pt.x), int (min_pt.y));
#else
  am_it->canvas->DrawSegment (int (min_pt.x), getSize ()[1] - int (max_pt.y), int (min_pt.x), getSize ()[1] - int (min_pt.y));
  am_it->canvas->DrawSegment (int (min_pt.x), getSize ()[1] - int (min_pt.y), int (max_pt.x), getSize ()[1] - int (min_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), getSize ()[1] - int (min_pt.y), int (max_pt.x), getSize ()[1] - int (max_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), getSize ()[1] - int (max_pt.y), int (min_pt.x), getSize ()[1] - int (max_pt.y));
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    const pcl::PointXY &min_pt, const pcl::PointXY &max_pt,
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addRectangle] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (0.0, 255.0, 0.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawSegment (int (min_pt.x), int (min_pt.y), int (min_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (min_pt.x), int (max_pt.y), int (max_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (max_pt.y), int (max_pt.x), int (min_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (min_pt.y), int (min_pt.x), int (min_pt.y));
#else
  am_it->canvas->DrawSegment (int (min_pt.x), getSize ()[1] - int (max_pt.y), int (min_pt.x), getSize ()[1] - int (min_pt.y));
  am_it->canvas->DrawSegment (int (min_pt.x), getSize ()[1] - int (min_pt.y), int (max_pt.x), getSize ()[1] - int (min_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), getSize ()[1] - int (min_pt.y), int (max_pt.x), getSize ()[1] - int (max_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), getSize ()[1] - int (max_pt.y), int (min_pt.x), getSize ()[1] - int (max_pt.y));
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addLine (unsigned int x_min, unsigned int y_min, 
                                          unsigned int x_max, unsigned int y_max,
                                          const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addLine] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (0.0, 255.0, 0.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawSegment (x_min, y_min, x_max, y_max);
#else
  am_it->canvas->DrawSegment (x_min, getSize ()[1] - y_min, x_max, getSize ()[1] - y_max);
#endif

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addLine (unsigned int x_min, unsigned int y_min, 
                                          unsigned int x_max, unsigned int y_max,
                                          double r, double g, double b, 
                                          const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addLine] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawSegment (x_min, y_min, x_max, y_max);
#else
  am_it->canvas->DrawSegment (x_min, getSize ()[1] - y_min, x_max, getSize ()[1] - y_max);
#endif
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::markPoint (
    size_t u, size_t v, Vector3ub fg_color, Vector3ub bg_color, double radius,
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::markPoint] No layer with ID='%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, getSize ()[0] - 1, getSize ()[1] - 1, opacity, true);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    interactor_style_->adjustCamera (ren_);
#endif
  }

  am_it->canvas->SetDrawColor (fg_color[0], fg_color[1], fg_color[2], opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawPoint (int (u), int (v));
#else
  am_it->canvas->DrawPoint (int (u), getSize ()[1] - int (v));
#endif
  am_it->canvas->SetDrawColor (bg_color[0], bg_color[1], bg_color[2], opacity * 255.0);
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  am_it->canvas->DrawCircle (int (u), int (v), radius);
#else
  am_it->canvas->DrawCircle (int (u), getSize ()[1] - int (v), radius);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::render ()
{
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION < 10))
  image_viewer_->Render ();
#else
  win_->Render ();
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::convertIntensityCloudToUChar (
    const pcl::PointCloud<pcl::Intensity> &cloud,
    boost::shared_array<unsigned char> data)
{
  int j = 0;
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    data[j++] = static_cast <unsigned char> (cloud.points[i].intensity * 255);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::convertIntensityCloud8uToUChar (
    const pcl::PointCloud<pcl::Intensity8u> &cloud,
    boost::shared_array<unsigned char> data)
{
  int j = 0;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    data[j++] = static_cast<unsigned char> (cloud.points[i].intensity);
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewerInteractorStyle::ImageViewerInteractorStyle ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewerInteractorStyle::OnChar ()
{
  FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
  vtkPropCollection *props = CurrentRenderer->GetViewProps ();
  vtkProp *prop = 0;
  vtkAssemblyPath *path;
  vtkImageSlice *image_prop = 0;
  vtkCollectionSimpleIterator pit;

  for (props->InitTraversal (pit); (prop = props->GetNextProp (pit)); )
  {
    for (prop->InitPathTraversal (); (path = prop->GetNextPath ()); )
    {
      vtkProp *try_prop = path->GetLastNode ()->GetViewProp ();
      if ( (image_prop = vtkImageSlice::SafeDownCast (try_prop)) != 0 )
        break;
    }
  }

  vtkImageProperty *property = image_prop->GetProperty ();
#endif

  switch (Interactor->GetKeyCode ())
  {
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION >= 10))
    case 'r':
    case 'R':
      property->SetColorLevel (127.5);
      property->SetColorWindow (255);
      this->Interactor->Render ();
      break;
#endif
    default:
      Superclass::OnChar ();
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewerInteractorStyle::adjustCamera (
    vtkImageData *image, vtkRenderer *ren)
{
  // Set up the background camera to fill the renderer with the image
  double origin[3], spacing[3];
  int extent[6];
  image->GetOrigin  (origin);
  image->GetSpacing (spacing);
  image->GetExtent  (extent);
 
  vtkCamera* camera = ren->GetActiveCamera ();
  double xc = origin[0] + 0.5 * (extent[0] + extent[1]) * spacing[0];
  double yc = origin[1] + 0.5 * (extent[2] + extent[3]) * spacing[1];
  double yd = (extent[3] - extent[2] + 1) * spacing[1];
  double d = camera->GetDistance ();
  camera->SetParallelScale (0.5 * yd);
  camera->SetFocalPoint (xc, yc, 0.0);
  camera->SetPosition (xc, yc, d);
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewerInteractorStyle::adjustCamera (vtkRenderer *ren)
{
  // Set up the background camera to fill the renderer with the image
  vtkCamera* camera = ren->GetActiveCamera ();
  int *wh = ren->GetRenderWindow ()->GetSize ();
  double xc = static_cast<double> (wh[0]) / 2.0,
         yc = static_cast<double> (wh[1]) / 2.0,
         yd = static_cast<double> (wh[1]),
         d = 3.346065;
  camera->SetParallelScale (0.5 * yd);
  camera->SetFocalPoint (xc, yc, 0.0);
  camera->SetPosition (xc, yc, d);
}

//////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewerInteractorStyle::OnLeftButtonDown ()
{
  int x = Interactor->GetEventPosition ()[0];
  int y = Interactor->GetEventPosition ()[1];

  FindPokedRenderer (x, y);
  if (CurrentRenderer == NULL)
    return;
  
  // Redefine this button to handle window/level
  GrabFocus (this->EventCallbackCommand);
  // If shift is held down, do nothing
  if (!this->Interactor->GetShiftKey() && !this->Interactor->GetControlKey()) 
  {
    WindowLevelStartPosition[0] = x;
    WindowLevelStartPosition[1] = y;
    StartWindowLevel ();
  }
  else if (Interactor->GetShiftKey ())
    return;
  // If ctrl is held down in slicing mode, do nothing
  else if (Interactor->GetControlKey ())
    return;
  // The rest of the button + key combinations remain the same
  else
    Superclass::OnLeftButtonDown ();
}

//////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  namespace visualization
  {
    vtkStandardNewMacro (ImageViewerInteractorStyle);
  }
}

