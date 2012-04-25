/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 */

#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/common/time.h>

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewer::ImageViewer (const std::string& window_title)
  : interactor_ (vtkSmartPointer<vtkRenderWindowInteractor>::New ())
  , mouse_command_ (vtkSmartPointer<vtkCallbackCommand>::New ())
  , keyboard_command_ (vtkSmartPointer<vtkCallbackCommand>::New ())
  , image_viewer_ (vtkSmartPointer<vtkImageViewer>::New ())
  , data_ ()
  , data_size_ (0)
  , stopped_ ()
  , timer_id_ ()
  , blend_ (vtkSmartPointer<vtkImageBlend>::New ())
  , layer_map_ ()
{
  blend_->SetBlendModeToNormal ();
  blend_->SetNumberOfThreads (1);
  image_viewer_->SetColorLevel (127.5);
  image_viewer_->SetColorWindow (255);

  // Set the mouse/keyboard callbacks
  mouse_command_->SetClientData (this);
  mouse_command_->SetCallback (ImageViewer::MouseCallback);
  
  keyboard_command_->SetClientData (this);
  keyboard_command_->SetCallback (ImageViewer::KeyboardCallback);

  // Create our own  interactor and set the window title
  image_viewer_->SetupInteractor (interactor_);
  image_viewer_->GetRenderWindow ()->SetWindowName (window_title.c_str ());

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

  resetStoppedFlag ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewer::~ImageViewer ()
{
   interactor_->DestroyTimer (timer_id_);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addRGBImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  if (unsigned (image_viewer_->GetRenderWindow ()->GetSize ()[0]) != width || 
      unsigned (image_viewer_->GetRenderWindow ()->GetSize ()[1]) != height)
    image_viewer_->SetSize (width, height);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::showRGBImage] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, width, height, opacity, false);
  }

  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
  importer->SetNumberOfScalarComponents (3);
  importer->SetDataScalarTypeToUnsignedChar ();
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataExtentToWholeExtent ();

  void* data = const_cast<void*> (reinterpret_cast<const void*> (rgb_data));
  importer->SetImportVoidPointer (data, 1);
  importer->Update ();

  vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New ();
  transform->Identity ();
  transform->SetElement (1, 1, -1.0);
  transform->SetElement (1, 3, height);
  vtkSmartPointer<vtkTransform> imageTransform = vtkSmartPointer<vtkTransform>::New ();
  imageTransform->SetMatrix (transform);
  // Now create filter and set previously created transformation
  vtkSmartPointer<vtkImageReslice> algo = vtkSmartPointer<vtkImageReslice>::New ();
  algo->SetInput (importer->GetOutput ());
  algo->SetInformationInput (importer->GetOutput ());
  algo->SetResliceTransform (imageTransform);
  algo->SetInterpolationModeToCubic ();
  algo->Update ();

  // If we already have other layers, then it makes sense to use a blender
//  if (layer_map_.size () != 1)
//  {
#if ((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 6))
    image_viewer_->SetInput (algo->GetOutput ());
#else
    am_it->canvas->SetNumberOfScalarComponents (3);
    am_it->canvas->DrawImage (algo->GetOutput ());

    blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), am_it->canvas->GetOutputPort ());
    image_viewer_->SetInputConnection (blend_->GetOutputPort ());
#endif
//  }
//  // If not, pass the data directly to the viewer
//  else
//    image_viewer_->SetInput (algo->GetOutput ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showRGBImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  addRGBImage (rgb_data, width, height, layer_id, opacity);
  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addMonoImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  if (unsigned (image_viewer_->GetRenderWindow ()->GetSize ()[0]) != width || 
      unsigned (image_viewer_->GetRenderWindow ()->GetSize ()[1]) != height)
    image_viewer_->SetSize (width, height);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::showMonoImage] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, width, height, opacity, false);
  }

  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New ();
  importer->SetNumberOfScalarComponents (1);
  importer->SetWholeExtent (0, width - 1, 0, height - 1, 0, 0);
  importer->SetDataScalarTypeToUnsignedChar ();
  importer->SetDataExtentToWholeExtent ();

  void* data = const_cast<void*> (reinterpret_cast<const void*> (rgb_data));
  importer->SetImportVoidPointer (data, 1);
  importer->Update ();

  vtkSmartPointer<vtkMatrix4x4> transform = vtkSmartPointer<vtkMatrix4x4>::New ();
  transform->Identity ();
  transform->SetElement (1, 1, -1.0);
  transform->SetElement (1, 3, height);
  vtkSmartPointer<vtkTransform> imageTransform = vtkSmartPointer<vtkTransform>::New ();
  imageTransform->SetMatrix (transform);
  // Now create filter and set previously created transformation
  vtkSmartPointer<vtkImageReslice> algo = vtkSmartPointer<vtkImageReslice>::New ();
  algo->SetInput (importer->GetOutput ());
  algo->SetInformationInput (importer->GetOutput ());
  algo->SetResliceTransform (imageTransform);
  algo->SetInterpolationModeToCubic ();
  algo->Update ();

  // If we already have other layers, then it makes sense to use a blender
//  if (layer_map_.size () != 1)
//  {
#if ((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 6))
    image_viewer_->SetInput (algo->GetOutput ());
#else
    am_it->canvas->SetNumberOfScalarComponents (1);
    am_it->canvas->DrawImage (algo->GetOutput ());

    blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), am_it->canvas->GetOutputPort ());
    image_viewer_->SetInputConnection (blend_->GetOutputPort ());
#endif
//  }
//  // If not, pass the data directly to the viewer
//  else
//    image_viewer_->SetInput (algo->GetOutput ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showMonoImage (
    const unsigned char* rgb_data, unsigned width, unsigned height,
    const std::string &layer_id, double opacity)
{
  addMonoImage (rgb_data, width, height, layer_id, opacity);
  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::addFloatImage (
    const float* float_image, unsigned int width, unsigned int height,
    float min_value, float max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (float_image, width, height,
                                                              min_value, max_value, grayscale);
  showRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;  
 }

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showFloatImage (
    const float* float_image, unsigned int width, unsigned int height,
    float min_value, float max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  addFloatImage (float_image, width, height, min_value, max_value, grayscale, layer_id, opacity);
  image_viewer_->Render ();
 }
 
/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::addAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualAngleImage (angle_image, width, height);
  showRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  addAngleImage (angle_image, width, height, layer_id, opacity);
  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::addHalfAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualHalfAngleImage (angle_image, width, height);
  showRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showHalfAngleImage (
    const float* angle_image, unsigned int width, unsigned int height,
    const std::string &layer_id, double opacity)
{
  addHalfAngleImage (angle_image, width, height, layer_id, opacity);
  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::addShortImage (
    const unsigned short* short_image, unsigned int width, unsigned int height, 
    unsigned short min_value, unsigned short max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (short_image, width, height,
                                                              min_value, max_value, grayscale);
  showRGBImage (rgb_image, width, height, layer_id, opacity);
  delete[] rgb_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showShortImage (
    const unsigned short* short_image, unsigned int width, unsigned int height, 
    unsigned short min_value, unsigned short max_value, bool grayscale,
    const std::string &layer_id, double opacity)
{
  addShortImage (short_image, width, height, min_value, max_value, grayscale, layer_id, opacity);
  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::spin ()
{
  image_viewer_->Render ();
  resetStoppedFlag ();
  // Render the window before we start the interactor
  interactor_->Render ();
  interactor_->Start ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::spinOnce (int time, bool force_redraw)
{
  if (force_redraw)
  {
    image_viewer_->Render ();
    interactor_->Render ();
  }

  if (time <= 0)
    time = 1;
  
  DO_EVERY (1.0 / interactor_->GetDesiredUpdateRate (),
    exit_main_loop_timer_callback_->right_timer_id = interactor_->CreateRepeatingTimer (time);
    interactor_->Start ();
    interactor_->DestroyTimer (exit_main_loop_timer_callback_->right_timer_id);
  );
}

/////////////////////////////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::emitMouseEvent (unsigned long event_id)
{
  int x,y;
  interactor_->GetMousePosition (&x, &y);
  MouseEvent event (MouseEvent::MouseMove, MouseEvent::NoButton, x, y, 
                    interactor_->GetAltKey (), interactor_->GetControlKey (), interactor_->GetShiftKey ());
  bool repeat = false;
  switch (event_id)
  {
    case vtkCommand::MouseMoveEvent :
      event.setType(MouseEvent::MouseMove);
      break;
    
    case vtkCommand::LeftButtonPressEvent :
      event.setButton(MouseEvent::LeftButton);
      if (interactor_->GetRepeatCount () == 0)
        event.setType(MouseEvent::MouseButtonPress);
      else
        event.setType(MouseEvent::MouseDblClick);
      break;
      
    case vtkCommand::LeftButtonReleaseEvent :
      event.setButton(MouseEvent::LeftButton);
      event.setType(MouseEvent::MouseButtonRelease);
      break;
      
    case vtkCommand::RightButtonPressEvent :
      event.setButton(MouseEvent::RightButton);
      if (interactor_->GetRepeatCount () == 0)
        event.setType(MouseEvent::MouseButtonPress);
      else
        event.setType(MouseEvent::MouseDblClick);
      break;
      
    case vtkCommand::RightButtonReleaseEvent :
      event.setButton(MouseEvent::RightButton);
      event.setType(MouseEvent::MouseButtonRelease);
      break;
      
    case vtkCommand::MiddleButtonPressEvent :
      event.setButton(MouseEvent::MiddleButton);
      if (interactor_->GetRepeatCount () == 0)
        event.setType(MouseEvent::MouseButtonPress);
      else
        event.setType(MouseEvent::MouseDblClick);
      break;
      
    case vtkCommand::MiddleButtonReleaseEvent :
      event.setButton(MouseEvent::MiddleButton);
      event.setType(MouseEvent::MouseButtonRelease);
      break;
      
      case vtkCommand::MouseWheelBackwardEvent :
      event.setButton(MouseEvent::VScroll);
      event.setType(MouseEvent::MouseScrollDown);
      if (interactor_->GetRepeatCount () != 0)
        repeat = true;
      break;
      
    case vtkCommand::MouseWheelForwardEvent :
      event.setButton(MouseEvent::VScroll);
      event.setType(MouseEvent::MouseScrollUp);
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

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::emitKeyboardEvent (unsigned long event_id)
{
  KeyboardEvent event (bool(event_id == vtkCommand::KeyPressEvent), interactor_->GetKeySym (),  interactor_->GetKeyCode (), interactor_->GetAltKey (), interactor_->GetControlKey (), interactor_->GetShiftKey ());
  keyboard_signal_ (event);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::MouseCallback (vtkObject*, unsigned long eid, void* clientdata, void*)
{
  ImageViewer* window = reinterpret_cast<ImageViewer*> (clientdata);
  window->emitMouseEvent (eid);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::KeyboardCallback (vtkObject*, unsigned long eid, void* clientdata, void*)
{
  ImageViewer* window = reinterpret_cast<ImageViewer*> (clientdata);
  window->emitKeyboardEvent (eid);
}

////////////////////////////////////////////////////////////////////////////////////////////
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
    l.canvas->SetDrawColor (0.0, 0.0, 0.0, 0.0);
    l.canvas->FillBox (0, width, 0, height);
    l.canvas->Update ();
    l.canvas->Modified ();
  }
  blend_->AddInputConnection (l.canvas->GetOutputPort ());
  blend_->SetOpacity (blend_->GetNumberOfInputs () - 1, opacity);

  // Add another element
  layer_map_.push_back (l);

  return (layer_map_.end () - 1);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addLayer (
    const std::string &layer_id, int width, int height, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it != layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addLayer] Layer with ID'=%s' already exists!\n", layer_id.c_str ());
    return (false);
  }

  Layer l;
  l.layer_name = layer_id;
  l.opacity = opacity;
  // Create a new layer
  l.canvas = vtkSmartPointer<PCLImageCanvasSource2D>::New ();
  l.canvas->SetScalarTypeToUnsignedChar ();
  l.canvas->SetExtent (0, width, 0, height, 0, 0);
  l.canvas->SetNumberOfScalarComponents (4);
  blend_->AddInputConnection (l.canvas->GetOutputPort ());
  blend_->SetOpacity (blend_->GetNumberOfInputs () - 1, opacity);

  // Add another element
  layer_map_.push_back (l);

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::removeLayer (const std::string &layer_id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::removeLayer] No layer with ID'=%s' found.\n", layer_id.c_str ());
    return;
  }

  // Remove the layers that we don't want anymore
  layer_map_.erase (layer_map_.begin () + int (am_it - layer_map_.begin ()));

  // Clear the blender
  blend_->RemoveAllInputs ();
  // Add the remaining layers back to the blender
  for (size_t i = 0; i < layer_map_.size (); ++i)
  {
    blend_->AddInputConnection (layer_map_[i].canvas->GetOutputPort ());
    blend_->SetOpacity (blend_->GetNumberOfInputs () - 1, layer_map_[i].opacity);
  }
  image_viewer_->SetInputConnection (blend_->GetOutputPort ());
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addCircle (
    unsigned int x, unsigned int y, double radius, double r, double g, double b, 
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addCircle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
  am_it->canvas->DrawCircle (x, y, radius);
//  blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), am_it->canvas->GetOutputPort ());
//  image_viewer_->SetInputConnection (blend_->GetOutputPort ());

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addCircle (unsigned int x, unsigned int y, double radius, 
                                            const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addCircle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->DrawCircle (x, y, radius);
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addFilledRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    double r, double g, double b, const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
  am_it->canvas->FillBox (x_min, x_max, y_min, y_max);
//  blend_->ReplaceNthInputConnection (int (am_it - layer_map_.begin ()), am_it->canvas->GetOutputPort ());
//  image_viewer_->SetInputConnection (blend_->GetOutputPort ());

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addFilledRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->FillBox (x_min, x_max, y_min, y_max);
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    double r, double g, double b, const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
  am_it->canvas->DrawSegment (x_min, y_min, x_min, y_max);
  am_it->canvas->DrawSegment (x_min, y_max, x_max, y_max);
  am_it->canvas->DrawSegment (x_max, y_max, x_max, y_min);
  am_it->canvas->DrawSegment (x_max, y_min, x_min, y_min);

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max, 
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->DrawSegment (x_min, y_min, x_min, y_max);
  am_it->canvas->DrawSegment (x_min, y_max, x_max, y_max);
  am_it->canvas->DrawSegment (x_max, y_max, x_max, y_min);
  am_it->canvas->DrawSegment (x_max, y_min, x_min, y_min);

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    const pcl::PointXY &min_pt, const pcl::PointXY &max_pt,
    double r, double g, double b, const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
  am_it->canvas->DrawSegment (int (min_pt.x), int (min_pt.y), int (min_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (min_pt.x), int (max_pt.y), int (max_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (max_pt.y), int (max_pt.x), int (min_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (min_pt.y), int (min_pt.x), int (min_pt.y));

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addRectangle (
    const pcl::PointXY &min_pt, const pcl::PointXY &max_pt,
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addFilledRectangle] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (0.0, 255.0, 0.0, 255.0);
  am_it->canvas->DrawSegment (int (min_pt.x), int (min_pt.y), int (min_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (min_pt.x), int (max_pt.y), int (max_pt.x), int (max_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (max_pt.y), int (max_pt.x), int (min_pt.y));
  am_it->canvas->DrawSegment (int (max_pt.x), int (min_pt.y), int (min_pt.x), int (min_pt.y));

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::ImageViewer::addLine (unsigned int x_min, unsigned int y_min, 
                                          unsigned int x_max, unsigned int y_max,
                                          const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addLine] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->DrawSegment (x_min, y_min, x_max, y_max);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
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
    PCL_DEBUG ("[pcl::visualization::ImageViewer::addLine] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (r * 255.0, g * 255.0, b * 255.0, opacity * 255.0);
  am_it->canvas->DrawSegment (x_min, y_min, x_max, y_max);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::markPoint (
    size_t u, size_t v, Vector3ub fg_color, Vector3ub bg_color, double radius,
    const std::string &layer_id, double opacity)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  LayerMap::iterator am_it = std::find_if (layer_map_.begin (), layer_map_.end (), LayerComparator (layer_id));
  if (am_it == layer_map_.end ())
  {
    PCL_DEBUG ("[pcl::visualization::ImageViewer::markPoint] No layer with ID'=%s' found. Creating new one...\n", layer_id.c_str ());
    am_it = createLayer (layer_id, image_viewer_->GetRenderWindow ()->GetSize ()[0] - 1, image_viewer_->GetRenderWindow ()->GetSize ()[1] - 1, opacity, true);
  }

  am_it->canvas->SetDrawColor (fg_color[0], fg_color[1], fg_color[2], opacity * 255.0);
  am_it->canvas->DrawPoint (int (u), int (v));
  am_it->canvas->SetDrawColor (bg_color[0], bg_color[1], bg_color[2], opacity * 255.0);
  am_it->canvas->DrawCircle (int (u), int (v), radius);
}

