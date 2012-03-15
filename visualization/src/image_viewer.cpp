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
 * Author: Suat Gedikli (gedikli@willowgarage.com)
 */

#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <vtkImageImport.h>
#include <vtkImageViewer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageReslice.h>
#include <vtkTransform.h>
#include <vtkImageChangeInformation.h>
#include <vtkCallbackCommand.h>
#include <vtkObject.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageBlend.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/common/time.h>

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::visualization::ImageViewer::ImageViewer (const std::string& window_title)
  : interactor_ (vtkSmartPointer<vtkRenderWindowInteractor>::New ()),
    mouse_command_ (vtkCallbackCommand::New ()), 
    keyboard_command_ (vtkCallbackCommand::New ()),
    image_viewer_ (vtkImageViewer::New ()),
    data_size_ (0)
{
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
//  interactor_->DestroyTimer (interactor_->timer_id_);
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showRGBImage (const unsigned char* rgb_data, unsigned width, unsigned height)
{
  vtkImageImport* importer = vtkImageImport::New ();
  importer->SetNumberOfScalarComponents (3);
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

  image_viewer_->SetInput (algo->GetOutput ());
  image_viewer_->SetColorLevel (127.5);
  image_viewer_->SetColorWindow (255);
  image_viewer_->SetSize (width, height);

  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showMonoImage (const unsigned char* rgb_data, unsigned width, unsigned height)
{
  vtkImageImport* importer = vtkImageImport::New ();
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

  image_viewer_->SetInput (algo->GetOutput ());
  image_viewer_->SetColorLevel (127.5);
  image_viewer_->SetColorWindow (255);
  image_viewer_->SetSize (width, height);

  image_viewer_->Render ();
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showRGBImage (const pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  if (data_size_ < cloud.width * cloud.height)
  {
    data_size_ = cloud.width * cloud.height * 3;
    data_.reset (new unsigned char[data_size_]);
  }

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    memcpy (&data_[i * 3], reinterpret_cast<const unsigned char*> (&cloud.points[i].rgb), sizeof (unsigned char) * 3);
    /// Convert from BGR to RGB
    unsigned char aux = data_[i*3];
    data_[i*3] = data_[i*3+2];
    data_[i*3+2] = aux;
  }
  return (showRGBImage (data_.get (), cloud.width, cloud.height));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showRGBImage (const pcl::PointCloud<pcl::PointXYZRGBA> &cloud)
{
  if (data_size_ < cloud.width * cloud.height)
  {
    data_size_ = cloud.width * cloud.height * 3;
    data_.reset (new unsigned char[data_size_]);
  }

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    memcpy (&data_[i * 3], reinterpret_cast<const unsigned char*> (&cloud.points[i].rgba), sizeof (unsigned char) * 3);
    /// Convert from BGR to RGB
    unsigned char aux = data_[i*3];
    data_[i*3] = data_[i*3+2];
    data_[i*3+2] = aux;
  }
  return (showRGBImage (data_.get (), cloud.width, cloud.height));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::showFloatImage (const float* float_image, unsigned int width, unsigned int height,
                                                float min_value, float max_value, bool grayscale)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (float_image, width, height,
                                                              min_value, max_value, grayscale);
  showRGBImage (rgb_image, width, height);
  delete[] rgb_image;  
 }
 
/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showAngleImage (const float* angle_image, unsigned int width, unsigned int height)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualAngleImage (angle_image, width, height);
  showRGBImage (rgb_image, width, height);
  delete[] rgb_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showHalfAngleImage (const float* angle_image, unsigned int width, unsigned int height)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualHalfAngleImage (angle_image, width, height);
  showRGBImage (rgb_image, width, height);
  delete[] rgb_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::showShortImage (const unsigned short* short_image, unsigned int width, unsigned int height, 
                                                unsigned short min_value, unsigned short max_value, bool grayscale)
{
  unsigned char* rgb_image = FloatImageUtils::getVisualImage (short_image, width, height,
                                                              min_value, max_value, grayscale);
  showRGBImage (rgb_image, width, height);
  delete[] rgb_image;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::markPoint (size_t u, size_t v, Vector3ub fg_color, Vector3ub bg_color, float radius)
{
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = vtkSmartPointer<vtkImageCanvasSource2D>::New ();
  drawing->SetNumberOfScalarComponents (3);
  drawing->SetScalarTypeToUnsignedChar ();
  vtkImageData* image_data = image_viewer_->GetInput ();
  drawing->SetExtent (image_data->GetExtent ());
  drawing->SetDrawColor (fg_color[0], fg_color[1], fg_color[2]);
  drawing->DrawPoint (static_cast<int> (u), static_cast<int> (v));
  drawing->SetDrawColor (bg_color[0], bg_color[1], bg_color[2]);
  drawing->DrawCircle (static_cast<int> (u), static_cast<int> (v), radius);
  vtkSmartPointer<vtkImageBlend> blend = vtkSmartPointer<vtkImageBlend>::New();
  blend->AddInput (image_data);
  blend->AddInput (drawing->GetOutput ());
  blend->SetOpacity (0, 0.6);
  blend->SetOpacity (1, 0.4);
  image_viewer_->SetInput (blend->GetOutput ());
}

/////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::visualization::ImageViewer::spin ()
{
  resetStoppedFlag ();
  // Render the window before we start the interactor
  interactor_->Render ();
  interactor_->Start ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::ImageViewer::spinOnce (int time, bool)
{
  if (time <= 0)
    time = 1;
  
  DO_EVERY (1.0 / interactor_->GetDesiredUpdateRate (),
    interactor_->Render ();
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

