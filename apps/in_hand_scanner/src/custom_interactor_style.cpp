/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/apps/in_hand_scanner/custom_interactor_style.h>

#include <list>
#include <cmath>

#include <pcl/visualization/common/io.h>
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
#include <vtkPointPicker.h>
#include <vtkMatrix4x4.h>
#include <vtkInteractorObserver.h>
#include <vtkCamera.h>

#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>

pcl::ihs::CustomInteractorStyle::CustomInteractorStyle ()
  : pcl::visualization::PCLVisualizerInteractorStyle (),
    R_     (1., 0., 0., 0.),
    t_     (0., 0., 0.),
    pivot_ (0., 0., 0.)
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::OnChar ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::OnKeyDown ()
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

  KeyboardEvent event (true, Interactor->GetKeySym (), Interactor->GetKeyCode (), Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey ());
  keyboard_signal_ (event);

  rens_->Render ();
  Interactor->Render ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::OnKeyUp ()
{
  KeyboardEvent event (false, Interactor->GetKeySym (), Interactor->GetKeyCode (), Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey ());
  keyboard_signal_ (event);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::resetCamera ()
{
  R_     = Quaternion (1., 0., 0., 0.);
  t_     = Vec3       (0., 0., 0.);
  pivot_ = Vec3       (0., 0., 0.);

  this->updateCamera ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::transformCamera (const Quaternion& rotation, const Vec3& translation)
{
  R_  = (R_ * rotation).normalized ();
  t_ += translation;
  this->updateCamera ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::setPivot (const Vec3& pivot)
{
  pivot_ = pivot;
}

////////////////////////////////////////////////////////////////////////////////

const pcl::ihs::CustomInteractorStyle::Vec3&
pcl::ihs::CustomInteractorStyle::getPivot () const
{
  return (pivot_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::Rotate ()
{
  if (!this->Interactor)                          return;
  if (!this->CurrentRenderer)                     return;
  if (!this->CurrentRenderer->GetRenderWindow ()) return;

  vtkRenderWindowInteractor* rwi = this->Interactor;
  vtkRenderWindow*           rw  = this->CurrentRenderer->GetRenderWindow();

  const double dx = static_cast <double> (rwi->GetEventPosition ()[0] - rwi->GetLastEventPosition ()[0]);
  const double dy = static_cast <double> (rwi->GetEventPosition ()[1] - rwi->GetLastEventPosition ()[1]);
  const double w  = static_cast <double> (rw->GetSize()[0]); // window width
  const double h  = static_cast <double> (rw->GetSize()[1]); // window height

  if ((int(dx)==0 && int(dy)==0) || (int(w)==0 && int(h)==0))
  {
    return;
  }

  const Vec3   rot_axis  = (R_ * Vec3::UnitX () * dy + R_ * Vec3::UnitY () * dx).normalized ();
  const double rot_angle = 10. * std::atan (std::sqrt ((dx*dx + dy*dy) / (w*w + h*h)));

  const Quaternion dR (AngleAxis (rot_angle, rot_axis));

  t_ = dR * (t_ - pivot_) + pivot_;
  R_ = (dR * R_).normalized ();

  this->updateCamera ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::Spin ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::Pan ()
{
  if (!this->Interactor)                          return;
  if (!this->CurrentRenderer)                     return;
  if (!this->CurrentRenderer->GetRenderWindow ()) return;

  vtkRenderWindowInteractor* rwi = this->Interactor;
  vtkRenderWindow*           rw  = this->CurrentRenderer->GetRenderWindow();

  const double dx = static_cast <double> (rwi->GetEventPosition ()[0] - rwi->GetLastEventPosition ()[0]);
  const double dy = static_cast <double> (rwi->GetEventPosition ()[1] - rwi->GetLastEventPosition ()[1]);
  const double w  = static_cast <double> (rw->GetSize()[0]); // window width
  const double h  = static_cast <double> (rw->GetSize()[1]); // window height

  if ((int(dx)==0 && int(dy)==0) || (int(w)==0 && int(h)==0))
  {
    return;
  }

  const double factor = 20. * std::max ((pivot_ - R_*Vec3::Zero ()).squaredNorm (), 1.);
  t_ += R_ * (Vec3::UnitY () * dy - Vec3::UnitX () * dx) * factor / (w*w+h*h);

  this->updateCamera ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::Dolly ()
{
  if (!this->Interactor)                          return;
  if (!this->CurrentRenderer)                     return;
  if (!this->CurrentRenderer->GetRenderWindow ()) return;

  vtkRenderWindowInteractor* rwi = this->Interactor;
  vtkRenderWindow*           rw  = this->CurrentRenderer->GetRenderWindow();

  const double dx = static_cast <double> (rwi->GetEventPosition ()[0] - rwi->GetLastEventPosition ()[0]);
  const double dy = static_cast <double> (rwi->GetEventPosition ()[1] - rwi->GetLastEventPosition ()[1]);
  const double w  = static_cast <double> (rw->GetSize()[0]); // window width
  const double h  = static_cast <double> (rw->GetSize()[1]); // window height

  if ((int(dx)==0 && int(dy)==0) || (int(w)==0 && int(h)==0))
  {
    return;
  }

  const double factor = 30. * std::max ((pivot_ - R_*Vec3::Zero ()).squaredNorm (), 10.);
  t_ += R_ * Vec3::UnitZ () * dy * factor / (w*w+h*h);

  this->updateCamera ();
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::OnMouseWheelForward ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::OnMouseWheelBackward ()
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::CustomInteractorStyle::updateCamera ()
{
  if (!this->Interactor)                          return;
  if (!this->CurrentRenderer)                     return;
  if (!this->CurrentRenderer->GetActiveCamera ()) return;

  vtkCamera* cam = this->CurrentRenderer->GetActiveCamera ();
  cam->SetPosition   ((R_ *  Vec3::Zero  () + t_).eval ().data ());
  cam->SetFocalPoint ((R_ *  Vec3::UnitZ () + t_).eval ().data ());
  cam->SetViewUp     ((R_ * -Vec3::UnitY ()     ).normalized ().eval ().data ());
  cam->SetViewAngle (48.6); // TODO: Get the camera parameters from the sensor
  cam->Modified ();

  if (this->AutoAdjustCameraClippingRange)
  {
    this->CurrentRenderer->ResetCameraClippingRange();
  }

  if (this->Interactor->GetLightFollowCamera())
  {
    this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
  }

  this->CurrentRenderer->Modified ();
  this->CurrentRenderer->Render ();
  this->Interactor->Render ();
  this->rens_->Render ();
}

////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    vtkStandardNewMacro (CustomInteractorStyle);
  }
}

////////////////////////////////////////////////////////////////////////////////
