/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#pragma once

// C++
#include <iostream>
#include <string>

// PCL
#include "camera.h"

// VTK
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkObject.h>
#include <vtkTextActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>

class Viewport
{
public:

  // Operators
  // -----------------------------------------------------------------------------
  Viewport (vtkSmartPointer<vtkRenderWindow> window, double xmin = 0.0, double ymin = 0.0, double xmax = 1.0,
            double ymax = 1.0);

  // Accessors
  // -----------------------------------------------------------------------------
  inline vtkSmartPointer<vtkRenderer>
  getRenderer () const
  {
    return renderer_;
  }

  void
  setCamera (Camera* camera)
  {
    renderer_->SetActiveCamera (vtkCamera::SafeDownCast (camera->getCamera ()));
    camera_hud_actor_->SetInput (camera->getName ().c_str ());
    renderer_->ResetCamera ();
  }

private:

  // Callbacks
  // -----------------------------------------------------------------------------
  static void
  viewportModifiedCallback (vtkObject* caller, unsigned long int vtkNotUsed(eventId), void* vtkNotUsed(clientData),
                            void* vtkNotUsed(callData));

  void
  viewportModified ();

  static void
  viewportActorUpdateCallback (vtkObject* caller, unsigned long int vtkNotUsed(eventId), void* vtkNotUsed(clientData),
                               void* vtkNotUsed(callData));

  void
  viewportActorUpdate ();

  static void
  viewportHudUpdateCallback (vtkObject* caller, unsigned long int vtkNotUsed(eventId), void* vtkNotUsed(clientData),
                             void* vtkNotUsed(callData));

  void
  viewportHudUpdate ();

  // Members
  // -----------------------------------------------------------------------------
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkCallbackCommand> viewport_modified_callback_;
  vtkSmartPointer<vtkCallbackCommand> viewport_actor_update_callback_;
  vtkSmartPointer<vtkCallbackCommand> viewport_hud_callback_;

  vtkSmartPointer<vtkTextActor> camera_hud_actor_;
  vtkSmartPointer<vtkTextActor> fps_hud_actor_;
  vtkSmartPointer<vtkTextActor> points_hud_actor_;
};
