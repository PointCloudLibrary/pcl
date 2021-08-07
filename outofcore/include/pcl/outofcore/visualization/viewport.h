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
