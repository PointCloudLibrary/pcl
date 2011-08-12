/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id: interactor.cpp 1829 2011-07-18 19:26:51Z mdixon $
 *
 */

#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/interactor_style.h>
#include <vtkPointPicker.h>
#include <vtkRendererCollection.h>

void
pcl::visualization::PointPickingCallback::Execute (vtkObject *caller, unsigned long eventid, void*)
{
  vtkRenderWindowInteractor* iren = reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->GetInteractor ();

  if ((eventid == vtkCommand::LeftButtonPressEvent) && (iren->GetShiftKey () == 1)) 
  {
    int idx = performSinglePick (iren);
    // Create a PointPickingEvent
    PointPickingEvent event (idx);
    reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->point_picking_signal_ (event);
  }
  // Call the parent's class mouse events
  reinterpret_cast<pcl::visualization::PCLVisualizerInteractorStyle*>(caller)->OnLeftButtonDown ();
}

int
pcl::visualization::PointPickingCallback::performSinglePick (vtkRenderWindowInteractor *iren)
{
  int mouse_x, mouse_y;
  vtkPointPicker *picker = (vtkPointPicker*)iren->GetPicker ();
  iren->GetMousePosition (&mouse_x, &mouse_y);
  iren->StartPickCallback ();

  vtkRendererCollection *coll = iren->GetRenderWindow ()->GetRenderers ();
  picker->Pick (mouse_x, mouse_y, 0.0, coll->GetFirstRenderer ());
  return ((int)picker->GetPointId ());
}

