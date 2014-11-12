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


#include <pcl/apps/cloud_composer/point_selectors/click_trackball_interactor_style.h>
#include <pcl/apps/cloud_composer/point_selectors/manipulation_event.h>
#include <pcl/apps/cloud_composer/project_model.h>


namespace pcl
{
  namespace cloud_composer
  {
    vtkStandardNewMacro(ClickTrackballStyleInteractor);
  }
}

pcl::cloud_composer::ClickTrackballStyleInteractor::ClickTrackballStyleInteractor ()
  : vtkInteractorStyleTrackballActor ()
{
  manipulation_complete_event_ = interactor_events::MANIPULATION_COMPLETE_EVENT;
  start_matrix_= vtkSmartPointer<vtkMatrix4x4>::New ();
  end_matrix_ = vtkSmartPointer<vtkMatrix4x4>::New ();
  transform_ = vtkSmartPointer<vtkTransform>::New ();
}

pcl::cloud_composer::ClickTrackballStyleInteractor::~ClickTrackballStyleInteractor ()
{
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnLeftButtonDown ()
{
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
  
  vtkActor* selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
    selected_actor->GetMatrix (start_matrix_);
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnRightButtonDown ()
{
  vtkInteractorStyleTrackballActor::OnRightButtonDown();
  
  vtkActor* selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
    selected_actor->GetMatrix (start_matrix_);
  
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnLeftButtonUp ()
{
  vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  vtkSmartPointer<vtkActor> selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
  {
    ManipulationEvent* manip_event = new ManipulationEvent ();
    //Fetch the actor we manipulated
    
    selected_actor->GetMatrix (end_matrix_);
    // Find the id of the actor we manipulated
    pcl::visualization::CloudActorMap::const_iterator end = actors_->end ();
    QString manipulated_id;
    for( pcl::visualization::CloudActorMap::const_iterator itr = actors_->begin (); itr != end; ++itr)
    {
      //qDebug () << "Id = "<<QString::fromStdString (itr->first);
      if ( (itr->second).actor == selected_actor)
      {
        manipulated_id = (QString::fromStdString (itr->first));
        
      }
    }
    if ( !manipulated_id.isEmpty() )
    {
      manip_event->addManipulation (manipulated_id, start_matrix_, end_matrix_);
      this->InvokeEvent (this->manipulation_complete_event_, manip_event);
    }
    else
    {
      qWarning () << "Could not find actor which matches manipulated actor in ClickTrackballStyleInteractor::OnLeftButtonUp!!!";
    }
  }
}

void
pcl::cloud_composer::ClickTrackballStyleInteractor::OnRightButtonUp ()
{
  vtkInteractorStyleTrackballActor::OnRightButtonUp();
  vtkSmartPointer<vtkActor> selected_actor = vtkActor::SafeDownCast(this->InteractionProp);
  if (selected_actor)
  {
    ManipulationEvent* manip_event = new ManipulationEvent ();
    //Fetch the actor we manipulated
    
    selected_actor->GetMatrix (end_matrix_);
    // Find the id of the actor we manipulated
    pcl::visualization::CloudActorMap::const_iterator end = actors_->end ();
    QString manipulated_id;
    for( pcl::visualization::CloudActorMap::const_iterator itr = actors_->begin (); itr != end; ++itr)
    {
      //qDebug () << "Id = "<<QString::fromStdString (itr->first);
      if ( (itr->second).actor == selected_actor)
      {
        manipulated_id = (QString::fromStdString (itr->first));
        
      }
    }
    if ( !manipulated_id.isEmpty() )
    {
      manip_event->addManipulation (manipulated_id, start_matrix_, end_matrix_);
      this->InvokeEvent (this->manipulation_complete_event_, manip_event);
    }
    else
    {
      qWarning () << "Could not find actor which matches manipulated actor in ClickTrackballStyleInteractor::OnRightButtonUp!!!";
    }
  }
   
}


