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


#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/cloud_viewer.h>
#include <pcl/apps/cloud_composer/project_model.h>
#include <pcl/apps/cloud_composer/cloud_view.h>

pcl::cloud_composer::CloudViewer::CloudViewer (QWidget* parent)
  : QTabWidget (parent)

{
  connect (this, SIGNAL (currentChanged (int)),
           this, SLOT (modelChanged (int)));
}

pcl::cloud_composer::CloudViewer::~CloudViewer ()
{
  
}

void
pcl::cloud_composer::CloudViewer::addModel (ProjectModel* new_model)
{
  CloudView* new_view = new CloudView (new_model);
  connect (new_model->getSelectionModel (), SIGNAL (selectionChanged (QItemSelection,QItemSelection)),
           new_view, SLOT (selectedItemChanged (QItemSelection,QItemSelection)));
  new_model->setCloudView (new_view);
  
  QStandardItem* title = new_model->horizontalHeaderItem (0);
  this->addTab (new_view, title->text ());
  
  model_view_map_.insert (new_model,new_view);
  
  setCurrentWidget (model_view_map_.value (new_model));
  //Refresh the view
  new_view->refresh ();

}
  
pcl::cloud_composer::ProjectModel*
pcl::cloud_composer::CloudViewer::getModel () const
{
  if (this->count() == 0)
    return 0;
  else
    return dynamic_cast<CloudView*> (currentWidget ())->getModel (); 
}

void
pcl::cloud_composer::CloudViewer::addNewProject (ProjectModel* new_model)
{
  //If we're already there, abort
  if (new_model == getModel ())
    return;
  //Check whether we've seen the model yet
  if ( !model_view_map_.contains (new_model))
  {
    addModel (new_model);
  }
  else
  {
    setCurrentWidget (model_view_map_.value (new_model));
    //Refresh the view
    model_view_map_.value (new_model)->refresh ();
  }
}

void
pcl::cloud_composer::CloudViewer::modelChanged (int)
{
  emit newModelSelected (getModel ());
}

