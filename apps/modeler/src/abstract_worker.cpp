/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#include <pcl/apps/modeler/abstract_worker.h>

#include <pcl/apps/modeler/qt.h>
#include <pcl/apps/modeler/parameter_dialog.h>
#include <pcl/apps/modeler/cloud_mesh_item.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::AbstractWorker::AbstractWorker(const QList<CloudMeshItem*>& cloud_mesh_items, QWidget* parent) :
  cloud_mesh_items_(cloud_mesh_items),
  parameter_dialog_(new ParameterDialog(getName(), parent))
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::AbstractWorker::~AbstractWorker(void)
{
  parameter_dialog_->deleteLater();
}

//////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::modeler::AbstractWorker::exec()
{
  for (QList<CloudMeshItem*>::iterator cloud_mesh_items_it = cloud_mesh_items_.begin();
    cloud_mesh_items_it != cloud_mesh_items_.end();
    ++ cloud_mesh_items_it)
    initParameters(*cloud_mesh_items_it);

  setupParameters();

  return (parameter_dialog_->exec());
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::AbstractWorker::process()
{
  for (QList<CloudMeshItem*>::iterator cloud_mesh_items_it = cloud_mesh_items_.begin();
    cloud_mesh_items_it != cloud_mesh_items_.end();
    ++ cloud_mesh_items_it)
  {
    processImpl(*cloud_mesh_items_it);
  }

  emit finished();

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::AbstractWorker::emitDataUpdated(CloudMeshItem* cloud_mesh_item)
{
  emit dataUpdated(cloud_mesh_item);
}
