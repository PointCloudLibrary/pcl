/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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


#include <pcl/apps/modeler/pcl_modeler.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/cloud_actor.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/io/pcd_io.h>


/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PCLModeler::PCLModeler(MainWindow* main_window) : 
  main_window_(main_window),
  QStandardItemModel(main_window)
{
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PCLModeler::~PCLModeler()
{
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::PCLModeler::openPointCloud(const std::string& filename)
{
  sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version;

  pcl::PCDReader pcd;
  if (pcd.read (filename, *cloud, origin, orientation, version) < 0)
    return (false);
  if (cloud->width * cloud->height == 0)
    return (false);

  Eigen::Matrix3f rotation;
  rotation = orientation;

  vtkSmartPointer<vtkRenderer> renderer = main_window_->getActiveRender();
  vtkSmartPointer<vtkCamera> camera = renderer->GetActiveCamera();
  camera->SetPosition(origin[0], origin[1], origin[2]);
  camera->SetFocalPoint(origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2));
  camera->SetViewUp(rotation (0, 1), rotation (1, 1), rotation (2, 1));

  CloudActor::Ptr cloud_actor(new CloudActor(main_window_, cloud, filename, origin, orientation));
  renderer->AddActor(cloud_actor->getActor());
  renderer->GetRenderWindow()->Render();

  cloud_actor_map_[cloud_actor->getActor()] = cloud_actor;

  main_window_->getActiveRenderWidget()->appendRow(cloud_actor.get());

  return (true);
}
