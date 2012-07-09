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
#include <pcl/apps/modeler/dock_widget.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/cloud_actor.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>


/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PCLModeler::PCLModeler (MainWindow* main_window) : 
  QStandardItemModel (main_window),
  main_window_ (main_window)
{
  connect (this, SIGNAL (rowsInserted (const QModelIndex &, int, int )), this, SLOT (slotUpdateRenderWidgetTitle ()));
  connect (this, SIGNAL (rowsRemoved (const QModelIndex &, int, int )), this, SLOT (slotUpdateRenderWidgetTitle ()));
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PCLModeler::~PCLModeler ()
{
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

  CloudActor* cloud_actor = new CloudActor(main_window_, cloud, filename, origin, orientation);
  renderer->AddActor(cloud_actor->getActor());
  renderer->GetRenderWindow()->Render();

  main_window_->getActiveRenderWidget()->appendRow(cloud_actor);
  cloud_actor->setCheckState(Qt::Checked);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PCLModeler::closePointCloud()
{
  std::vector<CloudActor*> cloud_actors = main_window_->getSelectedCloud();

  if (cloud_actors.empty())
    return;

  for (size_t i = 0, i_end = cloud_actors.size(); i < i_end; ++ i)
  {
    RenderWidget* render_widget = dynamic_cast<RenderWidget*>(cloud_actors[i]->TreeItem::parent());
    render_widget->getRenderer()->RemoveActor(cloud_actors[i]->getActor());
    render_widget->GetRenderWindow()->Render();

    removeRow(cloud_actors[i]->row(), indexFromItem(render_widget));
  }

  return;
}


//////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::PCLModeler::concatenatePointCloud (const sensor_msgs::PointCloud2 &cloud, sensor_msgs::PointCloud2 &cloud_out)
{
  if (cloud.fields.size () != cloud_out.fields.size ())
  {
    PCL_ERROR ("[pcl::concatenatePointCloud] Number of fields in cloud1 (%u) != Number of fields in cloud2 (%u)\n", cloud.fields.size (), cloud_out.fields.size ());
    return (false);
  }

  for (size_t i = 0; i < cloud.fields.size (); ++i)
    if (cloud.fields[i].name != cloud_out.fields[i].name)
    {
      PCL_ERROR ("[pcl::concatenatePointCloud] Name of field %d in cloud1, %s, does not match name in cloud2, %s\n", i, cloud.fields[i].name.c_str (), cloud_out.fields[i].name.c_str () );      
      return (false);
    }

    size_t nrpts = cloud_out.data.size ();
    cloud_out.data.resize (nrpts + cloud.data.size ());
    memcpy (&cloud_out.data[nrpts], &cloud.data[0], cloud.data.size ());

    // Height = 1 => no more organized
    cloud_out.width    = cloud.width * cloud.height + cloud_out.width * cloud_out.height;
    cloud_out.height   = 1;
    if (!cloud.is_dense || !cloud_out.is_dense)
      cloud_out.is_dense = false;
    else
      cloud_out.is_dense = true;

    return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::PCLModeler::savePointCloud(const std::string& filename)
{
  std::vector<CloudActor*> cloud_actors = main_window_->getSelectedCloud();

  if (cloud_actors.empty())
    return (false);

  pcl::PCDWriter pcd;

  if (cloud_actors.size() == 1)
    return (pcd.write (filename, *(cloud_actors.front()->getCloud())) >= 0);

  sensor_msgs::PointCloud2 cloud = *(cloud_actors[0]->getCloud());

  for (size_t i = 1, i_end = cloud_actors.size(); i < i_end; ++ i)
    concatenatePointCloud(*(cloud_actors[i]->getCloud()), cloud);

  return (pcd.write (filename, cloud) >= 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PCLModeler::slotUpdateRenderWidgetTitle()
{
  for (int i = 0, i_end = rowCount(); i < i_end; ++ i)
  {
    RenderWidget* render_widget = dynamic_cast<RenderWidget*>(itemFromIndex(index(i, 0)));
    render_widget->TreeItem::setText((i == 0)?(QObject::tr("Main Render Window")):(QObject::tr("Render Window %1").arg(i)));

    DockWidget* dock_widget = dynamic_cast<DockWidget*>(render_widget->QVTKWidget::parent());
    if (dock_widget != NULL)
      dock_widget->setWindowTitle(tr("Render Window %1").arg(i));
  }

  return;
}
