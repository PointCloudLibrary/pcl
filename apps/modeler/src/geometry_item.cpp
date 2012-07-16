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

#include <pcl/apps/modeler/geometry_item.h>
#include <pcl/apps/modeler/cloud_item.h>
#include <pcl/apps/modeler/tree_model.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/main_window.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::GeometryItem::GeometryItem(MainWindow* main_window) : 
  TreeItem(main_window)
{
  setCheckable(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::GeometryItem::GeometryItem(MainWindow* main_window, const QString & text) : 
  TreeItem(main_window, text)
{
  setCheckable(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::GeometryItem::GeometryItem(MainWindow* main_window, const QIcon & icon, const QString & text) : 
  TreeItem(main_window, icon, text)
{
  setCheckable(true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::GeometryItem::~GeometryItem ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWidget*
pcl::modeler::GeometryItem::getRenderWidget()
{
  RenderWidget* render_widget = NULL;

  TreeItem* item = parent();
  while (item != NULL)
  {
    render_widget = dynamic_cast<RenderWidget*>(item->parent());
    if (render_widget != NULL)
      return (render_widget);
    item = item->parent();
  }

  return (render_widget);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::GeometryItem::updateOnInserted()
{
  initHandlers();
  createActor();

  RenderWidget* render_widget = getRenderWidget();
  render_widget->getRenderer()->AddActor(actor_);
  render_widget->GetRenderWindow()->Render();

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::GeometryItem::updateOnAboutToBeRemoved()
{
  RenderWidget* render_widget = getRenderWidget();
  if (render_widget != NULL)
  {
    render_widget->getRenderer()->RemoveActor(actor_);
    render_widget->GetRenderWindow()->Render();
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::GeometryItem::handleDataChange()
{
  TreeItem::handleDataChange();

  RenderWidget* render_widget = getRenderWidget();
  if (checkState() != old_state_->checkState())
  {
    render_widget->getRenderer()->RemoveActor(actor_);
    if (checkState() != Qt::Unchecked)
      render_widget->getRenderer()->AddActor(actor_);
  }

  updateActor();
  render_widget->GetRenderWindow()->Render();

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::GeometryItem::setColorHandler(double r, double g, double b)
{
  PointCloud2Ptr cloud = dynamic_cast<CloudItem*>(parent())->getCloud();
  color_handler_.reset(new pcl::visualization::PointCloudColorHandlerCustom<PointCloud2>(cloud, r, g, b));

  dynamic_cast<TreeModel*>(model())->emitItemChanged(this);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::GeometryItem::setColorHandler(const std::string& field_name)
{
  PointCloud2Ptr cloud = dynamic_cast<CloudItem*>(parent())->getCloud();
  if (field_name == "rgb")
  {
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud2>(cloud));
  }
  else if (field_name == "hsv")
  {
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerHSVField<PointCloud2>(cloud));
  }
  else
  {
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerGenericField<PointCloud2>(cloud, field_name));
  }

  dynamic_cast<TreeModel*>(model())->emitItemChanged(this);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::GeometryItem::prepareContextMenu(QMenu* menu) const
{
  Ui::MainWindow* ui = main_window_->ui();
  menu->addAction(ui->actionSwitchColorHandler);
}


/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::GeometryItem::isCapable()
{
  return (geometry_handler_->isCapable() && color_handler_->isCapable());
}
