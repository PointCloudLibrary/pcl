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

#include <pcl/apps/modeler/surface_item.h>
#include <pcl/apps/modeler/polymesh_item.h>
#include <pcl/apps/modeler/main_window.h>

#include <QMenu>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::SurfaceItem::SurfaceItem (MainWindow* main_window) : 
  GeometryItem(main_window, "Polygonal Surface")
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::SurfaceItem::~SurfaceItem ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SurfaceItem::initHandlers()
{
  PointCloud2Ptr cloud = dynamic_cast<PolymeshItem*>(parent())->getCloud();

  geometry_handler_.reset(new pcl::visualization::PointCloudGeometryHandlerXYZ<PointCloud2>(cloud));

  color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud2>(cloud));
  if (!color_handler_->isCapable())
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRandom<PointCloud2>(cloud));
}


//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::SurfaceItem::createActor ()
{
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::SurfaceItem::prepareContextMenu(QMenu* menu) const
{
  GeometryItem::prepareContextMenu(menu);
}
