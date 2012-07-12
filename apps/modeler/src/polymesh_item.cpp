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

#include <pcl/apps/modeler/polymesh_item.h>
#include <pcl/apps/modeler/cloud_item.h>
#include <pcl/apps/modeler/normal_item.h>
#include <pcl/apps/modeler/render_widget.h>
#include <pcl/apps/modeler/main_window.h>
#include <pcl/apps/modeler/tree_model.h>
#include <pcl/io/pcd_io.h>

#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <QMenu>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PolymeshItem::PolymeshItem (MainWindow* main_window, const std::string &filename): 
  filename_(filename),
  TreeItem(main_window, filename.c_str())
{
  setCheckable(true);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PolymeshItem::~PolymeshItem ()
{
}


//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::updateOnInserted()
{
  open();
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::updateOnAboutToBeRemoved()
{
  while (hasChildren())
    removeRow(0);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::RenderWidget*
pcl::modeler::PolymeshItem::getParent()
{
  return (dynamic_cast<RenderWidget*>(TreeItem::parent()));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::open()
{
  Eigen::Vector4f origin;
  Eigen::Quaternionf orientation;
  int version;

  pcl::PCDReader pcd;
  if (pcd.read (filename_, cloud, origin, orientation, version) < 0)
    return;
  if (cloud.width * cloud.height == 0)
    return;

  Eigen::Matrix3f rotation;
  rotation = orientation;

  vtkSmartPointer<vtkCamera> camera = getParent()->getRenderer()->GetActiveCamera();
  camera->SetPosition(origin[0], origin[1], origin[2]);
  camera->SetFocalPoint(origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2));
  camera->SetViewUp(rotation (0, 1), rotation (1, 1), rotation (2, 1));

  CloudItem* cloud_item = new CloudItem(main_window_, origin, orientation);
  appendRow(cloud_item);

  //attachNormalItem();

  return;
}

//////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::PolymeshItem::concatenatePointCloud (const PointCloud2& cloud, PointCloud2& cloud_out)
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
void
pcl::modeler::PolymeshItem::save(const std::vector<PolymeshItem*>& polymesh_items, const std::string& filename)
{
  if (polymesh_items.empty())
    return;
  
  PointCloud2Ptr cloud = (polymesh_items.size() == 1)?
    (polymesh_items[0]->getCloud()):(boost::shared_ptr<PointCloud2>(new PointCloud2()));
  for (size_t i = 1, i_end = polymesh_items.size(); i < i_end; ++ i)
    concatenatePointCloud(polymesh_items[i]->cloud, *cloud);

  pcl::PCDWriter pcd;
  pcd.writeBinaryCompressed(filename, *cloud);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::prepareContextMenu(QMenu* menu) const
{
  Ui::MainWindow* ui = main_window_->ui();
  menu->addMenu(ui->menuPointCloudFilter);
  menu->addMenu(ui->menuSurfaceReconstruction);
  menu->addAction(ui->actionSavePointCloud);
  menu->addAction(ui->actionClosePointCloud);

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::string>
pcl::modeler::PolymeshItem::getAvaiableFieldNames() const
{
  const std::vector< ::sensor_msgs::PointField >& fields = cloud.fields;

  std::vector<std::string> field_names;
  for (size_t i = 0, i_end = fields.size(); i < i_end; ++ i)
  {
    field_names.push_back(fields[i].name);
  }

  return (field_names);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::updateGeometryItems()
{
  for (int i  = 0, i_end = rowCount(); i < i_end; ++ i)
    dynamic_cast<TreeModel*>(model())->emitItemChanged(child(i));
  
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::attachNormalItem()
{
  NormalItem* normal_item = new NormalItem(main_window_);
  appendRow(normal_item);
  if (!normal_item->isCapable())
  {
    removeRow(normal_item->row());
    delete normal_item;
  }

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PolymeshItem::setNormalField(pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  //int field_normal_x_idx = pcl::getFieldIndex (cloud, "normal_x");
  //int field_normal_y_idx = pcl::getFieldIndex (cloud, "normal_y");
  //int field_normal_z_idx = pcl::getFieldIndex (cloud, "normal_z");

  //// can not handle such case for now
  //if (field_normal_x_idx+field_normal_y_idx+field_normal_z_idx != -3
  //  || field_normal_x_idx+field_normal_y_idx+field_normal_z_idx != -3)
  //  assert(false);

  //if (field_normal_x_idx == -1)
  //{
  //  std::vector<pcl::uint8_t> old_data = cloud.data;
  //  cloud.data.resize(old_data.size() + sizeof(float)*3*normals->size());
  //  cloud.point_step = cloud.point_step + sizeof(float)*3;
  //}

  //for (size_t i = 0, i_end = normals->size(); i < i_end; ++ i)
  //{

  //}

  //// Fill point cloud binary data (padding and all)
  //size_t data_size = sizeof (PointT) * cloud.points.size ();
  //msg.data.resize (data_size);
  //memcpy (&msg.data[0], &cloud.points[0], data_size);

  //// Fill fields metadata
  //msg.fields.clear ();
  //for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));

  //msg.header     = cloud.header;
  //msg.point_step = sizeof (PointT);
  //msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
  //msg.is_dense   = cloud.is_dense;
  ///// @todo msg.is_bigendian = ?;
}
