/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/apps/optronic_viewer/cloud_filter.h>

#include <pcl/filters/voxel_grid.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
VoxelGridCF::
VoxelGridCF ()
: filter_selection_page_ (new QWizardPage ())
, voxel_grid_size_x_ (0.01)
, voxel_grid_size_y_ (0.01)
, voxel_grid_size_z_ (0.01)
{
  voxel_grid_size_x_label_ = new QLabel (QObject::tr ("Voxel Grid Size in X-Direction:"));
  voxel_grid_size_x_line_edit_ = new QLineEdit ();
  voxel_grid_size_y_label_ = new QLabel (QObject::tr ("Voxel Grid Size in Y-Direction:"));
  voxel_grid_size_y_line_edit_ = new QLineEdit ();
  voxel_grid_size_z_label_ = new QLabel (QObject::tr ("Voxel Grid Size in Z-Direction:"));
  voxel_grid_size_z_line_edit_ = new QLineEdit ();

  QDoubleValidator * double_validator = new QDoubleValidator ();
  voxel_grid_size_x_line_edit_->setValidator (double_validator);
  voxel_grid_size_y_line_edit_->setValidator (double_validator);
  voxel_grid_size_z_line_edit_->setValidator (double_validator);

  std::stringstream ss_x;
  ss_x << voxel_grid_size_x_;
  voxel_grid_size_x_line_edit_->setText (QString (ss_x.str ().c_str ()));

  std::stringstream ss_y;
  ss_y << voxel_grid_size_y_;
  voxel_grid_size_y_line_edit_->setText (QString (ss_y.str ().c_str ()));

  std::stringstream ss_z;
  ss_z << voxel_grid_size_z_;
  voxel_grid_size_z_line_edit_->setText (QString (ss_z.str ().c_str ()));

  main_layout_ = new QVBoxLayout (filter_selection_page_);
  main_layout_->addWidget (voxel_grid_size_x_label_);
  main_layout_->addWidget (voxel_grid_size_x_line_edit_);
  main_layout_->addWidget (voxel_grid_size_y_label_);
  main_layout_->addWidget (voxel_grid_size_y_line_edit_);
  main_layout_->addWidget (voxel_grid_size_z_label_);
  main_layout_->addWidget (voxel_grid_size_z_line_edit_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
VoxelGridCF::
~VoxelGridCF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
VoxelGridCF::
filter (
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out)
{
  voxel_grid_size_x_ = voxel_grid_size_x_line_edit_->text ().toFloat ();
  voxel_grid_size_y_ = voxel_grid_size_y_line_edit_->text ().toFloat ();
  voxel_grid_size_z_ = voxel_grid_size_z_line_edit_->text ().toFloat ();

  pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (voxel_grid_size_x_, voxel_grid_size_y_, voxel_grid_size_z_);
  sor.filter (*cloud_out);
}

 //////////////////////////////////////////////////////////////////////////////////////////////
std::vector<pcl::apps::optronic_viewer::Parameter*>
pcl::apps::optronic_viewer::
VoxelGridCF::
getParameters ()
{
  return (parameters_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::apps::optronic_viewer::
VoxelGridCF::
setParameters (const pcl::apps::optronic_viewer::Parameter &parameter)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
VoxelGridCFF::
VoxelGridCFF ()
: CloudFilterFactory ("VoxelGrid Filter")
//, filter_selection_page_ (new QWizardPage ())
{
  //voxel_grid_size_x_label_ = new QLabel (QObject::tr ("Voxel Grid Size in X-Direction:"));
  //voxel_grid_size_y_label_ = new QLabel (QObject::tr ("Voxel Grid Size in Y-Direction:"));
  //voxel_grid_size_z_label_ = new QLabel (QObject::tr ("Voxel Grid Size in Z-Direction:"));

  //main_layout_ = new QVBoxLayout (filter_selection_page_);
  //main_layout_->addWidget (voxel_grid_size_x_label_);
  //main_layout_->addWidget (voxel_grid_size_y_label_);
  //main_layout_->addWidget (voxel_grid_size_z_label_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::apps::optronic_viewer::
VoxelGridCFF::
~VoxelGridCFF ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
QWizardPage *
pcl::apps::optronic_viewer::
VoxelGridCF::
getParameterPage ()
{
  return (filter_selection_page_);
}

