/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: point_cloud_handlers.hpp 7678 2012-10-22 20:54:04Z rusu $
 *
 */
#ifndef PCL_POINT_CLOUD_GEOMETRY_HANDLERS_IMPL_HPP_
#define PCL_POINT_CLOUD_GEOMETRY_HANDLERS_IMPL_HPP_

#include <pcl/console/time.h>
#include <pcl/pcl_macros.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::visualization::PointCloudGeometryHandlerXYZ<PointT>::PointCloudGeometryHandlerXYZ (const PointCloudConstPtr &cloud) 
  : pcl::visualization::PointCloudGeometryHandler<PointT>::PointCloudGeometryHandler (cloud)
{
  field_x_idx_ = pcl::getFieldIndex (*cloud, "x", fields_);
  if (field_x_idx_ == -1)
    return;
  field_y_idx_ = pcl::getFieldIndex (*cloud, "y", fields_);
  if (field_y_idx_ == -1)
    return;
  field_z_idx_ = pcl::getFieldIndex (*cloud, "z", fields_);
  if (field_z_idx_ == -1)
    return;
  capable_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::visualization::PointCloudGeometryHandlerXYZ<PointT>::getGeometry (vtkSmartPointer<vtkPoints> &points) const
{
  if (!capable_)
    return;

  if (!points)
    points = vtkSmartPointer<vtkPoints>::New ();
  points->SetDataTypeToFloat ();

  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);
  vtkIdType nr_points = cloud_->points.size ();

  // Add all points
  vtkIdType j = 0;    // true point index
  float* pts = static_cast<float*> (malloc (nr_points * 3 * sizeof (float)));

  // If the dataset has no invalid values, just copy all of them
  if (cloud_->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      pts[i * 3 + 0] = cloud_->points[i].x;
      pts[i * 3 + 1] = cloud_->points[i].y;
      pts[i * 3 + 2] = cloud_->points[i].z;
    }
    data->SetArray (&pts[0], nr_points * 3, 0);
    points->SetData (data);
  }
  // Need to check for NaNs, Infs, ec
  else
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud_->points[i].x) || !pcl_isfinite (cloud_->points[i].y) || !pcl_isfinite (cloud_->points[i].z))
        continue;

      pts[j * 3 + 0] = cloud_->points[i].x;
      pts[j * 3 + 1] = cloud_->points[i].y;
      pts[j * 3 + 2] = cloud_->points[i].z;
      // Set j and increment
      j++;
    }
    data->SetArray (&pts[0], j * 3, 0);
    points->SetData (data);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<PointT>::PointCloudGeometryHandlerSurfaceNormal (const PointCloudConstPtr &cloud) 
  : pcl::visualization::PointCloudGeometryHandler<PointT>::PointCloudGeometryHandler (cloud)
{
  field_x_idx_ = pcl::getFieldIndex (*cloud, "normal_x", fields_);
  if (field_x_idx_ == -1)
    return;
  field_y_idx_ = pcl::getFieldIndex (*cloud, "normal_y", fields_);
  if (field_y_idx_ == -1)
    return;
  field_z_idx_ = pcl::getFieldIndex (*cloud, "normal_z", fields_);
  if (field_z_idx_ == -1)
    return;
  capable_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<PointT>::getGeometry (vtkSmartPointer<vtkPoints> &points) const
{
  if (!capable_)
    return;

  if (!points)
    points = vtkSmartPointer<vtkPoints>::New ();
  points->SetDataTypeToFloat ();
  points->SetNumberOfPoints (cloud_->points.size ());

  // Add all points
  double p[3];
  for (vtkIdType i = 0; i < static_cast<vtkIdType> (cloud_->points.size ()); ++i)
  {
    p[0] = cloud_->points[i].normal[0];
    p[1] = cloud_->points[i].normal[1];
    p[2] = cloud_->points[i].normal[2];

    points->SetPoint (i, p);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
pcl::visualization::PointCloudGeometryHandlerCustom<PointT>::PointCloudGeometryHandlerCustom (const PointCloudConstPtr &cloud,
    const std::string &x_field_name, const std::string &y_field_name, const std::string &z_field_name) : pcl::visualization::PointCloudGeometryHandler<PointT>::PointCloudGeometryHandler (cloud)
{
  field_x_idx_ = pcl::getFieldIndex (*cloud, x_field_name, fields_);
  if (field_x_idx_ == -1)
    return;
  field_y_idx_ = pcl::getFieldIndex (*cloud, y_field_name, fields_);
  if (field_y_idx_ == -1)
    return;
  field_z_idx_ = pcl::getFieldIndex (*cloud, z_field_name, fields_);
  if (field_z_idx_ == -1)
    return;
  field_name_ = x_field_name + y_field_name + z_field_name;
  capable_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::visualization::PointCloudGeometryHandlerCustom<PointT>::getGeometry (vtkSmartPointer<vtkPoints> &points) const
{
  if (!capable_)
    return;

  if (!points)
    points = vtkSmartPointer<vtkPoints>::New ();
  points->SetDataTypeToFloat ();
  points->SetNumberOfPoints (cloud_->points.size ());

  float data;
  // Add all points
  double p[3];
  for (vtkIdType i = 0; i < static_cast<vtkIdType> (cloud_->points.size ()); ++i)
  {
    // Copy the value at the specified field
    const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud_->points[i]);
    memcpy (&data, pt_data + fields_[field_x_idx_].offset, sizeof (float));
    p[0] = data;

    memcpy (&data, pt_data + fields_[field_y_idx_].offset, sizeof (float));
    p[1] = data;

    memcpy (&data, pt_data + fields_[field_z_idx_].offset, sizeof (float));
    p[2] = data;

    points->SetPoint (i, p);
  }
}

#endif      // PCL_POINT_CLOUD_GEOMETRY_HANDLERS_IMPL_HPP_


