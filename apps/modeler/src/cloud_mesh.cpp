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

#include <pcl/apps/modeler/cloud_mesh.h>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <vtkDataArray.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMesh::CloudMesh()
  :vtk_points_(vtkSmartPointer<vtkPoints>::New()),
  vtk_polygons_(vtkSmartPointer<vtkCellArray>::New())
{
  cloud_.reset(new pcl::PointCloud<pcl::PointSurfel>());
  vtk_points_->SetDataTypeToFloat ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMesh::CloudMesh(PointCloudPtr cloud)
  :cloud_(cloud),
  vtk_points_(vtkSmartPointer<vtkPoints>::New()),
  vtk_polygons_(vtkSmartPointer<vtkCellArray>::New())
{
  vtk_points_->SetDataTypeToFloat ();
  updateVtkPoints();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudMesh::~CloudMesh ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::string>
pcl::modeler::CloudMesh::getAvaiableFieldNames() const
{
  // TODO:
  return (std::vector<std::string>());
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMesh::open(const std::string& filename)
{
  if (pcl::io::loadPCDFile(filename, *cloud_) != 0)
    return (false);

  updateVtkPoints();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMesh::save(const std::string& filename) const
{
  return (save(*cloud_, filename));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudMesh::save(const pcl::PointCloud<pcl::PointSurfel>& cloud, const std::string& filename)
{
  return (pcl::io::savePCDFile(filename, cloud, true) == 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMesh::getColorScalarsFromField(vtkSmartPointer<vtkDataArray> &scalars, const std::string& field) const
{
  if (field == "rgb" || field == "rgba")
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler(cloud_);
    color_handler.getColor(scalars);
    return;
  }

  if (field == "random")
  {
    pcl::visualization::PointCloudColorHandlerRandom<PointT> color_handler(cloud_);
    color_handler.getColor(scalars);
    return;
  }

  pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud_, field);
  color_handler.getColor(scalars);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMesh::updateVtkPoints()
{
  if (vtk_points_->GetData() == NULL)
    vtk_points_->SetData(vtkSmartPointer<vtkFloatArray>::New ());

  vtkFloatArray* data = dynamic_cast<vtkFloatArray*>(vtk_points_->GetData());
  data->SetNumberOfComponents (3);

  // If the dataset has no invalid values, just copy all of them
  if (cloud_->is_dense)
  {
    vtkIdType nr_points = cloud_->points.size ();
    data->SetNumberOfValues(3*nr_points);

    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      data->SetValue(i*3+0, cloud_->points[i].x);
      data->SetValue(i*3+1, cloud_->points[i].y);
      data->SetValue(i*3+2, cloud_->points[i].z);
    }
  }
  // Need to check for NaNs, Infs, ec
  else
  {
    pcl::IndicesPtr indices(new std::vector<int>());
    pcl::removeNaNFromPointCloud(*cloud_, *indices);

    data->SetNumberOfValues(3*indices->size());

    for (vtkIdType i = 0, i_end = indices->size(); i < i_end; ++i)
    {
      vtkIdType idx = (*indices)[i];
      data->SetValue(i*3+0, cloud_->points[idx].x);
      data->SetValue(i*3+1, cloud_->points[idx].y);
      data->SetValue(i*3+2, cloud_->points[idx].z);
    }
  }
  data->Squeeze();

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMesh::updateVtkPolygons()
{
  vtk_polygons_->Reset();

  if (cloud_->is_dense)
  {
    for (size_t i = 0, i_end = polygons_.size (); i < i_end; ++i)
    {
      size_t n_points = polygons_[i].vertices.size ();
      vtk_polygons_->InsertNextCell (n_points);
      for (size_t j = 0; j < n_points; j++)
        vtk_polygons_->InsertCellPoint (polygons_[i].vertices[j]);
    }
  }
  else
  {
    pcl::IndicesPtr indices(new std::vector<int>());
    pcl::removeNaNFromPointCloud(*cloud_, *indices);

    for (size_t i = 0, i_end = polygons_.size(); i < i_end; ++i)
    {
      size_t n_points = polygons_[i].vertices.size ();
      vtk_polygons_->InsertNextCell (n_points);
      for (size_t j = 0; j < n_points; j++)
        vtk_polygons_->InsertCellPoint ((*indices)[polygons_[i].vertices[j]]);
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudMesh::transform(double tx, double ty, double tz, double rx, double ry, double rz)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_, centroid);

  CloudMesh::PointCloud mean_cloud = *cloud_;
  pcl::demeanPointCloud(*cloud_, centroid, mean_cloud);

  rx *= M_PI/180;
  ry *= M_PI/180;
  rz *= M_PI/180;
  Eigen::Affine3f affine_transform = pcl::getTransformation(tx, ty, tz, rx, ry, rz);
  CloudMesh::PointCloud transform_cloud = mean_cloud;
  pcl::transformPointCloudWithNormals(mean_cloud, transform_cloud, affine_transform.matrix());

  pcl::demeanPointCloud(transform_cloud, -centroid, *cloud_);

  return;
}
