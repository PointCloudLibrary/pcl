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

#include <pcl/apps/modeler/normals_actor_item.h>

#include <pcl/apps/modeler/cloud_mesh.h>
#include <pcl/filters/filter_indices.h>

#include <vtkLODActor.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkDataSetMapper.h>
#include <vtkPointData.h>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::NormalsActorItem::NormalsActorItem(QTreeWidgetItem* parent,
                                               const boost::shared_ptr<CloudMesh>& cloud_mesh,
                                               const vtkSmartPointer<vtkRenderWindow>& render_window)
  :ChannelActorItem(parent, cloud_mesh, render_window, vtkSmartPointer<vtkLODActor>::New(), "Normals"),
  level_(10), scale_(0.1)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::NormalsActorItem::~NormalsActorItem ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsActorItem::createNormalLines()
{
  vtkPoints* points = poly_data_->GetPoints();
  vtkCellArray* lines = poly_data_->GetLines();
  lines->Reset();

  CloudMesh::PointCloudConstPtr cloud = cloud_mesh_->getCloud();
  if (cloud->empty())
    return;

  if (points->GetData() == NULL)
    points->SetData(vtkSmartPointer<vtkFloatArray>::New ());

  vtkFloatArray* data = dynamic_cast<vtkFloatArray*>(points->GetData());
  data->SetNumberOfComponents (3);

  if (cloud->is_dense)
  {
    vtkIdType nr_normals = static_cast<vtkIdType> ((cloud->points.size () - 1) / level_ + 1);
    data->SetNumberOfValues(2*3*nr_normals);
    for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = static_cast<vtkIdType> (j * level_))
    {
      const CloudMesh::PointT& p = cloud->points[i];
      data->SetValue(2*j*3 + 0, p.x);
      data->SetValue(2*j*3 + 1, p.y);
      data->SetValue(2*j*3 + 2, p.z);
      data->SetValue(2*j*3 + 3, float (p.x + p.normal_x*scale_));
      data->SetValue(2*j*3 + 4, float (p.y + p.normal_y*scale_));
      data->SetValue(2*j*3 + 5, float (p.z + p.normal_z*scale_));

      lines->InsertNextCell(2);
      lines->InsertCellPoint(2*j);
      lines->InsertCellPoint(2*j + 1);
    }
  }
  else
  {
    pcl::IndicesPtr indices(new std::vector<int>());
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    vtkIdType nr_normals = static_cast<vtkIdType> ((indices->size () - 1) / level_ + 1);
    data->SetNumberOfValues(2*3*nr_normals);
    for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = static_cast<vtkIdType> (j * level_))
    {
      const CloudMesh::PointT& p= cloud->points[(*indices)[i]];
      data->SetValue (2*j*3 + 0, p.x);
      data->SetValue (2*j*3 + 1, p.y);
      data->SetValue (2*j*3 + 2, p.z);
      data->SetValue (2*j*3 + 3, float (p.x + p.normal_x*scale_));
      data->SetValue (2*j*3 + 4, float (p.y + p.normal_y*scale_));
      data->SetValue (2*j*3 + 5, float (p.z + p.normal_z*scale_));

      lines->InsertNextCell(2);
      lines->InsertCellPoint(2*j);
      lines->InsertCellPoint(2*j + 1);
    }
  }

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsActorItem::initImpl()
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToFloat ();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  poly_data_->SetPoints(points);
  poly_data_->SetLines(lines);

  createNormalLines();

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
#if VTK_MAJOR_VERSION < 6
  mapper->SetInput(poly_data_);
#else
  mapper->SetInputData (poly_data_);
#endif

  vtkSmartPointer<vtkDataArray> scalars;
  cloud_mesh_->getColorScalarsFromField(scalars, color_scheme_);
  poly_data_->GetPointData ()->SetScalars (scalars);
  double minmax[2];
  scalars->GetRange(minmax);
  mapper->SetScalarRange(minmax);

  mapper->SetColorModeToMapScalars();
  mapper->SetScalarModeToUsePointData();

  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>(dynamic_cast<vtkLODActor*>(actor_.GetPointer()));
  actor->SetMapper(mapper);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsActorItem::updateImpl()
{
  createNormalLines();

  vtkSmartPointer<vtkDataArray> scalars;
  cloud_mesh_->getColorScalarsFromField(scalars, color_scheme_);
  poly_data_->GetPointData ()->SetScalars (scalars);
  double minmax[2];
  scalars->GetRange(minmax);
  actor_->GetMapper()->SetScalarRange(minmax);

  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsActorItem::prepareContextMenu(QMenu* ) const
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsActorItem::prepareProperties(ParameterDialog* )
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsActorItem::setProperties()
{

}
