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

#include <pcl/apps/modeler/normals_item.h>
#include <pcl/apps/modeler/cloud_item.h>
#include <pcl/apps/modeler/main_window.h>

#include <QMenu>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::NormalsItem::NormalsItem (MainWindow* main_window) : 
  GeometryItem(main_window, "Normal Vector")
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::NormalsItem::~NormalsItem ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsItem::initHandlers()
{
  PointCloud2Ptr cloud = dynamic_cast<CloudItem*>(parent())->getCloud();

  geometry_handler_.reset(new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<PointCloud2>(cloud));

  color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRGBField<PointCloud2>(cloud));
  if (!color_handler_->isCapable())
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRandom<PointCloud2>(cloud));
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::NormalsItem::createActor ()
{
  if (actor_.GetPointer() == NULL)
    actor_ = vtkSmartPointer<vtkLODActor>::New ();

  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>(dynamic_cast<vtkLODActor*>(actor_.GetPointer()));

  PointCloud2Ptr cloud = dynamic_cast<CloudItem*>(parent())->getCloud();

  vtkSmartPointer<vtkPoints> points;
  pcl::visualization::PointCloudGeometryHandlerXYZ<PointCloud2> point_handler(cloud);
  point_handler.getGeometry(points);

  vtkSmartPointer<vtkPoints> normals;
  geometry_handler_->getGeometry(normals);

  float level = 10;
  float scale = 0.1;

  vtkSmartPointer<vtkPoints> sample_points = vtkSmartPointer<vtkPoints>::New();
  sample_points->SetDataTypeToFloat ();
  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  vtkIdType nr_normals = (points->GetNumberOfPoints() - 1) / level + 1 ;
  float* pts = new float[2 * nr_normals * 3];
  for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = j * level)
  {
    double point[3];
    points->GetPoint(i, point);

    double normal[3];
    normals->GetPoint(i, normal);

    pts[2 * j * 3 + 0] = point[0];
    pts[2 * j * 3 + 1] = point[1];
    pts[2 * j * 3 + 2] = point[2];
    pts[2 * j * 3 + 3] = point[0] + normal[0]*scale;
    pts[2 * j * 3 + 4] = point[1] + normal[1]*scale;
    pts[2 * j * 3 + 5] = point[2] + normal[2]*scale;

    lines->InsertNextCell (2);
    lines->InsertCellPoint (2 * j);
    lines->InsertCellPoint (2 * j + 1);
  }

  data->SetArray (&pts[0], 2 * nr_normals * 3, 0);
  points->SetData (data);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetLines(lines);

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput(polyData);
  mapper->SetColorModeToMapScalars();
  mapper->SetScalarModeToUsePointData();

  actor->SetMapper (mapper);

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::NormalsItem::prepareContextMenu(QMenu* menu) const
{
  GeometryItem::prepareContextMenu(menu);
}
