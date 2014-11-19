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

#include <pcl/apps/modeler/points_actor_item.h>

#include <pcl/apps/modeler/cloud_mesh.h>

#include <vtkVertexGlyphFilter.h>
#include <vtkLODActor.h>
#include <vtkDataSetMapper.h>
#include <vtkPointData.h>
#include <vtkProperty.h>


//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PointsActorItem::PointsActorItem(QTreeWidgetItem* parent,
                                               const boost::shared_ptr<CloudMesh>& cloud_mesh,
                                               const vtkSmartPointer<vtkRenderWindow>& render_window)
  :ChannelActorItem(parent, cloud_mesh, render_window, vtkSmartPointer<vtkLODActor>::New(), "Points")
{
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::PointsActorItem::~PointsActorItem ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PointsActorItem::initImpl()
{
  poly_data_->SetPoints(cloud_mesh_->getVtkPoints());

  vtkSmartPointer<vtkVertexGlyphFilter> vertex_glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION < 6
  vertex_glyph_filter->AddInput(poly_data_);
#else
  vertex_glyph_filter->AddInputData (poly_data_);
#endif
  vertex_glyph_filter->Update();

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New();
  mapper->SetInputConnection(vertex_glyph_filter->GetOutputPort());

  vtkSmartPointer<vtkDataArray> scalars;
  cloud_mesh_->getColorScalarsFromField(scalars, color_scheme_);
  poly_data_->GetPointData ()->SetScalars (scalars);

  double minmax[2];
  scalars->GetRange(minmax);
  mapper->SetScalarRange(minmax);

  mapper->SetScalarModeToUsePointData();
  mapper->InterpolateScalarsBeforeMappingOn();
  mapper->ScalarVisibilityOn();
  mapper->ImmediateModeRenderingOff();

  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>(dynamic_cast<vtkLODActor*>(actor_.GetPointer()));
  actor->SetMapper(mapper);

  actor->SetNumberOfCloudPoints(int(std::max<vtkIdType> (1, poly_data_->GetNumberOfPoints () / 10)));
  actor->GetProperty()->SetInterpolationToFlat();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PointsActorItem::updateImpl()
{
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
pcl::modeler::PointsActorItem::prepareContextMenu(QMenu*) const
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PointsActorItem::prepareProperties(ParameterDialog*)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::PointsActorItem::setProperties()
{

}
