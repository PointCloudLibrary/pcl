/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#pragma once
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkUnstructuredGrid.h>

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> vtkSmartPointer<vtkDataSet> 
pcl::visualization::createPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
  vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
  if (cloud->points.empty ())
    return (poly_grid);

  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkPolygon> polygon    = vtkSmartPointer<vtkPolygon>::New ();

  poly_points->SetNumberOfPoints (cloud->points.size ());
  polygon->GetPointIds ()->SetNumberOfIds (cloud->points.size ());

  size_t i;
  for (i = 0; i < cloud->points.size (); ++i)
  {
    poly_points->SetPoint (i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    polygon->GetPointIds ()->SetId (i, i);
  }

  allocVtkUnstructuredGrid (poly_grid);
  poly_grid->Allocate (1, 1);
  poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
  poly_grid->SetPoints (poly_points);

  return (poly_grid);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> vtkSmartPointer<vtkDataSet> 
pcl::visualization::createPolygon (const pcl::PlanarPolygon<PointT> &planar_polygon)
{
  vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
  if (planar_polygon.getContour ().empty ())
    return (poly_grid);

  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkPolygon> polygon    = vtkSmartPointer<vtkPolygon>::New ();

  poly_points->SetNumberOfPoints (planar_polygon.getContour ().size () + 1);
  polygon->GetPointIds ()->SetNumberOfIds (planar_polygon.getContour ().size () + 1);

  size_t i;
  for (i = 0; i < planar_polygon.getContour ().size (); ++i)
  {
    poly_points->SetPoint (i, planar_polygon.getContour ()[i].x, 
                              planar_polygon.getContour ()[i].y, 
                              planar_polygon.getContour ()[i].z);
    polygon->GetPointIds ()->SetId (i, i);
  }

  poly_points->SetPoint (i, planar_polygon.getContour ()[0].x, 
                            planar_polygon.getContour ()[0].y, 
                            planar_polygon.getContour ()[0].z);
  polygon->GetPointIds ()->SetId (i, i);
  
  allocVtkUnstructuredGrid (poly_grid);
  poly_grid->Allocate (1, 1);
  poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
  poly_grid->SetPoints (poly_points);

  return (poly_grid);
}

