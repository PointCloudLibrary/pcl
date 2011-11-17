/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: shapes.hpp 2186 2011-08-23 19:30:28Z bouffa $
 *
 */

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> vtkSmartPointer<vtkDataSet> 
pcl::visualization::createPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkPolygon> polygon    = vtkSmartPointer<vtkPolygon>::New ();

  poly_points->SetNumberOfPoints (cloud->points.size ());
  polygon->GetPointIds ()->SetNumberOfIds (cloud->points.size ());

  size_t i;
  for (i = 0; i < cloud->points.size (); ++i)
  {
    poly_points->InsertPoint (i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    polygon->GetPointIds ()->SetId (i, i);
  }

  vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
  allocVtkUnstructuredGrid (poly_grid);
  poly_grid->Allocate (1, 1);
  poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
  poly_grid->SetPoints (poly_points);
  poly_grid->Update ();

  return (poly_grid);
}

