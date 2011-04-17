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
 * $Id$
 *
 */

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Create a 3d poly line from a set of points. 
  * \param cloud the set of points used to create the 3d polyline
  */
template <typename PointT> vtkSmartPointer<vtkDataSet> 
  pcl_visualization::createPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
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

  vtkSmartPointer<vtkUnstructuredGrid> poly_grid = vtkSmartPointer<vtkUnstructuredGrid>::New ();
  poly_grid->Allocate (1, 1);
  poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
  poly_grid->SetPoints (poly_points);
  poly_grid->Update ();

  return (poly_grid);
}

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Create a line shape from two points
  * \param pt1 the first point on the line
  * \param pt2 the end point on the line
  */
template <typename P1, typename P2> vtkSmartPointer<vtkDataSet>
  pcl_visualization::createLine (const P1 &pt1, const P2 &pt2)
{
  vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New ();
  line->SetPoint1 (pt1.x, pt1.y, pt1.z);
  line->SetPoint2 (pt2.x, pt2.y, pt2.z);
  line->Update ();

  return (line->GetOutput ());
}

////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Create a sphere shape from a point and a radius
  * \param center the center of the sphere
  * \param radius the radius of the sphere
  * \param res (optional) the resolution used for rendering the model
  */
template <typename PointT> vtkSmartPointer<vtkDataSet> 
  pcl_visualization::createSphere (const PointT &center, double radius, int res)
{
  // Set the sphere origin
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New ();
  t->Identity (); t->Translate (center.x, center.y, center.z);

  vtkSmartPointer<vtkSphereSource> s_sphere = vtkSmartPointer<vtkSphereSource>::New ();
  s_sphere->SetRadius (radius);
  s_sphere->SetPhiResolution (res);
  s_sphere->SetThetaResolution (res);
  s_sphere->LatLongTessellationOff ();
  
  vtkSmartPointer<vtkTransformPolyDataFilter> tf = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  tf->SetTransform (t);
  tf->SetInputConnection (s_sphere->GetOutputPort ());
  tf->Update ();

  return (tf->GetOutput ());
}

