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
#ifndef PCL_PCL_VISUALIZER_SHAPES_H_
#define PCL_PCL_VISUALIZER_SHAPES_H_

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkUnstructuredGrid.h>
#include <vtkConeSource.h>
#include <vtkDiskSource.h>
#include <vtkPlaneSource.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>

/**
  * \file pcl/visualization/common/shapes.h
  * Define methods or creating 3D shapes from parametric models
  * \ingroup visualization
  */

/*@{*/
namespace pcl
{
  namespace visualization
  {
    /** \brief Create a 3d poly line from a set of points. 
      * \param cloud the set of points used to create the 3d polyline
      * \ingroup visualization
      */
    template <typename PointT> vtkSmartPointer<vtkDataSet> inline 
    createPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

    /** \brief Create a line shape from two points
      * \param pt1 the first point on the line
      * \param pt2 the end point on the line
      * \ingroup visualization
      */
    template <typename P1, typename P2> inline vtkSmartPointer<vtkDataSet> 
    createLine (const P1 &pt1, const P2 &pt2);

    /** \brief Create a sphere shape from a point and a radius
      * \param center the center of the sphere (as an Eigen Vector4f, with only the first 3 coordinates used)
      * \param radius the radius of the sphere
      * \param res (optional) the resolution used for rendering the model
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet>
    createSphere (const Eigen::Vector4f &center, double radius, int res = 10);

    /** \brief Create a cylinder shape from a set of model coefficients.
      * \param coefficients the model coefficients (point_on_axis, axis_direction, radius)
      * \param numsides (optional) the number of sides used for rendering the cylinder
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCylinder (const pcl::ModelCoefficients &coefficients, int numsides = 30);

    /** \brief Create a sphere shape from a set of model coefficients.
      * \param coefficients the model coefficients (sphere center, radius)
      * \param res (optional) the resolution used for rendering the model
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createSphere (const pcl::ModelCoefficients &coefficients, int res = 10);

    /** \brief Create a line shape from a set of model coefficients.
      * \param coefficients the model coefficients (point_on_line, line_direction)
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createLine (const pcl::ModelCoefficients &coefficients);

    /** \brief Create a planar shape from a set of model coefficients.
      * \param coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createPlane (const pcl::ModelCoefficients &coefficients);

    /** \brief Create a 2d circle shape from a set of model coefficients.
      * \param coefficients the model coefficients (x, y, radius)
      * \param z (optional) specify a z value (default: 0)
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    create2DCircle (const pcl::ModelCoefficients &coefficients, double z = 0.0);

    /** \brief Create a cone shape from a set of model coefficients.
      * \param coefficients the cone coefficients (point_on_axis, axis_direction, radius))
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCone (const pcl::ModelCoefficients &coefficients);
  }
}
/*@}*/

#include <pcl/visualization/common/impl/shapes.hpp>

#endif
