/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_PCL_VISUALIZER_SHAPES_H_
#define PCL_PCL_VISUALIZER_SHAPES_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/eigen.h>
#include <pcl/geometry/planar_polygon.h>

template <typename T> class vtkSmartPointer;
class vtkDataSet;
class vtkUnstructuredGrid;

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
      * \param[in] cloud the set of points used to create the 3d polyline
      * \ingroup visualization
      */
    template <typename PointT> vtkSmartPointer<vtkDataSet> inline 
    createPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);

    /** \brief Create a 3d poly line from a set of points on the boundary of a planar region. 
      * \param[in] planar_polygon the set of points used to create the 3d polyline
      * \ingroup visualization
      */
    template <typename PointT> vtkSmartPointer<vtkDataSet> inline 
    createPolygon (const pcl::PlanarPolygon<PointT> &planar_polygon);

    /** \brief Create a line shape from two points
      * \param[in] pt1 the first point on the line
      * \param[in] pt2 the end point on the line
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createLine (const Eigen::Vector4f &pt1, const Eigen::Vector4f &pt2);

    /** \brief Create a sphere shape from a point and a radius
      * \param[in] center the center of the sphere (as an Eigen Vector4f, with only the first 3 coordinates used)
      * \param[in] radius the radius of the sphere
      * \param[in] res (optional) the resolution used for rendering the model
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet>
    createSphere (const Eigen::Vector4f &center, double radius, int res = 10);

    /** \brief Create a cylinder shape from a set of model coefficients.
      * \param[in] coefficients the model coefficients (point_on_axis, axis_direction, radius)
      * \param[in] numsides (optional) the number of sides used for rendering the cylinder
      *
      * \code
      * // The following are given (or computed using sample consensus techniques -- see SampleConsensusModelCylinder)
      * // Eigen::Vector3f pt_on_axis, axis_direction;
      * // float radius;
      *
      * pcl::ModelCoefficients cylinder_coeff;
      * cylinder_coeff.values.resize (7);    // We need 7 values
      * cylinder_coeff.values[0] = pt_on_axis.x ();
      * cylinder_coeff.values[1] = pt_on_axis.y ();
      * cylinder_coeff.values[2] = pt_on_axis.z ();
      *
      * cylinder_coeff.values[3] = axis_direction.x ();
      * cylinder_coeff.values[4] = axis_direction.y ();
      * cylinder_coeff.values[5] = axis_direction.z ();
      *
      * cylinder_coeff.values[6] = radius;
      *
      * vtkSmartPointer<vtkDataSet> data = pcl::visualization::createCylinder (cylinder_coeff, numsides);
      * \endcode
      *
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCylinder (const pcl::ModelCoefficients &coefficients, int numsides = 30);

    /** \brief Create a sphere shape from a set of model coefficients.
      * \param[in] coefficients the model coefficients (sphere center, radius)
      * \param[in] res (optional) the resolution used for rendering the model
      *
      * \code
      * // The following are given (or computed using sample consensus techniques -- see SampleConsensusModelSphere)
      * // Eigen::Vector3f sphere_center;
      * // float radius;
      *
      * pcl::ModelCoefficients sphere_coeff;
      * sphere_coeff.values.resize (4);    // We need 4 values
      * sphere_coeff.values[0] = sphere_center.x ();
      * sphere_coeff.values[1] = sphere_center.y ();
      * sphere_coeff.values[2] = sphere_center.z ();
      *
      * sphere_coeff.values[3] = radius;
      *
      * vtkSmartPointer<vtkDataSet> data = pcl::visualization::createSphere (sphere_coeff, resolution);
      * \endcode
      *
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createSphere (const pcl::ModelCoefficients &coefficients, int res = 10);

    /** \brief Create a line shape from a set of model coefficients.
      * \param[in] coefficients the model coefficients (point_on_line, line_direction)
      * 
      * \code
      * // The following are given (or computed using sample consensus techniques -- see SampleConsensusModelLine)
      * // Eigen::Vector3f point_on_line, line_direction;
      *
      * pcl::ModelCoefficients line_coeff;
      * line_coeff.values.resize (6);    // We need 6 values
      * line_coeff.values[0] = point_on_line.x ();
      * line_coeff.values[1] = point_on_line.y ();
      * line_coeff.values[2] = point_on_line.z ();
      *
      * line_coeff.values[3] = line_direction.x ();
      * line_coeff.values[4] = line_direction.y ();
      * line_coeff.values[5] = line_direction.z ();
      *
      * vtkSmartPointer<vtkDataSet> data = pcl::visualization::createLine (line_coeff);
      * \endcode
      *
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createLine (const pcl::ModelCoefficients &coefficients);

    /** \brief Create a planar shape from a set of model coefficients.
      * \param[in] coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
      *
      * \code
      * // The following are given (or computed using sample consensus techniques -- see SampleConsensusModelPlane)
      * // Eigen::Vector4f plane_parameters;
      *
      * pcl::ModelCoefficients plane_coeff;
      * plane_coeff.values.resize (4);    // We need 4 values
      * plane_coeff.values[0] = plane_parameters.x ();
      * plane_coeff.values[1] = plane_parameters.y ();
      * plane_coeff.values[2] = plane_parameters.z ();
      * plane_coeff.values[3] = plane_parameters.w ();
      *
      * vtkSmartPointer<vtkDataSet> data = pcl::visualization::createPlane (plane_coeff);
      * \endcode
      *
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createPlane (const pcl::ModelCoefficients &coefficients);

    /** \brief Create a planar shape from a set of model coefficients.
      * \param[in] coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
      * \param[in] x,y,z projection of this point on the plane is used to get the center of the plane.
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z);
    
    /** \brief Create a 2d circle shape from a set of model coefficients.
      * \param[in] coefficients the model coefficients (x, y, radius)
      * \param[in] z (optional) specify a z value (default: 0)
      *
      * \code
      * // The following are given (or computed using sample consensus techniques -- see SampleConsensusModelCircle2D)
      * // float x, y, radius;
      *
      * pcl::ModelCoefficients circle_coeff;
      * circle_coeff.values.resize (3);    // We need 3 values
      * circle_coeff.values[0] = x;
      * circle_coeff.values[1] = y;
      * circle_coeff.values[2] = radius;
      *
      * vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff, z);
      * \endcode
      *
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    create2DCircle (const pcl::ModelCoefficients &coefficients, double z = 0.0);

    /** \brief Create a cone shape from a set of model coefficients.
      * \param[in] coefficients the cone coefficients (point_on_axis, axis_direction, radius)
      * \ingroup visualization
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCone (const pcl::ModelCoefficients &coefficients);

    /** \brief Creaet a cube shape from a set of model coefficients.
      * \param[in] coefficients the cube coefficients (Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
      * \ingroup visualization 
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCube (const pcl::ModelCoefficients &coefficients);

    /** \brief Creaet a cube shape from a set of model coefficients.
      *
      * \param[in] translation a translation to apply to the cube from 0,0,0
      * \param[in] rotation a quaternion-based rotation to apply to the cube 
      * \param[in] width the cube's width
      * \param[in] height the cube's height
      * \param[in] depth the cube's depth
      * \ingroup visualization 
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
                double width, double height, double depth);
    
    /** \brief Create a cube from a set of bounding points
      * \param[in] x_min is the minimum x value of the box
      * \param[in] x_max is the maximum x value of the box
      * \param[in] y_min is the minimum y value of the box 
      * \param[in] y_max is the maximum y value of the box
      * \param[in] z_min is the minimum z value of the box
      * \param[in] z_max is the maximum z value of the box
      */
    PCL_EXPORTS vtkSmartPointer<vtkDataSet> 
    createCube (double x_min, double x_max,
                double y_min, double y_max,
                double z_min, double z_max);
    
    /** \brief Allocate a new unstructured grid smartpointer. For internal use only.
      * \param[out] polydata the resultant unstructured grid. 
      */
    PCL_EXPORTS void
    allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata);
  }
}
/*@}*/

#include <pcl/visualization/common/impl/shapes.hpp>

#endif
