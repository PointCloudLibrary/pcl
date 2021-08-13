/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * 
 *
 */

#pragma once

#include <vector>

#undef Success
#include <Eigen/StdVector>

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
  namespace on_nurbs
  {

    // http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html
    typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > vector_vec2i;
    typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector_vec2d;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;

    /** \brief Data structure for NURBS surface fitting
     * (FittingSurface, FittingSurfaceTDM, FittingCylinder, GlobalOptimization, GlobalOptimizationTDM) */
    struct NurbsDataSurface
    {
      Eigen::Matrix3d eigenvectors;
      Eigen::Vector3d mean;

      vector_vec3d interior; ///<< input
      std::vector<double> interior_weight; ///<< input
      std::vector<double> interior_error; ///>> output
      vector_vec2d interior_param; ///>> output
      vector_vec3d interior_line_start; ///>> output
      vector_vec3d interior_line_end; ///>> output
      vector_vec3d interior_normals; ///>> output

      vector_vec3d boundary; ///<< input
      std::vector<double> boundary_weight; ///<< input
      std::vector<double> boundary_error; ///>> output
      vector_vec2d boundary_param; ///>> output
      vector_vec3d boundary_line_start; ///>> output
      vector_vec3d boundary_line_end; ///>> output
      vector_vec3d boundary_normals; ///>> output

      vector_vec3d common_boundary_point;
      std::vector<unsigned> common_boundary_idx;
      vector_vec2d common_boundary_param;

      std::vector<unsigned> common_idx;
      vector_vec2d common_param1;
      vector_vec2d common_param2;

      /** \brief Clear all interior data */
      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_weight.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      /** \brief Clear all boundary data */
      inline void
      clear_boundary ()
      {
        boundary.clear ();
        boundary_weight.clear ();
        boundary_error.clear ();
        boundary_param.clear ();
        boundary_line_start.clear ();
        boundary_line_end.clear ();
        boundary_normals.clear ();
      }

      inline void
      clear_common()
      {
        common_idx.clear();
        common_param1.clear();
        common_param2.clear();
      }

      /** \brief Clear all common data */
      inline void
      clear_common_boundary ()
      {
        common_boundary_point.clear ();
        common_boundary_idx.clear ();
        common_boundary_param.clear ();
      }

      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** \brief Data structure for 3D NURBS curve fitting
     * (FittingCurve) */
    struct NurbsDataCurve
    {
      Eigen::Matrix3d eigenvectors;
      Eigen::Vector3d mean;

      vector_vec3d interior; ///<< input
      std::vector<double> interior_error; ///>> output
      std::vector<double> interior_param; ///>> output
      vector_vec3d interior_line_start; ///>> output
      vector_vec3d interior_line_end; ///>> output
      vector_vec3d interior_normals; ///>> output

      /** \brief Clear all interior data */
      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    /** \brief Data structure for 2D NURBS curve fitting
     * (FittingCurve2d, FittingCurve2dTDM, FittingCurve2dSDM) */
    struct NurbsDataCurve2d
    {
      Eigen::Matrix2d eigenvectors;
      Eigen::Vector2d mean;

      vector_vec2d interior; ///<< input
      std::vector<double> interior_error; ///>> output
      std::vector<double> interior_param; ///>> output
      vector_vec2d interior_line_start; ///>> output
      vector_vec2d interior_line_end; ///>> output
      vector_vec2d interior_normals; ///>> output

      std::vector<double> interior_weight;
      std::vector<bool> interior_weight_function;

      vector_vec2d closest_points;
      std::vector<double> closest_points_param;
      std::vector<double> closest_points_error;

      int interior_ncps_prev;
      int closest_ncps_prev;

      vector_vec2d interior_tangents;
      std::vector<double> interior_rho;

      vector_vec2d closest_tangents;
      vector_vec2d closest_normals;
      std::vector<double> closest_rho;

      /** \brief Clear all interior data */
      inline void
      clear_interior ()
      {
        interior.clear ();
        interior_error.clear ();
        interior_param.clear ();
        interior_line_start.clear ();
        interior_line_end.clear ();
        interior_normals.clear ();
      }

      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

  }
}
