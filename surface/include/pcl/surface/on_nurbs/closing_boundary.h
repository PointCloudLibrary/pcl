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

#include <pcl/surface/on_nurbs/nurbs_data.h>
#include <pcl/surface/on_nurbs/nurbs_tools.h>
#include <pcl/surface/on_nurbs/fitting_surface_pdm.h>

namespace pcl
{
  namespace on_nurbs
  {

    /** \brief Functions for finding the common boundary of adjacent NURBS patches
     * \author Thomas Mörwald
     * \ingroup surface     */
    class ClosingBoundary
    {
    public:
      enum Type
      {
        COMMON_BOUNDARY_POINT_MEAN, COMMON_BOUNDARY_POINT_TANGENTS, COMMON_BOUNDARY_POINT_PLANES,
        CLOSEST_POINTS_BOUNDARY, CLOSEST_POINTS_INTERIOR,
      };

      struct Parameter
      {
        double max_dist;
        double max_error;
        unsigned samples;
        unsigned com_iter;
        unsigned fit_iter;
        double accuracy;
        double smoothness;
        double boundary_weight;
        double interior_weight;
        Type type;

        Parameter (double _max_dist = 0.02, double _max_error = 0.02, unsigned _samples = 10, unsigned _iter = 10,
                   unsigned _fit_iter = 10, double _accuracy = 1e-3, double _smooth = 0.00001,
                   double _bnd_weight = 1.0, double _int_weight = 1.0, Type _type = COMMON_BOUNDARY_POINT_MEAN) :
          max_dist (_max_dist), max_error (_max_error), samples (_samples), com_iter (_iter), fit_iter (_fit_iter),
              accuracy (_accuracy), smoothness (_smooth), boundary_weight (_bnd_weight),
              interior_weight (_int_weight), type (_type)
        {
        }

      };

      /** \brief calculate the distance and end-points (P,Q) of
       * two skew lines defined as L0 = P0 + t*u, L1 = Q0 + s*v */
      static double
      getLineDistance (const Eigen::Vector3d &P0, const Eigen::Vector3d &u, const Eigen::Vector3d &Q0,
                       const Eigen::Vector3d &v, Eigen::Vector3d &P, Eigen::Vector3d &Q);

      /** \brief calculate intersection point of three planes */
      static Eigen::Vector3d
      intersectPlanes (const Eigen::Vector3d &N1, double d1, const Eigen::Vector3d &N2, double d2,
                       const Eigen::Vector3d &N3, double d3);

      /** \brief calculate common boundary by iteratively calculating the mean of the closest points to the
       * 'start' point on both of the NURBS.   */
      static Eigen::Vector3d
      commonBoundaryPoint1 (ON_NurbsSurface &n1, ON_NurbsSurface &n2, Eigen::Vector2d &params1,
                            Eigen::Vector2d &params2, const Eigen::Vector3d &start, unsigned nsteps, double &error,
                            double accuracy);

      /** \brief calculate common boundary by iteratively calculating the intersections of the tangent of the closest points
       *  to the 'start' point.   */
      static Eigen::Vector3d
      commonBoundaryPoint2 (ON_NurbsSurface &n1, ON_NurbsSurface &n2, Eigen::Vector2d &params1,
                            Eigen::Vector2d &params2, const Eigen::Vector3d &start, unsigned nsteps, double &error,
                            double accuracy);

      /** \brief calculate common boundary by iteratively calculating the intersection of the tangent planes at the closest points
       *  and the plane defined by the 'start' and the 2 closest points.  */
      static Eigen::Vector3d
      commonBoundaryPoint3 (ON_NurbsSurface &n1, ON_NurbsSurface &n2, Eigen::Vector2d &params1,
                            Eigen::Vector2d &params2, const Eigen::Vector3d &start, unsigned nsteps, double &error,
                            double accuracy);

      /** \brief sample points from nurbs surface patch, uniform distributed */
      static void
      sampleUniform (ON_NurbsSurface *nurbs, vector_vec3d &point_list, unsigned samples);

      /** \brief sample points from nurbs surface patch, random distributed */
      static void
      sampleRandom (ON_NurbsSurface *nurbs, vector_vec3d &point_list, unsigned samples);

      /** \brief sample points from nurbs surface boundary, uniform distributed */
      static void
      sampleFromBoundary (ON_NurbsSurface *nurbs, vector_vec3d &point_list, vector_vec2d &param_list, unsigned samples);

      /** \brief close boundary sequentially (for global optimization, look up global_optimization.h).
       *  Common boundary points are computed and added to  */
      static void
      optimizeBoundary (std::vector<ON_NurbsSurface> &nurbs_list, std::vector<NurbsDataSurface> &data_list,
                        Parameter param);

//      static void
//      optimizeControlPoints (std::vector<ON_NurbsSurface> &nurbs_list, std::vector<NurbsDataSurface> &data_list,
//                             Parameter param);

    };

  }
}
