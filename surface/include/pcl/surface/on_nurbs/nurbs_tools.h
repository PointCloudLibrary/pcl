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

#ifndef NURBS_TOOLS_H
#define NURBS_TOOLS_H

#include <pcl/surface/on_nurbs/nurbs_data.h>

#include <pcl/surface/3rdparty/opennurbs/opennurbs.h>

#undef Success
#include <Eigen/Dense>

namespace pcl
{
  namespace on_nurbs
  {

    enum
    {
      NORTH = 1, NORTHEAST = 2, EAST = 3, SOUTHEAST = 4, SOUTH = 5, SOUTHWEST = 6, WEST = 7, NORTHWEST = 8
    };

    /** \brief Some useful tools for initialization, point search, ... */
    class NurbsTools
    {
    public:

      //      static std::list<unsigned>
      //      getClosestPoints (const Eigen::Vector2d &p, const vector_vec2d &data, unsigned s);

      /** \brief Get the closest point with respect to 'point'
       *  \param[in] point The point to which the closest point is searched for.
       *  \param[in] data Vector containing the set of points for searching. */
      static unsigned
      getClosestPoint (const Eigen::Vector2d &point, const vector_vec2d &data);

      /** \brief Get the closest point with respect to 'point'
       *  \param[in] point The point to which the closest point is searched for.
       *  \param[in] data Vector containing the set of points for searching. */
      static unsigned
      getClosestPoint (const Eigen::Vector3d &point, const vector_vec3d &data);

      /** \brief Get the closest point with respect to 'point' in Non-Euclidean metric
       *  \brief Related paper: TODO
       *  \param[in] point The point to which the closest point is searched for.
       *  \param[in] dir The direction defining 'inside' and 'outside'
       *  \param[in] data Vector containing the set of points for searching.
       *  \param[out] idxcp Closest point with respect to Euclidean metric. */
      static unsigned
      getClosestPoint (const Eigen::Vector2d &point, const Eigen::Vector2d &dir, const vector_vec2d &data,
                       unsigned &idxcp);

      /** \brief Compute the mean of a set of points
       *  \param[in] data Set of points.     */
      static Eigen::Vector3d
      computeMean (const vector_vec3d &data);
      /** \brief Compute the mean of a set of points
       *  \param[in] data Set of points.     */
      static Eigen::Vector2d
      computeMean (const vector_vec2d &data);

      /** \brief Compute the variance of a set of points
       *  \param[in] data Set of points       */
      static Eigen::Vector3d
      computeVariance (const Eigen::Vector3d &mean, const vector_vec3d &data);
      /** \brief Compute the variance of a set of points
       *  \param[in] data Set of points       */
      static Eigen::Vector2d
      computeVariance (const Eigen::Vector2d &mean, const vector_vec2d &data);

      /** compute bounding box of curve control points */
      static void
      computeBoundingBox (const ON_NurbsCurve &nurbs, Eigen::Vector3d &_min, Eigen::Vector3d &_max);
      static void
      computeBoundingBox (const ON_NurbsSurface &nurbs, Eigen::Vector3d &_min, Eigen::Vector3d &_max);

      static double
      computeRScale (const Eigen::Vector3d &_min, const Eigen::Vector3d &_max);

      /** \brief PCA - principal-component-analysis
       *  \param[in] data Set of points.
       *  \param[out] mean The mean of the set of points.
       *  \param[out] eigenvectors Matrix containing column-wise the eigenvectors of the set of points.
       *  \param[out] eigenvalues The eigenvalues of the set of points with respect to the eigenvectors. */
      static void
      pca (const vector_vec3d &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors,
           Eigen::Vector3d &eigenvalues);

      /** \brief PCA - principal-component-analysis
       *  \param[in] data Set of points.
       *  \param[out] mean The mean of the set of points.
       *  \param[out] eigenvectors Matrix containing column-wise the eigenvectors of the set of points.
       *  \param[out] eigenvalues The eigenvalues of the set of points with respect to the eigenvectors. */
      static void
      pca (const vector_vec2d &data, Eigen::Vector2d &mean, Eigen::Matrix2d &eigenvectors,
           Eigen::Vector2d &eigenvalues);

      /** \brief Downsample data points to a certain size.
       *  \param[in] data1 The original set of points.
       *  \param[out] data2 The downsampled set of points of size 'size'.
       *  \param[in] size The desired size of the resulting set of points.       */
      static void
      downsample_random (const vector_vec3d &data1, vector_vec3d &data2, unsigned size);
      /** \brief Downsample data points to a certain size.
       *  \param[in/out] data1 The set of points for downsampling;
       *  will be replaced by the resulting set of points of size 'size'.
       *  \param[in] size The desired size of the resulting set of points.       */
      static void
      downsample_random (vector_vec3d &data1, unsigned size);

    };

  }
}

#endif /* NTOOLS_H_ */
