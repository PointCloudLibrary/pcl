/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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
 * @author thomas.moerwald
 *
 */

#ifndef NURBS_TOOLS_H
#define NURBS_TOOLS_H

#include <pcl/surface/on_nurbs/nurbs_data.h>

#include <pcl/surface/openNURBS/opennurbs.h>

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

    class NurbsTools
    {
    public:

      static std::list<unsigned>
      getClosestPoints (const Eigen::Vector2d &p, const vector_vec2d &data, unsigned s);
      static unsigned
      getClosestPoint (const Eigen::Vector2d &point, const vector_vec2d &data);
      static unsigned
      getClosestPoint (const Eigen::Vector3d &point, const vector_vec3d &data);
      static unsigned
      getClosestPoint (const Eigen::Vector2d &p, const Eigen::Vector2d &dir, const vector_vec2d &data, unsigned &idxcp);

      static Eigen::Vector3d
      computeMean (const vector_vec3d &data);
      static Eigen::Vector2d
      computeMean (const vector_vec2d &data);

      static void
                  pca (const vector_vec3d &data, Eigen::Vector3d &mean, Eigen::Matrix3d &eigenvectors,
                       Eigen::Vector3d &eigenvalues);

      static void
      downsample_random (const vector_vec3d &data1, vector_vec3d &data2, unsigned size);
      static void
      downsample_random (vector_vec3d &data1, unsigned size);

    };

  }
}

#endif /* NTOOLS_H_ */
