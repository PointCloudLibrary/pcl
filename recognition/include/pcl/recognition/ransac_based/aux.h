/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#ifndef PCL_RECOGNITION_RANSAC_BASED_AUX_H_
#define PCL_RECOGNITION_RANSAC_BASED_AUX_H_

#include <cmath>

namespace pcl
{
  namespace recognition
  {
    /** \brief c = a - b */
    template <typename T> void
    vecDiff3 (const T a[3], const T b[3], T c[3])
    {
      c[0] = a[0] - b[0];
      c[1] = a[1] - b[1];
      c[2] = a[2] - b[2];
    }

    /** \brief Returns the length of v. */
    template <typename T> T
    vecLength3 (const T v[3])
    {
      return (static_cast<T> (sqrt (v[0]*v[0] + v[1]*v[1] + v[2]*v[2])));
    }

    /** \brief Returns the Euclidean distance between a and b. */
    template <typename T> T
    vecDistance3 (const T a[3], const T b[3])
    {
      T l[3] = {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
      return (vecLength3 (l));
    }

    /** \brief Returns the dot product a*b */
    template <typename T> T
    vecDot3 (const T a[3], const T b[3])
    {
      return (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]);
    }

    /** \brief v = scalar*v. */
    template <typename T> void
    vecMult3 (T v[3], T scalar)
    {
      v[0] *= scalar;
      v[1] *= scalar;
      v[2] *= scalar;
    }

    /** \brief Normalize v */
    template <typename T> void
    vecNormalize3 (T v[3])
    {
      T inv_len = (static_cast<T> (1.0))/vecLength3 (v);
      v[0] *= inv_len;
      v[1] *= inv_len;
      v[2] *= inv_len;
    }

    /** \brief Returns the square length of v. */
    template <typename T> T
    vecSqrLength3 (const T v[3])
    {
      return (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    }
  }
}

#endif // AUX_H_
