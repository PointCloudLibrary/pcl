/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_APPS_IN_HAND_SCANNER_VISIBILITY_CONFIDENCE_H
#define PCL_APPS_IN_HAND_SCANNER_VISIBILITY_CONFIDENCE_H

#include <stdint.h>

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/eigen.h>

namespace pcl
{
  namespace ihs
  {
    // - Frequency 3 Icosahedron where each vertex corresponds to a viewing direction
    // - First vertex aligned to z-axis
    // - Removed vertices with z < 0.3
    // -> 31 directions, fitting nicely into a 32 bit integer
    // -> Very oblique angles are not considered
    class PCL_EXPORTS Dome
    {
      public:

        static const int num_directions = 31;
        typedef Eigen::Matrix <float, 4, num_directions> Vertices;

        Dome ();

        Vertices
        getVertices () const;

      private:

        Vertices vertices_;

      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    PCL_EXPORTS void
    addDirection (const Eigen::Vector4f& normal,
                  const Eigen::Vector4f& direction,
                  uint32_t&              directions);

    PCL_EXPORTS unsigned int
    countDirections (const uint32_t directions);

  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_VISIBILITY_CONFIDENCE_H
