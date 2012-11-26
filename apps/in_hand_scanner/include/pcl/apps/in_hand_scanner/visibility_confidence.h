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

#ifndef PCL_IN_HAND_SCANNER_VISIBILITY_CONFIDENCE_H
#define PCL_IN_HAND_SCANNER_VISIBILITY_CONFIDENCE_H

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/eigen.h>

namespace pcl
{
  namespace ihs
  {

    class PCL_EXPORTS VisibilityConfidence
    {
      public:

        // - Frequency 3 Icosahedron where each vertex corresponds to a viewing direction
        // - First vertex aligned to z-axis
        // - Removed vertexes with z < 0.3
        // -> 31 directions, fitting nicely into a 32 bit integer
        // -> Very oblique angles are not considered
        class Dome
        {
          public:

            static const int                                NumDirections = 31;
            typedef Eigen::Matrix <float, 4, NumDirections> Directions;

          public:

            Dome ();

            const Directions&
            directions () const;

          private:

            Directions directions_;
        };

        typedef unsigned int Directions;
        static const int     NumDirections = Dome::NumDirections;

      public:

        VisibilityConfidence ();

        void
        addDirection (const Eigen::Vector4f& normal,
                      const Eigen::Vector4f& direction,
                      const float            weight);

        float
        getValue () const; // [0 1]

      private:

        static const Dome dome_;
        Directions        directions_;
        float             weight_;
        unsigned int      num_weights_;
    };

  } // End namespace ihs
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_VISIBILITY_CONFIDENCE_H
