/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 */

#ifndef PCL_RECOGNITION_POINT_TYPES
#define PCL_RECOGNITION_POINT_TYPES

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl
{

  /** \brief A point structure for representing RGB color
    * \ingroup common
    */
  //struct EIGEN_ALIGN16 PointRGB
  //{
  //  union
  //  {
  //    union
  //    {
  //      struct
  //      {
  //        uint8_t b;
  //        uint8_t g;
  //        uint8_t r;
  //        uint8_t _unused;
  //      };
  //      float rgb;
  //    };
  //    uint32_t rgba;
  //  };

  //  inline PointRGB ()
  //  {}

  //  inline PointRGB (const uint8_t b, const uint8_t g, const uint8_t r)
  //    : b (b), g (g), r (r), _unused (0)
  //  {}

  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //};


  /** \brief A point structure representing Euclidean xyz coordinates, and the intensity value.
    * \ingroup common
    */
  struct EIGEN_ALIGN16 GradientXY
  {
    union
    {
      struct
      {
        float x;
        float y;
        float angle;
        float magnitude;
      };
      float data[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline bool operator< (const GradientXY & rhs)
    {
      return (magnitude > rhs.magnitude);
    }
  };
  inline std::ostream & operator << (std::ostream & os, const GradientXY & p)
  {
    os << "(" << p.x << "," << p.y << " - " << p.magnitude << ")";
    return (os);
  }

}

#endif
