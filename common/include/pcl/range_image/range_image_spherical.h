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
 */

#pragma once

#include <pcl/range_image/range_image.h>

namespace pcl
{
  /** \brief @b RangeImageSpherical is derived from the original range image and uses a slightly different
    * spherical projection. In the original range image, the image will appear more and more
    * "scaled down" along the y axis, the further away from the mean line of the image a point is.
    * This class removes this scaling, which makes it especially suitable for spinning LIDAR sensors
    * that capure a 360° view, since a rotation of the sensor will now simply correspond to a shift of the
    * range image. (This class is similar to RangeImagePlanar, but changes less of the behaviour of the base class.)
    * \author Andreas Muetzel
    * \ingroup range_image
    */
  class RangeImageSpherical : public RangeImage
  {
    public:
      // =====TYPEDEFS=====
      using BaseClass = RangeImage;
      using Ptr = shared_ptr<RangeImageSpherical>;
      using ConstPtr = shared_ptr<const RangeImageSpherical>;

      // =====CONSTRUCTOR & DESTRUCTOR=====
      /** Constructor */
      PCL_EXPORTS RangeImageSpherical () {}
      /** Destructor */
      PCL_EXPORTS virtual ~RangeImageSpherical () {}

      /** Return a newly created RangeImagePlanar.
       *  Reimplementation to return an image of the same type. */
      virtual RangeImage*
      getNew () const { return new RangeImageSpherical; }

      // =====PUBLIC METHODS=====
      /** \brief Get a boost shared pointer of a copy of this */
      inline Ptr
      makeShared () { return Ptr (new RangeImageSpherical (*this)); }


      // Since we reimplement some of these overloaded functions, we have to do the following:
      using RangeImage::calculate3DPoint;
      using RangeImage::getImagePoint;

      /** \brief Calculate the 3D point according to the given image point and range
        * \param image_x the x image position
        * \param image_y the y image position
        * \param range the range
        * \param point the resulting 3D point
        * \note Implementation according to planar range images (compared to spherical as in the original)
        */
      virtual inline void
      calculate3DPoint (float image_x, float image_y, float range, Eigen::Vector3f& point) const;

      /** \brief Calculate the image point and range from the given 3D point
        * \param point the resulting 3D point
        * \param image_x the resulting x image position
        * \param image_y the resulting y image position
        * \param range the resulting range
        * \note Implementation according to planar range images (compared to spherical as in the original)
        */
      virtual inline void
      getImagePoint (const Eigen::Vector3f& point, float& image_x, float& image_y, float& range) const;

      /** Get the angles corresponding to the given image point */
      inline void
      getAnglesFromImagePoint (float image_x, float image_y, float& angle_x, float& angle_y) const;

      /** Get the image point corresponding to the given ranges */
      inline void
      getImagePointFromAngles (float angle_x, float angle_y, float& image_x, float& image_y) const;

  };
}  // namespace end


#include <pcl/range_image/impl/range_image_spherical.hpp>  // Definitions of templated and inline functions
