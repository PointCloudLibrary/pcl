/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

namespace pcl
{
  namespace common
  {
    /** \brief Intensity field accessor provides access to the intensity filed of a PoinT
      * implementation for specific types should be done in \file pcl/common/impl/intensity.hpp
      */
    template<typename PointT>
    struct IntensityFieldAccessor
    {
      /** \brief get intensity field
        * \param[in] p point
        * \return p.intensity
        */
      inline float
      operator () (const PointT &p) const
      {
        return p.intensity;
      }
      /** \brief gets the intensity value of a point
        * \param p point for which intensity to be get
        * \param[in] intensity value of the intensity field
        */
      inline void
      get (const PointT &p, float &intensity) const
      {
        intensity = p.intensity;
      }
      /** \brief sets the intensity value of a point
        * \param p point for which intensity to be set
        * \param[in] intensity value of the intensity field
        */
      inline void
      set (PointT &p, float intensity) const
      {
        p.intensity = intensity;
      }
      /** \brief subtract value from intensity field
        * \param p point for which to modify intensity
        * \param[in] value value to be subtracted from point intensity
        */
      inline void
      demean (PointT& p, float value) const
      {
        p.intensity -= value;
      }
      /** \brief add value to intensity field
        * \param p point for which to modify intensity
        * \param[in] value value to be added to point intensity
        */
      inline void
      add (PointT& p, float value) const
      {
        p.intensity += value;
      }
    };
  }
}

#include <pcl/common/impl/intensity.hpp>
