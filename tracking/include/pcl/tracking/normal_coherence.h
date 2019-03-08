/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#pragma once

#include <pcl/tracking/coherence.h>
namespace pcl
{
  namespace tracking
  {
    /** \brief @b NormalCoherence computes coherence between two points from the angle
        between their normals. the coherence is calculated by 1 / (1 + weight * theta^2 ).
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class NormalCoherence: public PointCoherence<PointInT>
    {
    public:

      /** \brief initialize the weight to 1.0. */
      NormalCoherence ()
      : PointCoherence<PointInT> ()
      , weight_ (1.0)
        {}

      /** \brief set the weight of coherence
        * \param weight the weight of coherence
        */
      inline void setWeight (double weight) { weight_ = weight; }

      /** \brief get the weight of coherence */
      inline double getWeight () { return weight_; }

    protected:

      /** \brief return the normal coherence between the two points.
        * \param source instance of source point.
        * \param target instance of target point.
        */
      double computeCoherence (PointInT &source, PointInT &target) override;

      /** \brief the weight of coherence */
      double weight_;
      
    };
  }
}

// #include <pcl/tracking/impl/normal_coherence.hpp>
#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/normal_coherence.hpp>
#endif
