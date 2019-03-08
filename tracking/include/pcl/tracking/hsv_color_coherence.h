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
    /** \brief @b HSVColorCoherence computes coherence between the two points from
        the color difference between them. the color difference is calculated in HSV color space.
        the coherence is calculated by 1 / ( 1 + w * (w_h^2 * h_diff^2 + w_s^2 * s_diff^2 + w_v^2 * v_diff^2))
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT>
    class HSVColorCoherence: public PointCoherence<PointInT>
    {
      public:

        /** \brief initialize the weights of the computation.
            weight_, h_weight_, s_weight_ default to 1.0 and
            v_weight_ defaults to 0.0.
         */
        HSVColorCoherence ()
        : PointCoherence<PointInT> ()
        , weight_ (1.0)
        , h_weight_ (1.0)
        , s_weight_ (1.0)
        , v_weight_ (0.0)
        {}

        /** \brief set the weight of coherence
          * \param[in] weight the weight of coherence.
          */
        inline void 
        setWeight (double weight) { weight_ = weight; }

        /** \brief get the weight (w) of coherence */
        inline double 
        getWeight () { return weight_; }
        
        /** \brief set the hue weight (w_h) of coherence
          * \param[in] weight the hue weight (w_h) of coherence.
          */
        inline void 
        setHWeight (double weight) { h_weight_ = weight; }

        /** \brief get the hue weight (w_h) of coherence */
        inline double 
        getHWeight () { return h_weight_; }

        /** \brief set the saturation weight (w_s) of coherence
          * \param[in] weight the saturation weight (w_s) of coherence.
          */
        inline void 
        setSWeight (double weight) { s_weight_ = weight; }

        /** \brief get the saturation weight (w_s) of coherence */
        inline double 
        getSWeight () { return s_weight_; }

        /** \brief set the value weight (w_v) of coherence
          * \param[in] weight the value weight (w_v) of coherence.
          */
        inline void 
        setVWeight (double weight) { v_weight_ = weight; }

        /** \brief get the value weight (w_v) of coherence */
        inline double 
        getVWeight () { return v_weight_; }
        
      protected:
        
        /** \brief return the color coherence between the two points.
          * \param[in] source instance of source point.
          * \param[in] target instance of target point.
          */
        double 
        computeCoherence (PointInT &source, PointInT &target) override;

        /** \brief the weight of coherence (w) */
        double weight_;

        /** \brief the hue weight (w_h) */
        double h_weight_;
        
        /** \brief the saturation weight (w_s) */
        double s_weight_;

        /** \brief the value weight (w_v) */
        double v_weight_;
        
      };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/hsv_color_coherence.hpp>
#endif
