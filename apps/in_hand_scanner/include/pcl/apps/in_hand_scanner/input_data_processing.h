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

#ifndef PCL_APPS_IN_HAND_SCANNER_INPUT_DATA_PROCESSING_H
#define PCL_APPS_IN_HAND_SCANNER_INPUT_DATA_PROCESSING_H

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/common_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <class PointInT, class PointOutT>
  class IntegralImageNormalEstimation;
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InputDataProcessing
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    class PCL_EXPORTS InputDataProcessing
    {
      public:

        typedef pcl::PointXYZRGBA              PointXYZRGBA;
        typedef pcl::PointCloud <PointXYZRGBA> CloudXYZRGBA;
        typedef CloudXYZRGBA::Ptr              CloudXYZRGBAPtr;
        typedef CloudXYZRGBA::ConstPtr         CloudXYZRGBAConstPtr;

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::Normal                            Normal;
        typedef pcl::PointCloud <Normal>               CloudNormals;
        typedef boost::shared_ptr <CloudNormals>       CloudNormalsPtr;
        typedef boost::shared_ptr <const CloudNormals> CloudNormalsConstPtr;

        typedef pcl::IntegralImageNormalEstimation <PointXYZRGBA, Normal> NormalEstimation;
        typedef boost::shared_ptr <NormalEstimation>                      NormalEstimationPtr;
        typedef boost::shared_ptr <const NormalEstimation>                NormalEstimationConstPtr;

      public:

        InputDataProcessing ();

      public:

        bool
        segment (const CloudXYZRGBAConstPtr& cloud_in,
                 CloudXYZRGBNormalPtr&       cloud_out,
                 CloudXYZRGBNormalPtr&       cloud_discarded) const;

        CloudXYZRGBNormalPtr
        calculateNormals (const CloudXYZRGBAConstPtr& cloud) const;

        void
        setCropBox (const float x_min, const float x_max,
                    const float y_min, const float y_max,
                    const float z_min, const float z_max);

        void
        getCropBox (float& x_min, float& x_max,
                    float& y_min, float& y_max,
                    float& z_min, float& z_max) const;

        void
        setColorRange (const float h_min, const float h_max,
                       const float s_min, const float s_max,
                       const float v_min, const float v_max);

        void
        getColorRange (float& h_min, float& h_max,
                       float& s_min, float& s_max,
                       float& v_min, float& v_max) const;

      private:

        void
        threadSegmentation (const PointXYZRGBA*const p_pt_in,
                            const Normal*const       p_n_in,
                            const PointXYZRGBNormal& invalid_pt,
                            const uint32_t           color_discarded,
                            const unsigned int       width,
                            const unsigned int       n_rows,
                            PointXYZRGBNormal*const  p_pt_out,
                            PointXYZRGBNormal*const  p_pt_discarded) const;

        void
        RGBToHSV (const unsigned char r,
                  const unsigned char g,
                  const unsigned char b,
                  float&              h,
                  float&              s,
                  float&              v) const;

        NormalEstimationPtr normal_estimation_;

        float x_min_;
        float x_max_;
        float y_min_;
        float y_max_;
        float z_min_;
        float z_max_;

        float h_min_;
        float h_max_;
        float s_min_;
        float s_max_;
        float v_min_;
        float v_max_;
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_INPUT_DATA_PROCESSING_H
