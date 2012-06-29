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
 * $Id$
 *
 */

#ifndef EDGE_H
#define EDGE_H

#include <vector>
#include "Convolution.h"
#include "Kernel.h"
#include <pcl/pcl_base.h>
namespace pcl
{
  namespace pcl_2d
  {
    template<typename PointT>
    class Edge{
      private:
        Convolution<PointT> *convolution;

        void traceEdge (int rowOffset, int colOffset, int row, int col, float theta, float tLow, float tHigh, PointCloud<PointT> &G, PointCloud<PointT> &thet);

      public:
        enum OUTPUT_TYPE
        {
          OUTPUT_Y,
          OUTPUT_X,
          OUTPUT_X_Y,
          OUTPUT_MAGNITUDE,
          OUTPUT_DIRECTION,
          OUTPUT_MAGNITUDE_DIRECTION,
          OUTPUT_ALL
        };

        enum DETECTOR_KERNEL_TYPE
        {
          CANNY,
          SOBEL,
          PREWITT,
          ROBERTS,
          LOG,
          DERIVATIVE_CENTRAL,
          DERIVATIVE_FORWARD,
          DERIVATIVE_BACKWARD
        };

        OUTPUT_TYPE output_type_;
        DETECTOR_KERNEL_TYPE detector_kernel_type_;
        bool non_maximal_suppression_;
        bool hysteresis_thresholding_;

        int hysteresis_threshold_low_;
        int hysteresis_threshold_high_;
        int non_max_suppression_radius_x_;
        int non_max_suppression_radius_y_;
        PointCloud<PointT> detector_kernel_;

        Edge () :
          output_type_ (OUTPUT_X),
          detector_kernel_type_ (SOBEL),
          non_maximal_suppression_ (false),
          hysteresis_thresholding_ (false),
          hysteresis_threshold_low_ (20),
          hysteresis_threshold_high_ (80),
          non_max_suppression_radius_x_ (3),
          non_max_suppression_radius_y_ (3)
        {
          convolution = new Convolution<PointT> ();
        }

        void detectEdge (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void detectEdgeCanny (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void detectEdgeSobel (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void detectEdgePrewitt (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void detectEdgeRoberts (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void detectEdgeLoG (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void computeDerivativeXCentral (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void computeDerivativeYCentral (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void computeDerivativeXForward (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void computeDerivativeYForward (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void computeDerivativeXBackward (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void computeDerivativeYBackward (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void nonMaximalSuppression (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void hysteresisThresholding (PointCloud<PointT> &output, PointCloud<PointT> &input);

    };
  }
}
#endif
