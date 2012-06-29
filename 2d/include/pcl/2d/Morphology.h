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
 */

#ifndef MORPHOLOGY_H_
#define MORPHOLOGY_H_

#include <pcl/pcl_base.h>
#include "Convolution.h"
namespace pcl
{
  namespace pcl_2d
  {
    template<typename PointT>
    class Morphology
    {
      private:
      public:
        enum MORPHOLOGICAL_OPERATOR_TYPE
        {
          EROSION_GRAY,
          DILATION_GRAY,
          OPENING_GRAY,
          CLOSING_GRAY,
          EROSION_BINARY,
          DILATION_BINARY,
          OPENING_BINARY,
          CLOSING_BINARY
        };

        MORPHOLOGICAL_OPERATOR_TYPE operator_type_;
        PointCloud<PointT> structuring_element_;

        Morphology ()
        {
        }

        void applyMorphologicalOperation (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void openingBinary (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void closingBinary (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void erosionBinary (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void dilationBinary (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void subtractionBinary (PointCloud<PointT> &output, PointCloud<PointT> &input1, PointCloud<PointT> &input2);

        void unionBinary (PointCloud<PointT> &output, PointCloud<PointT> &input1, PointCloud<PointT> &input2);

        void intersectionBinary (PointCloud<PointT> &output, PointCloud<PointT> &input1, PointCloud<PointT> &input2);

        void openingGray (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void closingGray (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void erosionGray (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void dilationGray (PointCloud<PointT> &output, PointCloud<PointT> &input);

        void structuringElementCircular (PointCloud<PointT> &kernel, const int radius);

        void structuringElementRectangle (PointCloud<PointT> &kernel, const int height, const int width);
    };
  }
}
#endif /* MORPHOLOGY_H_ */
