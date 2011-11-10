/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *  @author Suat Gedikli
 */

#ifndef PCL_HARRIS_KEYPOINT_3D_H_
#define PCL_HARRIS_KEYPOINT_3D_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  template <typename PointInT, typename PointOutT>
  class HarrisKeypoint3D : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, PointOutT>::KdTree KdTree;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::surface_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::k_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::search_parameter_;
 

      typedef enum {HARRIS = 1, NOBLE, LOWE, TOMASI, CURVATURE} ResponseMethod;
      
      /**
       * @brief Constructor 
       * @param method the method to be used to determine the corner responses
       * @param radius the radius for normal estimation as well as for non maxima suppression
       * @param threshold the threshold to filter out weak corners
       */
      HarrisKeypoint3D (ResponseMethod method = HARRIS, float radius = 0.01, float threshold = 0.0) 
      : radius_ (radius)
      , threshold_ (threshold)
      , refine_ (true)
      , nonmax_ (true)
      , method_ (method)
      {
        name_ = "HarrisKeypoint3D";
      }

      /**
       * @brief set the method of the response to be calculated.
       * @param type
       */
      void setMethod (ResponseMethod type);
      
      /**
       * @brief set the radius for normal estimation and non maxima supression.
       * @param radius
       */
      void setRadius (float radius);
      
      /**
       * @brief set the threshold value for detecting corners. This is only evaluated if non maxima suppression is turned on.
       * @brief note non maxima supression needs to be on in order to use this feature.
       * @param threshold
       */
      void setThreshold (float threshold);

      /**
       * @brief whether non maxima suppression should be applied or the response for each point should be returned
       * @note this value needs to be turned on in order to apply thresholding and refinement
       * @param nonmax default is false
       */
      void setNonMaxSupression (bool = false);
      
      /**
       * @brief whether the detected key points should be refined or not. If turned of, the key points are a subset of the original point cloud. Otherwise the key points may be arbitrary.
       * @brief note non maxima supression needs to be on in order to use this feature.
       * @param do_refine
       */
      void setRefine (bool do_refine);

    protected:
      void detectKeypoints (PointCloudOut &output);
      void responseHarris (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals,  PointCloudOut &output) const;
      void responseNoble (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const;
      void responseLowe (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const;
      void responseTomasi (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const;
      void responseCurvature (typename PointCloudIn::ConstPtr input, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &output) const;
      void refineCorners (typename PointCloudIn::ConstPtr surface, pcl::PointCloud<Normal>::ConstPtr normals, PointCloudOut &corners) const;
    private:
      float radius_;
      float threshold_;
      bool refine_;
      bool nonmax_;
      ResponseMethod method_;
  };
}

#include "pcl/keypoints/impl/harris_keypoint3D.hpp"

#endif // #ifndef PCL_HARRIS_KEYPOINT_3D_H_

