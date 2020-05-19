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
 * $Id$
 *
 */

#pragma once

#include <pcl/keypoints/keypoint.h>
#include <pcl/common/intensity.h>

namespace pcl
{
  /** \brief HarrisKeypoint2D detects Harris corners family points
    *
    * \author Nizar Sallem
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
  class HarrisKeypoint2D : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<HarrisKeypoint2D<PointInT, PointOutT, IntensityT> >;
      using ConstPtr = shared_ptr<const HarrisKeypoint2D<PointInT, PointOutT, IntensityT> >;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;
      using KdTree = typename Keypoint<PointInT, PointOutT>::KdTree;
      using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      enum ResponseMethod {HARRIS = 1, NOBLE, LOWE, TOMASI};

      /** \brief Constructor
        * \param[in] method the method to be used to determine the corner responses
        * \param window_width
        * \param window_height
        * \param min_distance
        * \param[in] threshold the threshold to filter out weak corners
        */
      HarrisKeypoint2D (ResponseMethod method = HARRIS, int window_width = 3, int window_height = 3, int min_distance = 5, float threshold = 0.0)
      : threshold_ (threshold)
      , refine_ (false)
      , nonmax_ (true)
      , method_ (method)
      , threads_ (0)
      , response_ (new pcl::PointCloud<PointOutT> ())
      , window_width_ (window_width)
      , window_height_ (window_height)
      , skipped_pixels_ (0)
      , min_distance_ (min_distance)
      {
        name_ = "HarrisKeypoint2D";
      }

      /** \brief set the method of the response to be calculated.
        * \param[in] type
        */
      void setMethod (ResponseMethod type);

      ///Set window width
      void setWindowWidth (int window_width);

      ///Set window height
      void setWindowHeight (int window_height);      

      ///Set number of pixels to skip
      void setSkippedPixels (int skipped_pixels);

      ///Set minimal distance between candidate keypoints
      void setMinimalDistance (int min_distance);
      
      /** \brief set the threshold value for detecting corners. This is only evaluated if non maxima suppression is turned on.
        * \brief note non maxima suppression needs to be activated in order to use this feature.
        * \param[in] threshold 
        */
      void setThreshold (float threshold);

      /** \brief whether non maxima suppression should be applied or the response for each point should be returned
        * \note this value needs to be turned on in order to apply thresholding and refinement
        * \param[in] nonmax default is false
        */
      void setNonMaxSupression (bool = false);

      /** \brief whether the detected key points should be refined or not. If turned of, the key points are a subset of 
        * the original point cloud. Otherwise the key points may be arbitrary.
        * \brief note non maxima supression needs to be on in order to use this feature.
        * \param[in] do_refine
        */
      void setRefine (bool do_refine);

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

    protected:
      bool 
      initCompute () override;
      void 
      detectKeypoints (PointCloudOut &output) override;
      /** \brief gets the corner response for valid input points*/
      void 
      responseHarris (PointCloudOut &output) const;
      void 
      responseNoble (PointCloudOut &output) const;
      void 
      responseLowe (PointCloudOut &output) const;
      void 
      responseTomasi (PointCloudOut &output) const;
//      void refineCorners (PointCloudOut &corners) const;
      /** \brief calculates the upper triangular part of unnormalized 
        * covariance matrix over intensities given by the 2D coordinates 
        * and window_width_ and window_height_
        */
      void 
      computeSecondMomentMatrix (std::size_t pos, float* coefficients) const;
      /// threshold for non maxima suppression 
      float threshold_;
      /// corner refinement 
      bool refine_;
      /// non maximas suppression
      bool nonmax_;
      /// cornerness computation method
      ResponseMethod method_;
      /// number of threads to be used
      unsigned int threads_;      

    private:
      Eigen::MatrixXf derivatives_rows_;
      Eigen::MatrixXf derivatives_cols_;
      /// intermediate holder for computed responses
      typename pcl::PointCloud<PointOutT>::Ptr response_;
      /// comparator for responses intensity
      bool 
      greaterIntensityAtIndices (int a, int b) const
      {
        return (response_->at (a).intensity > response_->at (b).intensity);
      }      
      /// Window width 
      int window_width_;
      /// Window height
      int window_height_;
      /// half window width
      int half_window_width_;
      /// half window height
      int half_window_height_;
      /// number of pixels to skip within search window
      int skipped_pixels_;
      /// minimum distance between two keypoints
      int min_distance_;
      /// intensity field accessor
      IntensityT intensity_;
  };
}

#include <pcl/keypoints/impl/harris_2d.hpp>
