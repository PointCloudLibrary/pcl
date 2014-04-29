/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2013-, Open Perception Inc.
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

#ifndef PCL_TRAJKOVIC_KEYPOINT_2D_H_
#define PCL_TRAJKOVIC_KEYPOINT_2D_H_

#include <pcl/keypoints/keypoint.h>
#include <pcl/common/intensity.h>

namespace pcl
{
  /** \brief TrajkovicKeypoint2D implements Trajkovic and Hedley corner detector on
    * organized pooint cloud using intensity information.
    * It uses first order statistics to find variation of intensities in horizontal
    * or vertical directions.
    *
    * \author Nizar Sallem
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
  class TrajkovicKeypoint2D : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<TrajkovicKeypoint2D<PointInT, PointOutT, IntensityT> > Ptr;
      typedef boost::shared_ptr<const TrajkovicKeypoint2D<PointInT, PointOutT, IntensityT> > ConstPtr;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      typedef enum { FOUR_CORNERS, EIGHT_CORNERS } ComputationMethod;

      /** \brief Constructor
        * \param[in] method the method to be used to determine the corner responses
        * \param[in] window_size
        * \param[in] first_threshold the threshold used in the simple cornerness test.
        * \param[in] second_threshold the threshold used to reject weak corners.
        */
      TrajkovicKeypoint2D (ComputationMethod method = FOUR_CORNERS,
                           int window_size = 3,
                           float first_threshold = 0.1,
                           float second_threshold = 100.0)
        : method_ (method)
        , window_size_ (window_size)
        , first_threshold_ (first_threshold)
        , second_threshold_ (second_threshold)
        , threads_ (1)
      {
        name_ = "TrajkovicKeypoint2D";
      }

      /** \brief set the method of the response to be calculated.
        * \param[in] method either 4 corners or 8 corners
        */
      inline void
      setMethod (ComputationMethod method) { method_ = method; }

      /// \brief \return the computation method
      inline ComputationMethod
      getMethod () const { return (method_); }

      /// \brief Set window size
      inline void
      setWindowSize (int window_size) { window_size_= window_size; }

      /// \brief \return window size i.e. window width or height
      inline int
      getWindowSize () const { return (window_size_); }

      /** \brief set the first_threshold to reject corners in the simple cornerness
        * computation stage.
        * \param[in] threshold
        */
      inline void
      setFirstThreshold (float threshold) { first_threshold_= threshold; }

      /// \brief \return first threshold
      inline float
      getFirstThreshold () const { return (first_threshold_); }

      /** \brief set the second threshold to reject corners in the final cornerness
        * computation stage.
        * \param[in] threshold
        */
      inline void
      setSecondThreshold (float threshold) { second_threshold_= threshold; }

      /// \brief \return second threshold
      inline float
      getSecondThreshold () const { return (second_threshold_); }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use, 0 for automatic.
        */
      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

      /// \brief \return the number of threads
      inline unsigned int
      getNumberOfThreads () const { return (threads_); }

    protected:
      bool
      initCompute ();

      void
      detectKeypoints (PointCloudOut &output);

    private:
      /// comparator for responses intensity
      inline bool
      greaterCornernessAtIndices (int a, int b) const
      {
        return (response_->points [a] > response_->points [b]);
      }

      /// computation method
      ComputationMethod method_;
      /// Window size
      int window_size_;
      /// half window size
      int half_window_size_;
      /// intensity field accessor
      IntensityT intensity_;
      /// first threshold for quick rejection
      float first_threshold_;
      /// second threshold for corner evaluation
      float second_threshold_;
      /// number of threads to be used
      unsigned int threads_;
      /// point cloud response
      pcl::PointCloud<float>::Ptr response_;
  };
}

#include <pcl/keypoints/impl/trajkovic_2d.hpp>

#endif // #ifndef PCL_TRAJKOVIC_KEYPOINT_2D_H_
