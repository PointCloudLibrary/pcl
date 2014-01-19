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

#ifndef PCL_SUSAN_KEYPOINT_2D_H_
#define PCL_SUSAN_KEYPOINT_2D_H_

#include <pcl/keypoints/keypoint.h>
#include <pcl/common/intensity.h>

namespace pcl
{
  /** \brief SusanKeypoint2D implements Susan and Hedley corner detector on
    * organized pooint cloud using intensity information.
    * It uses first order statistics to find variation of intensities inside a mask.
    * Originally the mask is circular which is quite slow instead we use the circumsing
    * sqaure window.
    *
    * \author Nizar Sallem
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
  class SusanKeypoint2D : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<SusanKeypoint2D<PointInT, PointOutT, IntensityT> > Ptr;
      typedef boost::shared_ptr<const SusanKeypoint2D<PointInT, PointOutT, IntensityT> > ConstPtr;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      /** \brief Constructor
        * \param[in] radius the radius for keypoint detection and for non maxima
        * suppression, default 3.4px.
        * \param[in] threshold for intensity comparison.
        * \param[in] distance to test if the nucleus is far enough from the centroid,
        */
      SusanKeypoint2D (int window_size = 7, float threshold = 20 / 255.f, float distance = 0.01f)
        : window_size_ (window_size)
        , threshold_ (threshold)
        , distance_threshold_ (distance)
        , threads_ (1)
      {
        name_ = "SusanKeypoint2D";
        test_distance_ = true;
        test_contiguity_ = false;
      }

      /// \brief Set window size
      inline void
      setWindowSize (int window_size) { window_size_= window_size; }

      /// \brief \return window size i.e. window width or height
      inline int
      getWindowSize () const { return (window_size_); }

      /** \brief set the threshold to reject non similar points.
        * \param[in] threshold
        */
      inline void
      setThreshold (float threshold) { threshold_= threshold; }

      /// \brief \return first threshold
      inline float
      getThreshold () const { return (threshold_); }

      /** \brief Set the minimal distance between nucleus and centroid to reject false
        * positive. This is only evaluated if distance test is turned on.
        * \param[in] distance minimal distance between centroid and nucleus.
        */
      void
      setDistanceThreshold (float distance) { distance_threshold_ = distance; }

      /// \brief \return distance threshold
      float
      getDistanceThreshold () const { return (distance_threshold_); }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use, 0 for automatic.
        */
      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }

      /// \brief \return the number of threads
      inline unsigned int
      getNumberOfThreads () const { return (threads_); }

      /** \brief test that a point in the USAN is part of the [nucleus centroid] to filter
        * out false positives, default is false.
        * \remark setting this value to true can drastically slow down computation.
        * \param[in] test_contiguity whether or not to test for continguity
        */
      void
      setTestForContiguity (bool test_contiguity) { test_contiguity_ = test_contiguity; }

      /** \brief test if the nucleus and the centrooid are far enough to filter out false
        * positives, default true.
        * \param[in] test_distance whether or not to test for distance
        */
      void
      setTestForDistance (bool test_distance) { test_distance_ = test_distance; }

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

      /** get pixels indices that form a line segment between start and end using Bresenheim
        * algorithm.
        * param[in] centroid_x the centroid coordinate on x axis
        * param[in] centroid_y the centroid coordinate on y axis
        * param[out] points indices of points lying on [start end] segment
        */
      void
      lineToCentroid (int centroid_x, int centroid_y, std::vector<Eigen::Vector2i>& points);

      /// Window size
      int window_size_;
      /// half window size
      int half_window_size_;
      /// nucleus to centroid distance threshold
      float distance_threshold_;
      /// threshold for c computation
      float threshold_;
      /// geometric threshold
      float geometric_threshold_;
      /// test for distance between centroid and nucleus
      bool test_distance_;
      /// test for contiguity inside USAN
      bool test_contiguity_;
      /// number of threads to be used
      unsigned int threads_;
      /// intensity field accessor
      IntensityT intensity_;
      /// point cloud response
      pcl::PointCloud<float>::Ptr response_;
  };
}

#include <pcl/keypoints/impl/susan_2d.hpp>

#endif // #ifndef PCL_SUSAN_KEYPOINT_2D_H_
