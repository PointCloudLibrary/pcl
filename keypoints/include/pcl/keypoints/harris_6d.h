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

#ifndef PCL_HARRIS_KEYPOINT_6D_H_
#define PCL_HARRIS_KEYPOINT_6D_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{

  /** \brief Keypoint detector for detecting corners in 3D (XYZ), 2D (intensity) AND mixed versions of these.
    * \author Suat Gedikli
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
  class HarrisKeypoint6D : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<HarrisKeypoint6D<PointInT, PointOutT, NormalT> > Ptr;
      typedef boost::shared_ptr<const HarrisKeypoint6D<PointInT, PointOutT, NormalT> > ConstPtr;

      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, PointOutT>::KdTree KdTree;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::surface_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::k_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::search_parameter_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      /**
       * @brief Constructor
       * @param radius the radius for normal estimation as well as for non maxima suppression
       * @param threshold the threshold to filter out weak corners
       */
      HarrisKeypoint6D (float radius = 0.01, float threshold = 0.0)
      : threshold_ (threshold)
      , refine_ (true)
      , nonmax_ (true)
      , threads_ (0)
      , normals_ (new pcl::PointCloud<NormalT>)
      , intensity_gradients_ (new pcl::PointCloud<pcl::IntensityGradient>)
      {
        name_ = "HarrisKeypoint6D";
        search_radius_ = radius;
      }
      
      /** \brief Empty destructor */
      virtual ~HarrisKeypoint6D () {}

      /**
       * @brief set the radius for normal estimation and non maxima supression.
       * @param radius
       */
      void setRadius (float radius);

      /**
       * @brief set the threshold value for detecting corners. This is only evaluated if non maxima suppression is turned on.
       * @brief note non maxima suppression needs to be activated in order to use this feature.
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

      virtual void
      setSearchSurface (const PointCloudInConstPtr &cloud) { surface_ = cloud; normals_->clear (); intensity_gradients_->clear ();}

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }
    protected:
      void detectKeypoints (PointCloudOut &output);
      void responseTomasi (PointCloudOut &output) const;
      void refineCorners (PointCloudOut &corners) const;
      void calculateCombinedCovar (const std::vector<int>& neighbors, float* coefficients) const;
    private:
      float threshold_;
      bool refine_;
      bool nonmax_;
      unsigned int threads_;    
      boost::shared_ptr<pcl::PointCloud<NormalT> > normals_;
      boost::shared_ptr<pcl::PointCloud<pcl::IntensityGradient> > intensity_gradients_;
  } ;
}

#include <pcl/keypoints/impl/harris_6d.hpp>

#endif // #ifndef PCL_HARRIS_KEYPOINT_6D_H_

