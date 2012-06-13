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
 */

#ifndef PCL_KEYPOINTS_AGAST_KEYPOINT_2D_H_
#define PCL_KEYPOINTS_AGAST_KEYPOINT_2D_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/common/intensity.h>

namespace pcl
{
  /** \brief Helper class for AGAST corner point detector.
    *        Adapted from the C++ implementation of Elmar Mair (http://www6.in.tum.de/Main/ResearchAgast).
    *
    * \author Stefan Holzer
    * \ingroup keypoints
    */
  class PCL_EXPORTS AgastHelper7_12
  {
    public:
      /** \brief Constructor. 
        * \param[in] width the width of the image to process
        * \param[in] height the height of the image to process
        */
      AgastHelper7_12 (const size_t width, const size_t height, const size_t threshold) : width_ (width), height_ (height), threshold_ (threshold) 
      {
        initPattern ();
      }
      /** \brief Destructor. */
      ~AgastHelper7_12 () {}

      /** \brief Detects corner points. */
      void 
      detectKeypoints (std::vector<unsigned char> & intensity_data, pcl::PointCloud<pcl::PointXY> &output);

      /** \brief Applys non-max-suppression. */
      void
      applyNonMaxSuppression (std::vector<unsigned char> & intensity_data, pcl::PointCloud<pcl::PointXY> &input, pcl::PointCloud<pcl::PointXY> &output);

    protected:
      /** \brief Initializes the sample pattern. */
      void initPattern ()
      {
			  s_offset0=(-2)+(0)*width_;
			  s_offset1=(-2)+(-1)*width_;
			  s_offset2=(-1)+(-2)*width_;
			  s_offset3=(0)+(-2)*width_;
			  s_offset4=(1)+(-2)*width_;
			  s_offset5=(2)+(-1)*width_;
			  s_offset6=(2)+(0)*width_;
			  s_offset7=(2)+(1)*width_;
			  s_offset8=(1)+(2)*width_;
			  s_offset9=(0)+(2)*width_;
			  s_offset10=(-1)+(2)*width_;
			  s_offset11=(-2)+(1)*width_;
		  }

      /** \brief Detects corners. */
      void detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all) const;

      /** \brief Computes corner score. */
      int computeCornerScore (const unsigned char* im) const;

      /** \brief Computes corner scores for the specified points. */
      void computeCornerScores (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all, std::vector<int> & scores);

    private:
      /** \brief Width of the image to process. */
      size_t width_;
      /** \brief Height of the image to process. */
      size_t height_;

      /** \brief Threshold for corner detection. */
      size_t threshold_;

      /** \brief Border width. */
		  static const int borderWidth = 2;

      // offsets defining the sample pattern
		  int_fast16_t s_offset0;
		  int_fast16_t s_offset1;
		  int_fast16_t s_offset2;
		  int_fast16_t s_offset3;
		  int_fast16_t s_offset4;
		  int_fast16_t s_offset5;
		  int_fast16_t s_offset6;
		  int_fast16_t s_offset7;
		  int_fast16_t s_offset8;
		  int_fast16_t s_offset9;
		  int_fast16_t s_offset10;
		  int_fast16_t s_offset11;
  };

  /** \brief Detects AGAST corner points.
    *        See: Elmar Mair, Gregory D. Hager, Darius Burschka, Michael Suppa, and Gerhard Hirzinger. 
    *             Adaptive and generic corner detection based on the accelerated segment test. 
    *             In Proceedings of the European Conference on Computer Vision (ECCV'10), September 2010.
    *
    * \author Stefan Holzer
    * \ingroup keypoints
    */
  template <typename PointInT, typename IntensityT= pcl::common::IntensityFieldAccessor<PointInT> >
  class AgastKeypoint2D : public Keypoint<PointInT, pcl::PointXY>
  {
    public:
      typedef typename Keypoint<PointInT, pcl::PointXY>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, pcl::PointXY>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, pcl::PointXY>::KdTree KdTree;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      using Keypoint<PointInT, pcl::PointXY>::name_;
      using Keypoint<PointInT, pcl::PointXY>::input_;
      using Keypoint<PointInT, pcl::PointXY>::indices_;


      /** \brief Constructor */
      AgastKeypoint2D ()
        : threshold_ (10)
        , apply_non_max_suppression_ (true)
      {
        name_ = "AgastKeypoint2D";
      }

      /** \brief Destructor. */
      virtual ~AgastKeypoint2D ()
      {
      }

      /** \brief Sets the threshold for corner detection.
        * \param[in] threshold the threshold used for corner detection.
        */
      inline void
      setThreshold (const int threshold)
      {
        threshold_ = threshold;
      }

      /** \brief Sets whether non-max-suppression is applied or not.
        * \param[in] enabled determines whether non-max-suppression is enabled.
        */
      inline void
      setNonMaxSuppression (const bool enabled)
      {
        apply_non_max_suppression_ = enabled;
      }

    protected:
      /** \brief Initializes everything and checks whether input data is fine. */
      bool 
      initCompute ();
      
      /** \brief Detects the key */
      void 
      detectKeypoints (PointCloudOut &output);

    private:
      /// intensity field accessor
      IntensityT intensity_;
      /** \brief threshold for corner detection */
      int threshold_;
      /** \brief determines whether non-max-suppression is activated */
      bool apply_non_max_suppression_;
  };
}

#include "pcl/keypoints/impl/agast_keypoint2D.hpp"

#endif
