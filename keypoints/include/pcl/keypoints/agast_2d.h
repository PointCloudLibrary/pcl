/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
  /** \brief Detects 2D AGAST corner points. Based on the original work and paper referece by:
    *
    * \note
    *   Elmar Mair, Gregory D. Hager, Darius Burschka, Michael Suppa, and Gerhard Hirzinger. 
    *   Adaptive and generic corner detection based on the accelerated segment test. 
    *   In Proceedings of the European Conference on Computer Vision (ECCV'10), September 2010.
    *
    * Code example
    * \code
    * pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    * pcl::BriskKeypoint2D<pcl::PointXYZRGBA> agast;
    * agast.setThreshold (30);
    * agast.setInputCloud (cloud);
    *
    * PointCloud<pcl::PointXY> keypoints;
    * agast.compute (keypoints);
    * \endcode
    *
    * \author Stefan Holzer, Radu B. Rusu
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
      using Keypoint<PointInT, pcl::PointXY>::k_;

      /** \brief Constructor */
      AgastKeypoint2D ()
        : threshold_ (10)
        , apply_non_max_suppression_ (true)
      {
        k_ = 1;
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

      /** \brief Get the threshold for corner detection, as set by the user. */
      inline size_t
      getThreshold ()
      {
        return (threshold_);
      }

      /** \brief Sets whether non-max-suppression is applied or not.
        * \param[in] enabled determines whether non-max-suppression is enabled.
        */
      inline void
      setNonMaxSuppression (const bool enabled)
      {
        apply_non_max_suppression_ = enabled;
      }

      /** \brief Returns whether non-max-suppression is applied or not. */
      inline bool
      getNonMaxSuppression ()
      {
        return (apply_non_max_suppression_);
      }

    protected:
      /** \brief Initializes everything and checks whether input data is fine. */
      bool 
      initCompute ();
      
      /** \brief Detects the keypoints. */
      void 
      detectKeypoints (PointCloudOut &output);

    private:
      /** \brief Intensity field accessor. */
      IntensityT intensity_;
      
      /** \brief Threshold for corner detection. */
      int threshold_;

      /** \brief Determines whether non-max-suppression is activated. */
      bool apply_non_max_suppression_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  namespace keypoints
  {
    namespace agast
    {

      /** \brief Abstract detector class for AGAST corner point detectors.
        *        
        * Adapted from the C++ implementation of Elmar Mair 
        * (http://www6.in.tum.de/Main/ResearchAgast).
        *
        * \author Stefan Holzer
        * \ingroup keypoints
        */
      class PCL_EXPORTS AbstractAgastDetector
      {
        public:
          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            */
          AbstractAgastDetector (const size_t width, 
                                 const size_t height, 
                                 const size_t threshold) 
            : width_ (width), height_ (height), threshold_ (threshold)
          {}

          /** \brief Destructor. */
          ~AbstractAgastDetector () {}

          /** \brief Detects corner points. 
            * \param intensity_data
            * \param output
            */
          void 
          detectKeypoints (const std::vector<unsigned char> &intensity_data, 
                           pcl::PointCloud<pcl::PointXY> &output);

          /** \brief Applies non-max-suppression. 
            * \param intensity_data
            * \param input
            * \param output
            */
          void
          applyNonMaxSuppression (const std::vector<unsigned char>& intensity_data, 
                                  const pcl::PointCloud<pcl::PointXY> &input, 
                                  pcl::PointCloud<pcl::PointXY> &output);

          /** \brief Computes corner score. 
            * \param[in] im the pixels to compute the score at
            */
          virtual int 
          computeCornerScore (const unsigned char* im) const = 0;

          /** \brief Sets the threshold for corner detection.
            * \param[in] threshold the threshold used for corner detection.
            */
          inline void
          setThreshold (const int threshold)
          {
            threshold_ = threshold;
          }

          /** \brief Get the threshold for corner detection, as set by the user. */
          inline size_t
          getThreshold ()
          {
            return (threshold_);
          }

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          virtual void 
          detect (const unsigned char* im, 
                  std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > &corners_all) const = 0;

        protected:
          /** \brief Initializes the sample pattern. */
          virtual void
          initPattern () = 0;

          /** \brief Computes corner scores for the specified points. 
            * \param im
            * \param corners_all
            * \param scores
            */
          void 
          computeCornerScores (const unsigned char* im, 
                               const std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > & corners_all, 
                               std::vector<int> & scores);

          /** \brief Width of the image to process. */
          size_t width_;
          /** \brief Height of the image to process. */
          size_t height_;

          /** \brief Threshold for corner detection. */
          size_t threshold_;
      };

      /** \brief Detector class for AGAST corner point detector (7_12s). 
        *        
        * Adapted from the C++ implementation of Elmar Mair 
        * (http://www6.in.tum.de/Main/ResearchAgast).
        *
        * \author Stefan Holzer
        * \ingroup keypoints
        */
      class PCL_EXPORTS AgastDetector7_12s : public AbstractAgastDetector
      {
        public:
          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            */
          AgastDetector7_12s (const size_t width, 
                              const size_t height, 
                              const size_t threshold) 
            : AbstractAgastDetector (width, height, threshold)
          {
            initPattern ();
          }

          /** \brief Destructor. */
          ~AgastDetector7_12s () {}

          /** \brief Computes corner score. 
            * \param im 
            */
          int 
          computeCornerScore (const unsigned char* im) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > &corners_all) const;

        protected:
          /** \brief Initializes the sample pattern. */
          void 
          initPattern ();

        private:
          /** \brief Border width. */
          static const int border_width_ = 2;

          // offsets defining the sample pattern
          int_fast16_t s_offset0_;
          int_fast16_t s_offset1_;
          int_fast16_t s_offset2_;
          int_fast16_t s_offset3_;
          int_fast16_t s_offset4_;
          int_fast16_t s_offset5_;
          int_fast16_t s_offset6_;
          int_fast16_t s_offset7_;
          int_fast16_t s_offset8_;
          int_fast16_t s_offset9_;
          int_fast16_t s_offset10_;
          int_fast16_t s_offset11_;
      };

      /** \brief Detector class for AGAST corner point detector (5_8). 
        *        
        * Adapted from the C++ implementation of Elmar Mair 
        * (http://www6.in.tum.de/Main/ResearchAgast).
        *
        * \author Stefan Holzer
        * \ingroup keypoints
        */
      class PCL_EXPORTS AgastDetector5_8 : public AbstractAgastDetector
      {
        public:
          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            */
          AgastDetector5_8 (const size_t width, 
                            const size_t height, 
                            const size_t threshold) 
            : AbstractAgastDetector (width, height, threshold)
          {
            initPattern ();
          }

          /** \brief Destructor. */
          ~AgastDetector5_8 () {}

          /** \brief Computes corner score. 
            * \param im 
            */
          int 
          computeCornerScore (const unsigned char* im) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > &corners_all) const;

        protected:
          /** \brief Initializes the sample pattern. */
          void 
          initPattern ();

        private:
          /** \brief Border width. */
          static const int border_width_ = 1;

          // offsets defining the sample pattern
          int_fast16_t s_offset0_;
          int_fast16_t s_offset1_;
          int_fast16_t s_offset2_;
          int_fast16_t s_offset3_;
          int_fast16_t s_offset4_;
          int_fast16_t s_offset5_;
          int_fast16_t s_offset6_;
          int_fast16_t s_offset7_;
      };

      /** \brief Detector class for AGAST corner point detector (OAST 9_16). 
        *        
        * Adapted from the C++ implementation of Elmar Mair 
        * (http://www6.in.tum.de/Main/ResearchAgast).
        *
        * \author Stefan Holzer
        * \ingroup keypoints
        */
      class PCL_EXPORTS OastDetector9_16 : public AbstractAgastDetector
      {
        public:
          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            */
          OastDetector9_16 (const size_t width, 
                            const size_t height, 
                            const size_t threshold) 
            : AbstractAgastDetector (width, height, threshold)
          {
            initPattern ();
          }

          /** \brief Destructor. */
          ~OastDetector9_16 () {}

          /** \brief Computes corner score. 
            * \param im 
            */
          int 
          computeCornerScore (const unsigned char* im) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const unsigned char* im, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > &corners_all) const;

        protected:
          /** \brief Initializes the sample pattern. */
          void 
          initPattern ();

        private:
          /** \brief Border width. */
          static const int border_width_ = 3;

          // offsets defining the sample pattern
          int_fast16_t s_offset0_;
          int_fast16_t s_offset1_;
          int_fast16_t s_offset2_;
          int_fast16_t s_offset3_;
          int_fast16_t s_offset4_;
          int_fast16_t s_offset5_;
          int_fast16_t s_offset6_;
          int_fast16_t s_offset7_;
          int_fast16_t s_offset8_;
          int_fast16_t s_offset9_;
          int_fast16_t s_offset10_;
          int_fast16_t s_offset11_;
          int_fast16_t s_offset12_;
          int_fast16_t s_offset13_;
          int_fast16_t s_offset14_;
          int_fast16_t s_offset15_;
      };
    } // namespace agast
  } // namespace keypoints
}

#include "pcl/keypoints/impl/agast_2d.hpp"

#endif
