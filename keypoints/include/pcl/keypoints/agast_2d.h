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
          typedef boost::shared_ptr<AbstractAgastDetector> Ptr;
          typedef boost::shared_ptr<const AbstractAgastDetector> ConstPtr;

          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            * \param[in] bmax the max image value (default: 255)
            */
          AbstractAgastDetector (const size_t width, 
                                 const size_t height, 
                                 const double threshold,
                                 const double bmax) 
            : width_ (width)
            , height_ (height)
            , threshold_ (threshold)
            , nr_max_keypoints_ (std::numeric_limits<unsigned int>::max ())
            , bmax_ (bmax)
          {}

          /** \brief Destructor. */
          virtual ~AbstractAgastDetector () {}

          /** \brief Detects corner points. 
            * \param intensity_data
            * \param output
            */
          void 
          detectKeypoints (const std::vector<unsigned char> &intensity_data, 
                           pcl::PointCloud<pcl::PointUV> &output);

          /** \brief Detects corner points. 
            * \param intensity_data
            * \param output
            */
          void 
          detectKeypoints (const std::vector<float> &intensity_data, 
                           pcl::PointCloud<pcl::PointUV> &output);

          /** \brief Applies non-max-suppression. 
            * \param[in] intensity_data the image data
            * \param[in] input the keypoint positions
            * \param[out] output the resultant keypoints after non-max-supression
            */
          void
          applyNonMaxSuppression (const std::vector<unsigned char>& intensity_data, 
                                  const pcl::PointCloud<pcl::PointUV> &input, 
                                  pcl::PointCloud<pcl::PointUV> &output);

          /** \brief Applies non-max-suppression. 
            * \param[in] intensity_data the image data
            * \param[in] input the keypoint positions
            * \param[out] output the resultant keypoints after non-max-supression
            */
          void
          applyNonMaxSuppression (const std::vector<float>& intensity_data, 
                                  const pcl::PointCloud<pcl::PointUV> &input, 
                                  pcl::PointCloud<pcl::PointUV> &output);

          /** \brief Computes corner score. 
            * \param[in] im the pixels to compute the score at
            */
          virtual int 
          computeCornerScore (const unsigned char* im) const = 0;

          /** \brief Computes corner score. 
            * \param[in] im the pixels to compute the score at
            */
          virtual int 
          computeCornerScore (const float* im) const = 0;

          /** \brief Sets the threshold for corner detection.
            * \param[in] threshold the threshold used for corner detection.
            */
          inline void
          setThreshold (const double threshold)
          {
            threshold_ = threshold;
          }

          /** \brief Get the threshold for corner detection, as set by the user. */
          inline double
          getThreshold ()
          {
            return (threshold_);
          }

          /** \brief Sets the maximum number of keypoints to return. The
            * estimated keypoints are sorted by their internal score.
            * \param[in] nr_max_keypoints set the maximum number of keypoints to return
            */
          inline void
          setMaxKeypoints (const unsigned int nr_max_keypoints)
          {
            nr_max_keypoints_ = nr_max_keypoints;
          }

          /** \brief Get the maximum nuber of keypoints to return, as set by the user. */
          inline unsigned int 
          getMaxKeypoints ()
          {
            return (nr_max_keypoints_);
          }

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          virtual void 
          detect (const unsigned char* im, 
                  std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const = 0;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            */
          virtual void 
          detect (const float* im, 
                  std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &) const = 0;

        protected:

          /** \brief Structure holding an index and the associated keypoint score. */
          struct ScoreIndex
          {
            int idx;
            int score;
          };

          /** \brief Score index comparator. */
          struct CompareScoreIndex
          {
            /** \brief Comparator
              * \param[in] i1 the first score index
              * \param[in] i2 the second score index
              */
            inline bool
            operator() (const ScoreIndex &i1, const ScoreIndex &i2)
            {
              return (i1.score > i2.score);
            }
          };

          /** \brief Initializes the sample pattern. */
          virtual void
          initPattern () = 0;

          /** \brief Non-max-suppression helper method.
            * \param[in] input the keypoint positions
            * \param[in] scores the keypoint scores computed on the image data
            * \param[out] output the resultant keypoints after non-max-supression
            */
          void
          applyNonMaxSuppression (const pcl::PointCloud<pcl::PointUV> &input, 
                                  const std::vector<ScoreIndex>& scores, 
                                  pcl::PointCloud<pcl::PointUV> &output);

          /** \brief Computes corner scores for the specified points. 
            * \param im
            * \param corners_all
            * \param scores
            */
          void 
          computeCornerScores (const unsigned char* im, 
                               const std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners_all, 
                               std::vector<ScoreIndex> & scores);

          /** \brief Computes corner scores for the specified points. 
            * \param im
            * \param corners_all
            * \param scores
            */
          void 
          computeCornerScores (const float* im, 
                               const std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > & corners_all, 
                               std::vector<ScoreIndex> & scores);

          /** \brief Width of the image to process. */
          size_t width_;
          /** \brief Height of the image to process. */
          size_t height_;

          /** \brief Threshold for corner detection. */
          double threshold_;

          /** \brief The maximum number of keypoints to return. */
          unsigned int nr_max_keypoints_;

          /** \brief Max image value. */
          double bmax_;
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
          typedef boost::shared_ptr<AgastDetector7_12s> Ptr;
          typedef boost::shared_ptr<const AgastDetector7_12s> ConstPtr;

          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            * \param[in] bmax the max image value (default: 255)
            */
          AgastDetector7_12s (const size_t width, 
                              const size_t height, 
                              const double threshold,
                              const double bmax = 255) 
            : AbstractAgastDetector (width, height, threshold, bmax)
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

          /** \brief Computes corner score. 
            * \param im 
            */
          int 
          computeCornerScore (const float* im) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const unsigned char* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const float* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const;

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
          typedef boost::shared_ptr<AgastDetector5_8> Ptr;
          typedef boost::shared_ptr<const AgastDetector5_8> ConstPtr;

          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            * \param[in] bmax the max image value (default: 255)
            */
          AgastDetector5_8 (const size_t width, 
                            const size_t height, 
                            const double threshold,
                            const double bmax = 255) 
            : AbstractAgastDetector (width, height, threshold, bmax)
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

          /** \brief Computes corner score. 
            * \param im 
            */
          int 
          computeCornerScore (const float* im) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const unsigned char* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const float* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const;

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
          typedef boost::shared_ptr<OastDetector9_16> Ptr;
          typedef boost::shared_ptr<const OastDetector9_16> ConstPtr;

          /** \brief Constructor. 
            * \param[in] width the width of the image to process
            * \param[in] height the height of the image to process
            * \param[in] threshold the corner detection threshold
            * \param[in] bmax the max image value (default: 255)
            */
          OastDetector9_16 (const size_t width, 
                            const size_t height, 
                            const double threshold,
                            const double bmax = 255) 
            : AbstractAgastDetector (width, height, threshold, bmax)
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

          /** \brief Computes corner score. 
            * \param im 
            */
          int 
          computeCornerScore (const float* im) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const unsigned char* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const;

          /** \brief Detects points of interest (i.e., keypoints) in the given image
            * \param[in] im the image to detect keypoints in 
            * \param[out] corners_all the resultant set of keypoints detected
            */
          void 
          detect (const float* im, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &corners_all) const;

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

  /////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////
  namespace keypoints
  {
    namespace internal
    {
      /////////////////////////////////////////////////////////////////////////////////////
      template <typename Out> 
      struct AgastApplyNonMaxSuppresion
      {
        AgastApplyNonMaxSuppresion (
            const std::vector<unsigned char> &image_data, 
            const pcl::PointCloud<pcl::PointUV> &tmp_cloud,
            const pcl::keypoints::agast::AbstractAgastDetector::Ptr &detector,
            pcl::PointCloud<Out> &output)
        {
          pcl::PointCloud<pcl::PointUV> output_temp;
          detector->applyNonMaxSuppression (image_data, tmp_cloud, output_temp);
          pcl::copyPointCloud<pcl::PointUV, Out> (output_temp, output);
        }
      };

      /////////////////////////////////////////////////////////////////////////////////////
      template <>
      struct AgastApplyNonMaxSuppresion<pcl::PointUV>
      {
        AgastApplyNonMaxSuppresion (
            const std::vector<unsigned char> &image_data, 
            const pcl::PointCloud<pcl::PointUV> &tmp_cloud,
            const pcl::keypoints::agast::AbstractAgastDetector::Ptr &detector,
            pcl::PointCloud<pcl::PointUV> &output)
        {
          detector->applyNonMaxSuppression (image_data, tmp_cloud, output);
        }
      };
      /////////////////////////////////////////////////////////////////////////////////////
      template <typename Out> 
      struct AgastDetector
      {
        AgastDetector (
            const std::vector<unsigned char> &image_data, 
            const pcl::keypoints::agast::AbstractAgastDetector::Ptr &detector,
            pcl::PointCloud<Out> &output)
        {
          pcl::PointCloud<pcl::PointUV> output_temp;
          detector->detectKeypoints (image_data, output_temp);
          pcl::copyPointCloud<pcl::PointUV, Out> (output_temp, output);
        }
      };

      /////////////////////////////////////////////////////////////////////////////////////
      template <>
      struct AgastDetector<pcl::PointUV>
      {
        AgastDetector (
            const std::vector<unsigned char> &image_data, 
            const pcl::keypoints::agast::AbstractAgastDetector::Ptr &detector,
            pcl::PointCloud<pcl::PointUV> &output)
        {
          detector->detectKeypoints (image_data, output);
        }
      };
    } // namespace agast
  } // namespace keypoints

  /////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Detects 2D AGAST corner points. Based on the original work and
    * paper reference by
    *
    * \par
    * Elmar Mair, Gregory D. Hager, Darius Burschka, Michael Suppa, and Gerhard Hirzinger. 
    * Adaptive and generic corner detection based on the accelerated segment test. 
    * In Proceedings of the European Conference on Computer Vision (ECCV'10), September 2010.
    *
    * \note This is an abstract base class. All children must implement a detectKeypoints method, based on the type of AGAST keypoint to be used.
    *
    * \author Stefan Holzer, Radu B. Rusu
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
  class AgastKeypoint2DBase : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, PointOutT>::KdTree KdTree;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef pcl::keypoints::agast::AbstractAgastDetector::Ptr AgastDetectorPtr;
     
      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::k_;

      /** \brief Constructor */
      AgastKeypoint2DBase ()
        : threshold_ (10)
        , apply_non_max_suppression_ (true)
        , bmax_ (255)
        , detector_ ()
        , nr_max_keypoints_ (std::numeric_limits<unsigned int>::max ())
      {
        k_ = 1;
      }

      /** \brief Destructor. */
      virtual ~AgastKeypoint2DBase ()
      {
      }

      /** \brief Sets the threshold for corner detection.
        * \param[in] threshold the threshold used for corner detection.
        */
      inline void
      setThreshold (const double threshold)
      {
        threshold_ = threshold;
      }

      /** \brief Get the threshold for corner detection, as set by the user. */
      inline double
      getThreshold ()
      {
        return (threshold_);
      }

      /** \brief Sets the maximum number of keypoints to return. The
        * estimated keypoints are sorted by their internal score.
        * \param[in] nr_max_keypoints set the maximum number of keypoints to return
        */
      inline void
      setMaxKeypoints (const unsigned int nr_max_keypoints)
      {
        nr_max_keypoints_ = nr_max_keypoints;
      }

      /** \brief Get the maximum nuber of keypoints to return, as set by the user. */
      inline unsigned int 
      getMaxKeypoints ()
      {
        return (nr_max_keypoints_);
      }

      /** \brief Sets the max image data value (affects how many iterations AGAST does)
        * \param[in] bmax the max image data value
        */
      inline void
      setMaxDataValue (const double bmax)
      {
        bmax_ = bmax;
      }

      /** \brief Get the bmax image value, as set by the user. */
      inline double
      getMaxDataValue ()
      {
        return (bmax_);
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

      inline void
      setAgastDetector (const AgastDetectorPtr &detector)
      {
        detector_ = detector;
      }

      inline AgastDetectorPtr
      getAgastDetector ()
      {
        return (detector_);
      }
    protected:

      /** \brief Initializes everything and checks whether input data is fine. */
      bool 
      initCompute ();
      
      /** \brief Detects the keypoints.
        * \param[out] output the resultant keypoints
        */
      virtual void 
      detectKeypoints (PointCloudOut &output) = 0;

      /** \brief Intensity field accessor. */
      IntensityT intensity_;
      
      /** \brief Threshold for corner detection. */
      double threshold_;

      /** \brief Determines whether non-max-suppression is activated. */
      bool apply_non_max_suppression_;

      /** \brief Max image value. */
      double bmax_;

      /** \brief The Agast detector to use. */
      AgastDetectorPtr detector_;

      /** \brief The maximum number of keypoints to return. */
      unsigned int nr_max_keypoints_;
  };

  /** \brief Detects 2D AGAST corner points. Based on the original work and
    * paper reference by
    *
    * \par
    * Elmar Mair, Gregory D. Hager, Darius Burschka, Michael Suppa, and Gerhard Hirzinger. 
    * Adaptive and generic corner detection based on the accelerated segment test. 
    * In Proceedings of the European Conference on Computer Vision (ECCV'10), September 2010.
    *
    * Code example:
    *
    * \code
    * pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    * pcl::AgastKeypoint2D<pcl::PointXYZRGBA> agast;
    * agast.setThreshold (30);
    * agast.setInputCloud (cloud);
    *
    * PointCloud<pcl::PointUV> keypoints;
    * agast.compute (keypoints);
    * \endcode
    *
    * \note The AGAST keypoint type used is 7_12s.
    *
    * \author Stefan Holzer, Radu B. Rusu
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT = pcl::PointUV>
  class AgastKeypoint2D : public AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >
  {
    public:
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::k_;
      using AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >::intensity_;
      using AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >::threshold_;
      using AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >::bmax_;
      using AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >::apply_non_max_suppression_;
      using AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >::detector_;
      using AgastKeypoint2DBase<PointInT, PointOutT, pcl::common::IntensityFieldAccessor<PointInT> >::nr_max_keypoints_;

      /** \brief Constructor */
      AgastKeypoint2D ()
      {
        name_ = "AgastKeypoint2D";
      }

      /** \brief Destructor. */
      virtual ~AgastKeypoint2D ()
      {
      }

    protected:
      /** \brief Detects the keypoints.
        * \param[out] output the resultant keypoints
        */
      virtual void 
      detectKeypoints (PointCloudOut &output);
  };

  /** \brief Detects 2D AGAST corner points. Based on the original work and
    * paper reference by
    *
    * \par
    * Elmar Mair, Gregory D. Hager, Darius Burschka, Michael Suppa, and Gerhard Hirzinger. 
    * Adaptive and generic corner detection based on the accelerated segment test. 
    * In Proceedings of the European Conference on Computer Vision (ECCV'10), September 2010.
    *
    * Code example:
    *
    * \code
    * pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    * pcl::AgastKeypoint2D<pcl::PointXYZRGBA> agast;
    * agast.setThreshold (30);
    * agast.setInputCloud (cloud);
    *
    * PointCloud<pcl::PointUV> keypoints;
    * agast.compute (keypoints);
    * \endcode
    *
    * \note This is a specialized version for PointXYZ clouds, and operates on depth (z) as float. The output keypoints are of the PointXY type.
    * \note The AGAST keypoint type used is 7_12s.
    *
    * \author Stefan Holzer, Radu B. Rusu
    * \ingroup keypoints
    */
  template <>
  class AgastKeypoint2D<pcl::PointXYZ, pcl::PointUV>
    : public AgastKeypoint2DBase<pcl::PointXYZ, pcl::PointUV, pcl::common::IntensityFieldAccessor<pcl::PointXYZ> > 
  {
    public:
      /** \brief Constructor */
      AgastKeypoint2D ()
      {
        name_ = "AgastKeypoint2D";
        bmax_ = 4;    // max data value for an OpenNI camera
      }

      /** \brief Destructor. */
      virtual ~AgastKeypoint2D ()
      {
      }

    protected:
      /** \brief Detects the keypoints.
        * \param[out] output the resultant keypoints
        */
      virtual void 
      detectKeypoints (pcl::PointCloud<pcl::PointUV> &output);
  };

}

#include <pcl/keypoints/impl/agast_2d.hpp>

#endif

