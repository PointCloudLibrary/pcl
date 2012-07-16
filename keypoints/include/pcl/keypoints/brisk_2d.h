/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_KEYPOINTS_BRISK_KEYPOINT_2D_H_
#define PCL_KEYPOINTS_BRISK_KEYPOINT_2D_H_

#include <pcl/keypoints/agast_2d.h>

namespace pcl
{
  /** \brief Detects BRISK interest points.
    *
    * \author Radu B. Rusu, Stefan Holzer
    * \ingroup keypoints
    */
  template <typename PointInT, typename IntensityT= pcl::common::IntensityFieldAccessor<PointInT> >
  class BriskKeypoint2D: public Keypoint<PointInT, pcl::PointWithScale>
  {
    public:
      typedef typename Keypoint<PointInT, pcl::PointWithScale>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, pcl::PointWithScale>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, pcl::PointWithScale>::KdTree KdTree;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      using Keypoint<PointInT, pcl::PointWithScale>::name_;
      using Keypoint<PointInT, pcl::PointWithScale>::input_;
      using Keypoint<PointInT, pcl::PointWithScale>::indices_;
      using Keypoint<PointInT, pcl::PointWithScale>::k_;

      /** \brief Constructor */
      BriskKeypoint2D (int octaves = 4, int threshold = 60)
        : threshold_ (threshold)
        , octaves_ (octaves)
      {
        k_ = 1;
        name_ = "BriskKeypoint2D";
      }

      /** \brief Destructor. */
      virtual ~BriskKeypoint2D ()
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

      /** \brief Set the number of octaves to use
        * \param[in] octaves the number of octaves to use
        */
      inline void
      setOctaves (const int octaves)
      {
        octaves_ = octaves;
      }

      /** \brief Returns the number of octaves used. */
      inline int
      getOctaves ()
      {
        return (octaves_);
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

      int octaves_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  namespace keypoints
  {
    namespace brisk
    {
      /** A layer in the BRISK detector pyramid */
      class PCL_EXPORTS Layer
      {
        public:
          // constructor arguments
          struct CommonParams
          {
            static const int HALFSAMPLE = 0;
            static const int TWOTHIRDSAMPLE = 1;
          };
          // construct a base layer
          Layer (const std::vector<unsigned char>& img, 
                 int width, int height, 
                 float scale = 1.0f, float offset = 0.0f);
          // derive a layer
          Layer (const Layer& layer, int mode);

          // Fast/Agast without non-max suppression
          void 
          getAgastPoints (uint8_t threshold, std::vector<pcl::PointXY, Eigen::aligned_allocator<pcl::PointXY> > &keypoints);

          // get scores - attention, this is in layer coordinates, not scale=1 coordinates!
          inline uint8_t 
          getAgastScore (int x, int y, uint8_t threshold);
          inline uint8_t 
          getAgastScore_5_8 (int x, int y, uint8_t threshold);
          inline uint8_t 
          getAgastScore (float xf, float yf, uint8_t threshold, float scale = 1.0f);

          // half sampling
          inline void 
          halfsample (const std::vector<unsigned char>& srcimg,
                      int srcwidth, int srcheight,
                      std::vector<unsigned char>& dstimg,
                      int dstwidth, int dstheight);

          // two third sampling
          inline void 
          twothirdsample (const std::vector<unsigned char>& srcimg,
                          int srcwidth, int srcheight,
                          std::vector<unsigned char>& dstimg,
                          int dstwidth, int dstheight);

          /** access gray values (smoothed/interpolated) */
          inline uint8_t 
          getValue (const std::vector<unsigned char>& mat, 
                    int width, int height, float xf, float yf, float scale);

          /** the image */
          std::vector<unsigned char> img_;
          int img_width_;
          int img_height_;
          
          /** its Fast scores */
          std::vector<unsigned char> scores_;

          /** coordinate transformation */
          float scale_;
          float offset_;

        private:
          /** agast */
          boost::shared_ptr<pcl::keypoints::agast::OastDetector9_16> oast_detector_;
          boost::shared_ptr<pcl::keypoints::agast::AgastDetector5_8> agast_detector_5_8_;
      };

      /** BRISK Scale Space helper.
        */ 
      class PCL_EXPORTS ScaleSpace
      {
        public:
          // construct telling the octaves number:
          ScaleSpace (uint8_t octaves = 3);
          ~ScaleSpace ();

          // construct the image pyramids
          void 
          constructPyramid (const std::vector<unsigned char>& image,
                            int width, int height);

          // get Keypoints
          void 
          getKeypoints (const uint8_t threshold, 
                        std::vector<pcl::PointWithScale, Eigen::aligned_allocator<pcl::PointWithScale> >  &keypoints);

        protected:
          /** Nonmax suppression. */
          inline bool 
          isMax2D (const uint8_t layer, const int x_layer, const int y_layer);

          /** 1D (scale axis) refinement: around octave */
          inline float 
          refine1D (const float s_05, const float s0, const float s05, float& max); 

          /** 1D (scale axis) refinement: around intra */
          inline float 
          refine1D_1 (const float s_05, const float s0, const float s05, float& max); 

          /** 1D (scale axis) refinement: around octave 0 only */
          inline float 
          refine1D_2 (const float s_05, const float s0, const float s05, float& max); 

          /** 2D maximum refinement */
          inline float 
          subpixel2D (const int s_0_0, const int s_0_1, const int s_0_2,
                      const int s_1_0, const int s_1_1, const int s_1_2,
                      const int s_2_0, const int s_2_1, const int s_2_2,
                      float& delta_x, float& delta_y);

          /** 3D maximum refinement centered around (x_layer,y_layer) */
          inline float 
          refine3D (const uint8_t layer,
                    const int x_layer, const int y_layer,
                    float& x, float& y, float& scale, bool& ismax);

          /** interpolated score access with recalculation when needed */
          inline int 
          getScoreAbove (const uint8_t layer, const int x_layer, const int y_layer);
          
          inline int 
          getScoreBelow (const uint8_t layer, const int x_layer, const int y_layer);

          /** return the maximum of score patches above or below */
          inline float 
          getScoreMaxAbove (const uint8_t layer,
                            const int x_layer, const int y_layer,
                            const int threshold, bool& ismax,
                            float& dx, float& dy);

          inline float 
          getScoreMaxBelow (const uint8_t layer,
                            const int x_layer, const int y_layer,
                            const int threshold, bool& ismax,
                            float& dx, float& dy);

          // the image pyramids
          uint8_t layers_;
          std::vector<pcl::keypoints::brisk::Layer> pyramid_;

          // Agast
          uint8_t threshold_;
          uint8_t safe_threshold_;

          // some constant parameters
          static const float safety_factor_ = 1.0;
          static const float basic_size_    = 12.0;
      };
    } // namespace brisk
  } // namespace keypoints

}

#include "pcl/keypoints/impl/brisk_2d.hpp"

#endif
