/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (C) 2011, The Autonomous Systems Lab (ASL), ETH Zurich, 
 *                      Stefan Leutenegger, Simon Lynen and Margarita Chli.
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
  /** \brief Detects BRISK interest points based on the original code and paper
    * reference by
    * 
    * \par
    * Stefan Leutenegger,Margarita Chli and Roland Siegwart, 
    * BRISK: Binary Robust Invariant Scalable Keypoints, 
    * in Proceedings of the IEEE International Conference on Computer Vision (ICCV2011).
    *
    * Code example:
    *
    * \code
    * pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    * pcl::BriskKeypoint2D<pcl::PointXYZRGBA> brisk;
    * brisk.setThreshold (60);
    * brisk.setOctaves (4);
    * brisk.setInputCloud (cloud);
    *
    * PointCloud<pcl::PointWithScale> keypoints;
    * brisk.compute (keypoints);
    * \endcode
    *
    * \author Radu B. Rusu, Stefan Holzer
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT = pcl::PointWithScale, typename IntensityT = pcl::common::IntensityFieldAccessor<PointInT> >
  class BriskKeypoint2D: public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<BriskKeypoint2D<PointInT, PointOutT, IntensityT> > Ptr;
      typedef boost::shared_ptr<const BriskKeypoint2D<PointInT, PointOutT, IntensityT> > ConstPtr;

      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, PointOutT>::KdTree KdTree;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::k_;

      /** \brief Constructor */
      BriskKeypoint2D (int octaves = 4, int threshold = 60)
        : threshold_ (threshold)
        , octaves_ (octaves)
        , remove_invalid_3D_keypoints_ (false)
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

      /** \brief Specify whether we should do a 2nd pass through the list of keypoints
        * found, and remove the ones that do not have a valid 3D (x-y-z) position 
        * (i.e., are NaN or Inf).
        * \param[in] remove set to true whether we want the invalid 3D keypoints removed
        */
      inline void
      setRemoveInvalid3DKeypoints (bool remove)
      {
        remove_invalid_3D_keypoints_ = remove;
      }

      /** \brief Specify whether the keypoints that do not have a valid 3D position are
        * kept (false) or removed (true).
        */
      inline bool
      getRemoveInvalid3DKeypoints ()
      {
        return (remove_invalid_3D_keypoints_);
      }

      /////////////////////////////////////////////////////////////////////////
      inline void
      bilinearInterpolation (const PointCloudInConstPtr &cloud, 
                             float x, float y,
                             PointOutT &pt)
      {
        int u = int (x);
        int v = int (y);
        
        pt.x = pt.y = pt.z = 0;

        const PointInT &p1 = (*cloud)(u,   v);
        const PointInT &p2 = (*cloud)(u+1, v);
        const PointInT &p3 = (*cloud)(u,   v+1);
        const PointInT &p4 = (*cloud)(u+1, v+1);
        
        float fx = x - float (u), fy = y - float (v);
        float fx1 = 1.0f - fx, fy1 = 1.0f - fy;

        float w1 = fx1 * fy1, w2 = fx * fy1, w3 = fx1 * fy, w4 = fx * fy;
        float weight = 0;
        
        if (pcl::isFinite (p1))
        {
          pt.x += p1.x * w1;
          pt.y += p1.y * w1;
          pt.z += p1.z * w1;
          weight += w1;
        }
        if (pcl::isFinite (p2))
        {
          pt.x += p2.x * w2;
          pt.y += p2.y * w2;
          pt.z += p2.z * w2;
          weight += w2;
        }
        if (pcl::isFinite (p3))
        {
          pt.x += p3.x * w3;
          pt.y += p3.y * w3;
          pt.z += p3.z * w3;
          weight += w3;
        }
        if (pcl::isFinite (p4))
        {
          pt.x += p4.x * w4;
          pt.y += p4.y * w4;
          pt.z += p4.z * w4;
          weight += w4;
        }

        if (weight == 0)
          pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        else
        {
          weight = 1.0f / weight;
          pt.x *= weight; pt.y *= weight; pt.z *= weight;
        }
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

      /** \brief Specify whether the keypoints that do not have a valid 3D position are
        * kept (false) or removed (true).
        */
      bool remove_invalid_3D_keypoints_;
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  namespace keypoints
  {
    namespace brisk
    {
      /** \brief A layer in the BRISK detector pyramid. */
      class PCL_EXPORTS Layer
      {
        public:
          // constructor arguments
          struct CommonParams
          {
            static const int HALFSAMPLE = 0;
            static const int TWOTHIRDSAMPLE = 1;
          };

          /** \brief Constructor.
            * \param[in] img input image
            * \param[in] width image width
            * \param[in] height image height
            * \param[in] scale scale
            * \param[in] offset offset
            */
          Layer (const std::vector<unsigned char>& img, 
                 int width, int height, 
                 float scale = 1.0f, float offset = 0.0f);
        
          /** \brief Copy constructor for deriving a layer.
            * \param[in] layer layer to derive from
            * \param[in] mode deriving mode
            */
          Layer (const Layer& layer, int mode);

          /** \brief AGAST keypoints without non-max suppression.
            * \param[in] threshold the keypoints threshold
            * \param[out] keypoints the AGAST keypoints
            */
          void 
          getAgastPoints (uint8_t threshold, std::vector<pcl::PointUV, Eigen::aligned_allocator<pcl::PointUV> > &keypoints);

          // get scores - attention, this is in layer coordinates, not scale=1 coordinates!
          /** \brief Get the AGAST keypoint score for a given pixel using a threshold
            * \param[in] x the U coordinate of the pixel
            * \param[in] y the V coordinate of the pixel
            * \param[in] threshold the threshold to use for cutting the response
            */
          uint8_t 
          getAgastScore (int x, int y, uint8_t threshold);
          /** \brief Get the AGAST keypoint score for a given pixel using a threshold
            * \param[in] x the U coordinate of the pixel
            * \param[in] y the V coordinate of the pixel
            * \param[in] threshold the threshold to use for cutting the response
            */
          uint8_t 
          getAgastScore_5_8 (int x, int y, uint8_t threshold);
          /** \brief Get the AGAST keypoint score for a given pixel using a threshold
            * \param[in] xf the X coordinate of the pixel
            * \param[in] yf the Y coordinate of the pixel
            * \param[in] threshold the threshold to use for cutting the response
            * \param[in] scale the scale
            */
          uint8_t 
          getAgastScore (float xf, float yf, uint8_t threshold, float scale = 1.0f);

          /** \brief Access gray values (smoothed/interpolated) 
            * \param[in] mat the image
            * \param[in] width the image width
            * \param[in] height the image height
            * \param[in] xf the x coordinate
            * \param[in] yf the y coordinate
            * \param[in] scale the scale
            */
          uint8_t 
          getValue (const std::vector<unsigned char>& mat, 
                    int width, int height, float xf, float yf, float scale);
         
          /** \brief Get the image used. */
          const std::vector<unsigned char>&
          getImage () const
          {
            return (img_);
          }

          /** \brief Get the width of the image used. */
          int
          getImageWidth () const
          {
            return (img_width_);
          }

          /** \brief Get the height of the image used. */
          int
          getImageHeight () const
          {
            return (img_height_);
          }

          /** \brief Get the scale used. */
          float
          getScale () const
          {
            return (scale_);
          }

          /** \brief Get the offset used. */
          inline float
          getOffset () const
          {
            return (offset_);
          }

          /** \brief Get the scores obtained. */
          inline const std::vector<unsigned char>&
          getScores () const
          {
            return (scores_);
          }

        private:
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

          /** the image */
          std::vector<unsigned char> img_;
          int img_width_;
          int img_height_;

          /** its Fast scores */
          std::vector<unsigned char> scores_;

          /** coordinate transformation */
          float scale_;
          float offset_;

          /** agast */
          boost::shared_ptr<pcl::keypoints::agast::OastDetector9_16> oast_detector_;
          boost::shared_ptr<pcl::keypoints::agast::AgastDetector5_8> agast_detector_5_8_;
      };

      /** BRISK Scale Space helper. */ 
      class PCL_EXPORTS ScaleSpace
      {
        public:
          /** \brief Constructor. Specify the number of octaves.
            * \param[in] octaves the number of octaves (default: 3)
            */
          ScaleSpace (int octaves = 3);
          ~ScaleSpace ();

          /** \brief Construct the image pyramids.
            * \param[in] image the image to construct pyramids for
            * \param[in] width the image width
            * \param[in] height the image height
            */ 
          void 
          constructPyramid (const std::vector<unsigned char>& image,
                            int width, int height);

          /** \brief Get the keypoints for the associated image and threshold.
            * \param[in] threshold the threshold for the keypoints
            * \param[out] keypoints the resultant list of keypoints
            */
          void 
          getKeypoints (const int threshold, 
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
          float safety_factor_;
          float basic_size_;
      };
    } // namespace brisk
  } // namespace keypoints

}

#include <pcl/keypoints/impl/brisk_2d.hpp>

#endif
