/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_APPS_IN_HAND_SCANNER_INPUT_DATA_PROCESSING_H
#define PCL_APPS_IN_HAND_SCANNER_INPUT_DATA_PROCESSING_H

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/eigen.h>
#include <pcl/apps/in_hand_scanner/common_types.h>
#include <pcl/apps/in_hand_scanner/utils.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <class PointInT, class PointOutT>
  class IntegralImageNormalEstimation;
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InputDataProcessing
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    /** \brief Bundles methods that are applied to the input data from the sensor.
      * \author Martin Saelzle
      * \ingroup apps
      */
    class PCL_EXPORTS InputDataProcessing
    {
      public:

        typedef pcl::PointXYZRGBA              PointXYZRGBA;
        typedef pcl::PointCloud <PointXYZRGBA> CloudXYZRGBA;
        typedef CloudXYZRGBA::Ptr              CloudXYZRGBAPtr;
        typedef CloudXYZRGBA::ConstPtr         CloudXYZRGBAConstPtr;

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        /** \brief Constructor */
        InputDataProcessing ();

        /** \brief Apply the segmentation on the input cloud (XYZ and HSV).
          * \param[in] cloud_in The input cloud.
          * \param[out] cloud_out The segmented cloud.
          * \param[out] cloud_discarded Cloud containing all points that were removed during the HSV segmentation. The points in the XYZ segmentation are NOT used!
          * \return true if success.
          * \note Converts from m to cm.
          */
        bool
        segment (const CloudXYZRGBAConstPtr& cloud_in,
                 CloudXYZRGBNormalPtr&       cloud_out,
                 CloudXYZRGBNormalPtr&       cloud_discarded) const;

        /** \brief Calculate the normals of the input cloud.
          * \param[in] cloud_in The input cloud.
          * \param[out] cloud_out Input cloud with normals appended.
          * \return true if success.
          * \note Converts from m to cm.
          */
        bool
        calculateNormals (const CloudXYZRGBAConstPtr& cloud_in,
                          CloudXYZRGBNormalPtr&       cloud_out) const;

        /** @{ */
        /** \brief Points outside of X - Y - Z - min / max are discarded. The unit is cm. The min values must be smaller than the max values. */
        inline void setXMin (const float x_min) {if (x_min < x_max_) x_min_ = x_min;}
        inline void setXMax (const float x_max) {if (x_max > x_min_) x_max_ = x_max;}
        inline void setYMin (const float y_min) {if (y_min < y_max_) y_min_ = y_min;}
        inline void setYMax (const float y_max) {if (y_max > y_min_) y_max_ = y_max;}
        inline void setZMin (const float z_min) {if (z_min < z_max_) z_min_ = z_min;}
        inline void setZMax (const float z_max) {if (z_max > z_min_) z_max_ = z_max;}

        inline float getXMin () const {return (x_min_);}
        inline float getXMax () const {return (x_max_);}
        inline float getYMin () const {return (y_min_);}
        inline float getYMax () const {return (y_max_);}
        inline float getZMin () const {return (z_min_);}
        inline float getZMax () const {return (z_max_);}
        /** @} */

        /** @{ */
        /** \brief Simple color segmentation in the HSV color space. Points inside of H - S - V min / max are discarded. H must be in the range 0 and 360, S and V in the range 0 and 1.
          * \note If you set values outside of the allowed range the member variables are clamped to the next best value. E.g. H is set to 0 if you pass -1.
          */
        inline void setHMin (const float h_min) {h_min_ = pcl::ihs::clamp (h_min, 0.f, 360.f);}
        inline void setHMax (const float h_max) {h_max_ = pcl::ihs::clamp (h_max, 0.f, 360.f);}
        inline void setSMin (const float s_min) {s_min_ = pcl::ihs::clamp (s_min, 0.f,   1.f);}
        inline void setSMax (const float s_max) {s_max_ = pcl::ihs::clamp (s_max, 0.f,   1.f);}
        inline void setVMin (const float v_min) {v_min_ = pcl::ihs::clamp (v_min, 0.f,   1.f);}
        inline void setVMax (const float v_max) {v_max_ = pcl::ihs::clamp (v_max, 0.f,   1.f);}

        inline float getHMin () const {return (h_min_);}
        inline float getHMax () const {return (h_max_);}
        inline float getSMin () const {return (s_min_);}
        inline float getSMax () const {return (s_max_);}
        inline float getVMin () const {return (v_min_);}
        inline float getVMax () const {return (v_max_);}
        /** @} */

        /** @{ */
        /** \brief If true the color values inside of H - S - V min / max are accepted instead of discarded. */
        inline void setColorSegmentationInverted (const bool hsv_inverted) {hsv_inverted_ = hsv_inverted;}
        inline bool getColorSegmentationInverted () const {return (hsv_inverted_);}
        /** @} */

        /** @{ */
        /** \brief Enable / disable the color segmentation. */
        inline void setColorSegmentationEnabled (const bool hsv_enabled) {hsv_enabled_ = hsv_enabled;}
        inline bool getColorSegmentationEnabled () const {return (hsv_enabled_);}
        /** @} */

        /** @{ */
        /** \brief The XYZ mask is eroded with a kernel of this size. */
        inline void setXYZErodeSize (const unsigned int size) {size_erode_ = size;}
        inline unsigned int getXYZErodeSize () const {return (size_erode_);}
        /** @} */

        /** @{ */
        /** \brief The HSV mask is dilated with a kernel of this size. */
        inline void setHSVDilateSize (const unsigned int size) {size_dilate_ = size;}
        inline unsigned int getHSVDilateSize () const {return (size_dilate_);}
        /** @} */

      private:

        typedef pcl::Normal                            Normal;
        typedef pcl::PointCloud <Normal>               CloudNormals;
        typedef boost::shared_ptr <CloudNormals>       CloudNormalsPtr;
        typedef boost::shared_ptr <const CloudNormals> CloudNormalsConstPtr;

        typedef pcl::IntegralImageNormalEstimation <PointXYZRGBA, Normal> NormalEstimation;
        typedef boost::shared_ptr <NormalEstimation>                      NormalEstimationPtr;
        typedef boost::shared_ptr <const NormalEstimation>                NormalEstimationConstPtr;

        typedef Eigen::Matrix <bool, Eigen::Dynamic, Eigen::Dynamic> MatrixXb;
        typedef Eigen::MatrixXi                                      MatrixXi;

        /** \brief Erodes the input mask k times with a diamond shaped structuring element.
          * \see http://ostermiller.org/dilate_and_erode.html
          */
        void
        erode (MatrixXb& mask, const int k) const;

        /** \brief Dilates the input mask k times with a diamond shaped structuring element.
          * \see http://ostermiller.org/dilate_and_erode.html
          */
        void
        dilate (MatrixXb& mask, const int k) const;

        /** \brief Calculates the manhattan distance map for the input matrix.
          * \param[in] mat  Input matrix.
          * \param[in] comp Compared value. mat==comp will have zero distance.
          * \return Matrix containing the distances to mat==comp
          * \see http://ostermiller.org/dilate_and_erode.html
          */
        MatrixXi
        manhattan (const MatrixXb& mat, const bool comp) const;

        /** \brief Conversion from the RGB to HSV color space. */
        void
        RGBToHSV (const unsigned char r,
                  const unsigned char g,
                  const unsigned char b,
                  float&              h,
                  float&              s,
                  float&              v) const;

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        NormalEstimationPtr normal_estimation_;

        float x_min_;
        float x_max_;
        float y_min_;
        float y_max_;
        float z_min_;
        float z_max_;

        float h_min_;
        float h_max_;
        float s_min_;
        float s_max_;
        float v_min_;
        float v_max_;

        bool hsv_inverted_;
        bool hsv_enabled_;

        unsigned int size_dilate_;
        unsigned int size_erode_;
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_INPUT_DATA_PROCESSING_H
