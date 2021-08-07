/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/surface/processing.h>

namespace pcl
{

  /** \brief Bilateral filtering implementation, based on the following paper:
    *   * Kopf, Johannes and Cohen, Michael F. and Lischinski, Dani and Uyttendaele, Matt - Joint Bilateral Upsampling,
    *   * ACM Transactions in Graphics, July 2007
    *
    * Takes in a colored organized point cloud (i.e. PointXYZRGB or PointXYZRGBA), that might contain nan values for the
    * depth information, and it will return an upsampled version of this cloud, based on the formula:
    * \f[
    *    \tilde{S}_p = \frac{1}{k_p} \sum_{q_d \in \Omega} {S_{q_d} f(||p_d - q_d|| g(||\tilde{I}_p-\tilde{I}_q||})
    * \f]
    *
    * where S is the depth image, I is the RGB image and f and g are Gaussian functions centered at 0 and with
    * standard deviations \f$\sigma_{color}\f$ and \f$\sigma_{depth}\f$
    */
  template <typename PointInT, typename PointOutT>
  class BilateralUpsampling: public CloudSurfaceProcessing<PointInT, PointOutT>
  {
    public:
      typedef shared_ptr<BilateralUpsampling<PointInT, PointOutT> > Ptr;
      typedef shared_ptr<const BilateralUpsampling<PointInT, PointOutT> > ConstPtr;

      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;
      using CloudSurfaceProcessing<PointInT, PointOutT>::process;

      using PointCloudOut = pcl::PointCloud<PointOutT>;

      Eigen::Matrix3f KinectVGAProjectionMatrix, KinectSXGAProjectionMatrix;

      /** \brief Constructor. */
      BilateralUpsampling () 
        : window_size_ (5)
        , sigma_color_ (15.0f)
        , sigma_depth_ (0.5f)
      {
        KinectVGAProjectionMatrix << 525.0f, 0.0f, 320.0f,
                                     0.0f, 525.0f, 240.0f,
                                     0.0f, 0.0f, 1.0f;
        KinectSXGAProjectionMatrix << 1050.0f, 0.0f, 640.0f,
                                      0.0f, 1050.0f, 480.0f,
                                      0.0f, 0.0f, 1.0f;
      };

      /** \brief Method that sets the window size for the filter
        * \param[in] window_size the given window size
        */
      inline void
      setWindowSize (int window_size) { window_size_ = window_size; }

      /** \brief Returns the filter window size */
      inline int
      getWindowSize () const { return (window_size_); }

      /** \brief Method that sets the sigma color parameter
        * \param[in] sigma_color the new value to be set
        */
      inline void
      setSigmaColor (const float &sigma_color) { sigma_color_ = sigma_color; }

      /** \brief Returns the current sigma color value */
      inline float
      getSigmaColor () const { return (sigma_color_); }

      /** \brief Method that sets the sigma depth parameter
        * \param[in] sigma_depth the new value to be set
        */
      inline void
      setSigmaDepth (const float &sigma_depth) { sigma_depth_ = sigma_depth; }

      /** \brief Returns the current sigma depth value */
      inline float
      getSigmaDepth () const { return (sigma_depth_); }

      /** \brief Method that sets the projection matrix to be used when unprojecting the points in the depth image
        * back to (x,y,z) positions.
        * \note There are 2 matrices already set in the class, used for the 2 modes available for the Kinect. They
        * are tuned to be the same as the ones in the OpenNiGrabber
        * \param[in] projection_matrix the new projection matrix to be set */
      inline void
      setProjectionMatrix (const Eigen::Matrix3f &projection_matrix) { projection_matrix_ = projection_matrix; }

      /** \brief Returns the current projection matrix */
      inline Eigen::Matrix3f
      getProjectionMatrix () const { return (projection_matrix_); }

      /** \brief Method that does the actual processing on the input cloud.
        * \param[out] output the container of the resulting upsampled cloud */
      void
      process (pcl::PointCloud<PointOutT> &output) override;

    protected:
      void
      performProcessing (pcl::PointCloud<PointOutT> &output) override;

      /** \brief Computes the distance for depth and RGB.
        * \param[out] val_exp_depth distance values for depth
        * \param[out] val_exp_rgb distance values for RGB */
      void
      computeDistances (Eigen::MatrixXf &val_exp_depth, Eigen::VectorXf &val_exp_rgb);

    private:
      int window_size_;
      float sigma_color_, sigma_depth_;
      Eigen::Matrix3f projection_matrix_, unprojection_matrix_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}
