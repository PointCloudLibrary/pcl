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

#pragma once

#include <pcl/2d/convolution.h>
#include <pcl/2d/kernel.h>
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

namespace pcl {

template <typename PointInT, typename PointOutT>
class Edge {
private:
  using PointCloudIn = pcl::PointCloud<PointInT>;
  using PointCloudInPtr = typename PointCloudIn::Ptr;

  PointCloudInPtr input_;
  pcl::Convolution<PointInT> convolution_;
  kernel<PointInT> kernel_;

  /** \brief This function performs edge tracing for Canny Edge detector.
   *
   * \param[in] rowOffset row offset for direction in which the edge is to be traced
   * \param[in] colOffset column offset for direction in which the edge is to be traced
   * \param[in] row row location of the edge point
   * \param[in] col column location of the edge point
   * \param[out] maxima point cloud containing the edge information in the magnitude
   * channel
   */
  inline void
  cannyTraceEdge(int rowOffset,
                 int colOffset,
                 int row,
                 int col,
                 pcl::PointCloud<pcl::PointXYZI>& maxima);

  /** \brief This function discretizes the edge directions in steps of 22.5 degrees.
   * \param thet point cloud containing the edge information in the direction channel
   */
  void
  discretizeAngles(pcl::PointCloud<PointOutT>& thet);

  /** \brief This function suppresses the edges which don't form a local maximum
   * in the edge direction.
   * \param[in] edges point cloud containing all the edges
   * \param[out] maxima point cloud containing the non-max suppressed edges
   * \param[in] tLow
   */
  void
  suppressNonMaxima(const pcl::PointCloud<PointXYZIEdge>& edges,
                    pcl::PointCloud<pcl::PointXYZI>& maxima,
                    float tLow);

public:
  using Ptr = shared_ptr<Edge<PointInT, PointOutT>>;
  using ConstPtr = shared_ptr<const Edge<PointInT, PointOutT>>;

  enum OUTPUT_TYPE {
    OUTPUT_Y,
    OUTPUT_X,
    OUTPUT_X_Y,
    OUTPUT_MAGNITUDE,
    OUTPUT_DIRECTION,
    OUTPUT_MAGNITUDE_DIRECTION,
    OUTPUT_ALL
  };

  enum DETECTOR_KERNEL_TYPE {
    CANNY,
    SOBEL,
    PREWITT,
    ROBERTS,
    LOG,
    DERIVATIVE_CENTRAL,
    DERIVATIVE_FORWARD,
    DERIVATIVE_BACKWARD
  };

private:
  OUTPUT_TYPE output_type_;
  DETECTOR_KERNEL_TYPE detector_kernel_type_;
  bool non_maximal_suppression_;
  bool hysteresis_thresholding_;

  float hysteresis_threshold_low_;
  float hysteresis_threshold_high_;
  float non_max_suppression_radius_x_;
  float non_max_suppression_radius_y_;

public:
  Edge()
  : output_type_(OUTPUT_X)
  , detector_kernel_type_(SOBEL)
  , non_maximal_suppression_(false)
  , hysteresis_thresholding_(false)
  , hysteresis_threshold_low_(20)
  , hysteresis_threshold_high_(80)
  , non_max_suppression_radius_x_(3)
  , non_max_suppression_radius_y_(3)
  {}

  /** \brief Set the output type.
   * \param[in] output_type the output type
   */
  void
  setOutputType(OUTPUT_TYPE output_type)
  {
    output_type_ = output_type;
  }

  void
  setHysteresisThresholdLow(float threshold)
  {
    hysteresis_threshold_low_ = threshold;
  }

  void
  setHysteresisThresholdHigh(float threshold)
  {
    hysteresis_threshold_high_ = threshold;
  }

  /**
   * \param[in] input_x
   * \param[in] input_y
   * \param[out] output
   */
  void
  sobelMagnitudeDirection(const pcl::PointCloud<PointInT>& input_x,
                          const pcl::PointCloud<PointInT>& input_y,
                          pcl::PointCloud<PointOutT>& output);

  /** Perform Canny edge detection with two separated input images for horizontal and
   * vertical derivatives.
   *
   * All edges of magnitude above t_high are always classified as edges. All edges
   * below t_low are discarded. Edge values between t_low and t_high are classified
   * as edges only if they are connected to edges having magnitude > t_high and are
   * located in a direction perpendicular to that strong edge.
   *
   * \param[in] input_x Input point cloud passed by reference for the first derivative
   *                    in the horizontal direction
   * \param[in] input_y Input point cloud passed by reference for the first derivative
   *                    in the vertical direction
   * \param[out] output Output point cloud passed by reference
   */
  void
  canny(const pcl::PointCloud<PointInT>& input_x,
        const pcl::PointCloud<PointInT>& input_y,
        pcl::PointCloud<PointOutT>& output);

  /** \brief This is a convenience function which performs edge detection based on
   * the variable detector_kernel_type_
   * \param[out] output
   */
  void
  detectEdge(pcl::PointCloud<PointOutT>& output);

  /** \brief All edges of magnitude above t_high are always classified as edges.
   * All edges below t_low are discarded.
   * Edge values between t_low and t_high are classified as edges only if they are
   * connected to edges having magnitude > t_high and are located in a direction
   * perpendicular to that strong edge.
   * \param[out] output Output point cloud passed by reference
   */
  void
  detectEdgeCanny(pcl::PointCloud<PointOutT>& output);

  /** \brief Uses the Sobel kernel for edge detection.
   * This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  detectEdgeSobel(pcl::PointCloud<PointOutT>& output);

  /** \brief Uses the Prewitt kernel for edge detection.
   * This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  detectEdgePrewitt(pcl::PointCloud<PointOutT>& output);

  /** \brief Uses the Roberts kernel for edge detection.
   * This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  detectEdgeRoberts(pcl::PointCloud<PointOutT>& output);

  /** \brief Uses the LoG kernel for edge detection.
   * Zero crossings of the Laplacian operator applied on an image indicate edges.
   * Gaussian kernel is used to smoothen the image prior to the Laplacian.
   * This is because Laplacian uses the second order derivative of the image and hence,
   * is very sensitive to noise. The implementation is not two-step but rather applies
   * the LoG kernel directly.
   *
   * \param[in] kernel_sigma variance of the LoG kernel used.
   * \param[in] kernel_size a LoG kernel of dimensions kernel_size x kernel_size is
   *                        used.
   * \param[out] output Output point cloud passed by reference.
   */
  void
  detectEdgeLoG(const float kernel_sigma,
                const float kernel_size,
                pcl::PointCloud<PointOutT>& output);

  /** \brief Computes the image derivatives in X direction using the kernel
   * kernel::derivativeYCentralKernel. This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  computeDerivativeXCentral(pcl::PointCloud<PointOutT>& output);

  /** \brief Computes the image derivatives in Y direction using the kernel
   * kernel::derivativeYCentralKernel. This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  computeDerivativeYCentral(pcl::PointCloud<PointOutT>& output);

  /** \brief Computes the image derivatives in X direction using the kernel
   * kernel::derivativeYForwardKernel. This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  computeDerivativeXForward(pcl::PointCloud<PointOutT>& output);

  /** \brief Computes the image derivatives in Y direction using the kernel
   * kernel::derivativeYForwardKernel. This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  computeDerivativeYForward(pcl::PointCloud<PointOutT>& output);

  /** \brief Computes the image derivatives in X direction using the kernel
   * kernel::derivativeXBackwardKernel. This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param output Output point cloud passed by reference
   */
  void
  computeDerivativeXBackward(pcl::PointCloud<PointOutT>& output);

  /** \brief Computes the image derivatives in Y direction using the kernel
   * kernel::derivativeYBackwardKernel. This function does NOT include a smoothing step.
   * The image should be smoothed before using this function to reduce noise.
   * \param[out] output Output point cloud passed by reference
   */
  void
  computeDerivativeYBackward(pcl::PointCloud<PointOutT>& output);

  /** \brief Override function to implement the pcl::Filter interface */
  void
  applyFilter(pcl::PointCloud<PointOutT>& /*output*/)
  {}

  /** \brief Set the input point cloud pointer
   * \param[in] input pointer to input point cloud
   */
  void
  setInputCloud(PointCloudInPtr input)
  {
    input_ = input;
  }

  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pcl

#include <pcl/2d/impl/edge.hpp>
