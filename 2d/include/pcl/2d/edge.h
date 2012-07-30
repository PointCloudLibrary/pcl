/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id$
 *
 */

#ifndef EDGE_H
#define EDGE_H

#include <vector>
#include "pcl/2d/convolution.h"
#include "pcl/2d/kernel.h"
namespace pcl
{
  namespace pcl_2d
  {
    template<typename PointInT, typename PointOutT>
    class edge
    {

private:
    typedef typename pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;

    PointCloudInPtr input_;
    convolution<PointInT> *convolution_;
    kernel<PointInT>  *kernel_;


    /**
     *
     * @param rowOffset row offset for direction in which the edge is to be traced
     * @param colOffset column offset for direction in which the edge is to be traced
     * @param row row location of the edge point
     * @param col column location of the edge point
     * @param maxima point cloud containing the edge information in the magnitude channel
     *
     * This function performs edge tracing for Canny Edge detector.
     */
    void
    cannyTraceEdge (int rowOffset, int colOffset, int row, int col, pcl::PointCloud<pcl::PointXYZI> &maxima);

    /**
     *
     * @param thet point cloud containing the edge information in the direction channel
     *
     * This function discretizes the edge directions in steps of 22.5 degrees.
     */
    void
    discretizeAngles (pcl::PointCloud<PointOutT> &thet);

    /**
     *
     * @param edges point cloud containing all the edges
     * @param maxima point cloud containing the non-max supressed edges
     * @param tLow
     *
     * This function suppresses the edges which don't form a local maximum in the edge direction.
     */
    void
    suppressNonMaxima (pcl::PointCloud<PointXYZIEdge> &edges, pcl::PointCloud<pcl::PointXYZI> &maxima, float tLow);


public:
    enum OUTPUT_TYPE
    {
      OUTPUT_Y,
      OUTPUT_X,
      OUTPUT_X_Y,
      OUTPUT_MAGNITUDE,
      OUTPUT_DIRECTION,
      OUTPUT_MAGNITUDE_DIRECTION,
      OUTPUT_ALL
    };

    enum DETECTOR_KERNEL_TYPE
    {
      CANNY,
      SOBEL,
      PREWITT,
      ROBERTS,
      LOG,
      DERIVATIVE_CENTRAL,
      DERIVATIVE_FORWARD,
      DERIVATIVE_BACKWARD
    };

    OUTPUT_TYPE output_type_;
    DETECTOR_KERNEL_TYPE detector_kernel_type_;
    bool non_maximal_suppression_;
    bool hysteresis_thresholding_;

    float hysteresis_threshold_low_;
    float hysteresis_threshold_high_;
    float non_max_suppression_radius_x_;
    float non_max_suppression_radius_y_;

    edge () :
      output_type_ (OUTPUT_X),
      detector_kernel_type_ (SOBEL),
      non_maximal_suppression_ (false),
      hysteresis_thresholding_ (false),
      hysteresis_threshold_low_ (20),
      hysteresis_threshold_high_ (80),
      non_max_suppression_radius_x_ (3),
      non_max_suppression_radius_y_ (3)
    {
      convolution_ = new convolution<PointInT> ();
      kernel_ = new kernel<PointXYZI>();
    }

    /**
     *
     * @param output
     * @param input_x
     * @param input_y
     */
    void sobelMagnitudeDirection (pcl::PointCloud<PointOutT> &output, pcl::PointCloud<PointInT> &input_x, pcl::PointCloud<PointInT> &input_y);

    /**
     *
     * @param output Output point cloud passed by reference
     * @param input_x Input point cloud passed by reference for the first derivative in the horizontal direction
     * @param input_y Input point cloud passed by reference for the first derivative in the vertical direction
     *
     * Perform Canny edge detection with two separated input images for horizontal and vertical derivatives.
     * All edges of magnitude above t_high are always classified as edges. All edges below t_low are discarded.
     * Edge values between t_low and t_high are classified as edges only if they are connected to edges having magnitude > t_high
     * and are located in a direction perpendicular to that strong edge.
     *
     */
    void canny (pcl::PointCloud<PointOutT> &output, pcl::PointCloud<PointInT> &input_x, pcl::PointCloud<PointInT> &input_y);

    /**
     *
     * @param output
     *
     * This is a convenience function which performs edge detection based on the variable detector_kernel_type_
     */
    void detectEdge (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * All edges of magnitude above t_high are always classified as edges. All edges below t_low are discarded.
     * Edge values between t_low and t_high are classified as edges only if they are connected to edges having magnitude > t_high
     * and are located in a direction perpendicular to that strong edge.
     */
    void detectEdgeCanny (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Uses the Sobel kernel for edge detection.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void detectEdgeSobel (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Uses the Prewitt kernel for edge detection.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void detectEdgePrewitt (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Uses the Roberts kernel for edge detection.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void detectEdgeRoberts (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference.
     * @param kernel_sigma variance of the LoG kernel used.
     * @param kernel_size a LoG kernel of dimensions kernel_size x kernel_size is used.
     *
     * Uses the LoG kernel for edge detection.
     * Zero crossings of the Laplacian operator applied on an image indicate edges.
     * Gaussian kernel is used to smoothen the image prior to the Laplacian.
     * This is because Laplacian uses the second order derivative of the image and hence, is very sensitive to noise.
     * The implementation is not two-step but rather applies the LoG kernel directly.
     */
    void detectEdgeLoG  (pcl::PointCloud<PointOutT> &output, const float kernel_sigma, const float kernel_size);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Computes the image derivatives in X direction using the kernel kernel::derivativeYCentralKernel.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void computeDerivativeXCentral (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Computes the image derivatives in Y direction using the kernel kernel::derivativeYCentralKernel.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void computeDerivativeYCentral (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Computes the image derivatives in X direction using the kernel kernel::derivativeYForwardKernel.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void computeDerivativeXForward (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Computes the image derivatives in Y direction using the kernel kernel::derivativeYForwardKernel.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void computeDerivativeYForward (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Computes the image derivatives in X direction using the kernel kernel::derivativeXBackwardKernel.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void computeDerivativeXBackward (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Computes the image derivatives in Y direction using the kernel kernel::derivativeYBackwardKernel.
     * This function does NOT include a smoothing step.
     * The image should be smoothed before using this function to reduce noise.
     *
     */
    void computeDerivativeYBackward (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Override function to implement the pcl::Filter interface
     */
    void applyFilter (pcl::PointCloud<PointOutT> &output);

    /**
     *
     * @param input pointer to input point cloud
     *
     *
     */
    void setInputCloud (PointCloudInPtr input);
    void setHysteresisThresholdLow(float t_low);
    void setHysteresisThresholdHigh(float t_high);

    };

  }
}
#include <pcl/2d/impl/edge.hpp>
#endif

