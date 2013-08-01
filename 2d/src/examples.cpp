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
 *  $Id$
 */

#include <pcl/point_types.h>

#include <pcl/2d/Convolution.h>
#include <pcl/2d/Edge.h>
#include <pcl/2d/Kernel.h>
#include <pcl/2d/Morphology.h>
#include <pcl/pcl_base.h>

using namespace pcl;

void example_edge ()
{
  Edge<pcl::PointXYZRGB> edge;

  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  /*example 1*/
  edge.output_type_ = Edge<pcl::PointXYZRGB>::OUTPUT_X_Y;
  edge.detectEdgeRoberts (*output_cloud, *input_cloud);

  /*example 2*/
  edge.hysteresis_threshold_low_ = 20;
  edge.hysteresis_threshold_high_ = 80;
  edge.non_max_suppression_radius_x_ = 3;
  edge.non_max_suppression_radius_y_ = 3;
  edge.detectEdgeCanny (*output_cloud, *input_cloud);

  /*example 3*/
  edge.detector_kernel_type_ = Edge<pcl::PointXYZRGB>::PREWITT;
  edge.hysteresis_thresholding_ = true;
  edge.hysteresis_threshold_low_ = 20;
  edge.hysteresis_threshold_high_ = 80;
  edge.non_maximal_suppression_ = true;
  edge.non_max_suppression_radius_x_ = 1;
  edge.non_max_suppression_radius_y_ = 1;
  edge.output_type_ = Edge<pcl::PointXYZRGB>::OUTPUT_X_Y;
  edge.detectEdge (*output_cloud, *input_cloud);
}

void example_convolution ()
{
  Kernel<pcl::PointXYZRGB> kernel;
  Convolution<pcl::PointXYZRGB> convolution;

  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kernel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  /*example 1 : Gaussian Smoothing*/
  kernel.sigma_ = 2.0;
  kernel.kernel_size_ = 3;
  kernel.gaussianKernel (*kernel_cloud);
  convolution.kernel_ = *kernel_cloud;
  convolution.convolve (*output_cloud, *input_cloud);

  /*example 2 : forward derivative in X direction*/
  kernel.kernel_type_ = Kernel<pcl::PointXYZRGB>::DERIVATIVE_FORWARD_X;
  kernel.fetchKernel (*kernel_cloud);
  convolution.kernel_ = *kernel_cloud;
  convolution.convolve (*output_cloud, *input_cloud);

  /*example 3*/
  kernel.kernel_type_ = Kernel<pcl::PointXYZRGB>::DERIVATIVE_FORWARD_X;
  kernel.fetchKernel (convolution.kernel_);
  convolution.convolve (*output_cloud, *input_cloud);
}

void example_morphology ()
{
  Morphology<pcl::PointXYZRGB> morphology;

  /*dummy clouds*/
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr structuring_element_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  /*example 1 : Gaussian Smoothing*/
  morphology.structuringElementCircular (*structuring_element_cloud, 3);
  morphology.structuring_element_ = *structuring_element_cloud;
  morphology.operator_type_ = Morphology<pcl::PointXYZRGB>::EROSION_GRAY;
  morphology.applyMorphologicalOperation (*output_cloud, *input_cloud);

  /*example 2 : forward derivative in X direction*/
  morphology.structuringElementCircular (morphology.structuring_element_, 3);
  morphology.operator_type_ = Morphology<pcl::PointXYZRGB>::EROSION_GRAY;
  morphology.applyMorphologicalOperation (*output_cloud, *input_cloud);

}

int main(char *args, int argv)
{
  return 0;
}
