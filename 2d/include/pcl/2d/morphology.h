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

#include <pcl/pcl_base.h>

namespace pcl {

template <typename PointT>
class Morphology : public PCLBase<PointT> {
private:
  using PointCloudIn = pcl::PointCloud<PointT>;
  using PointCloudInPtr = typename PointCloudIn::Ptr;

  PointCloudInPtr structuring_element_;

public:
  using PCLBase<PointT>::input_;

  Morphology() {}

  /** \brief This function performs erosion followed by dilation.
   * It is useful for removing noise in the form of small blobs and patches.
   * \param[out] output Output point cloud passed by reference
   */
  void
  openingBinary(pcl::PointCloud<PointT>& output);

  /** \brief This function performs dilation followed by erosion.
   * It is useful for filling up (holes/cracks/small discontinuities) in a binary
   * segmented region
   * \param[out] output Output point cloud passed by reference
   */
  void
  closingBinary(pcl::PointCloud<PointT>& output);

  /** \brief Binary dilation is similar to a logical disjunction of sets.
   * At each pixel having value 1, if for all pixels in the structuring element  having
   * value 1, the corresponding pixels in the input image are also 1, the center pixel
   * is set to 1. Otherwise, it is set to 0.
   *
   * \param[out] output Output point cloud passed by reference
   */
  void
  erosionBinary(pcl::PointCloud<PointT>& output);

  /** \brief Binary erosion is similar to a logical addition of sets.
   * At each pixel having value 1, if at least one pixel in the structuring  element is
   * 1 and the corresponding point in the input image is 1, the center pixel is set
   * to 1. Otherwise, it is set to 0.
   *
   * \param[out] output Output point cloud passed by reference
   */
  void
  dilationBinary(pcl::PointCloud<PointT>& output);

  /** \brief Grayscale erosion followed by dilation.
   * This is used to remove small bright artifacts from the image. Large bright objects
   * are relatively undisturbed.
   *
   * \param[out] output Output point cloud passed by reference
   */
  void
  openingGray(pcl::PointCloud<PointT>& output);

  /** \brief Grayscale dilation followed by erosion.
   * This is used to remove small dark artifacts from the image. Bright features or
   * large dark features are relatively undisturbed.
   *
   * \param[out] output Output point cloud passed by reference
   */
  void
  closingGray(pcl::PointCloud<PointT>& output);

  /** \brief Takes the min of the pixels where kernel is 1
   * \param[out] output Output point cloud passed by reference
   */
  void
  erosionGray(pcl::PointCloud<PointT>& output);

  /** \brief Takes the max of the pixels where kernel is 1
   * \param[out] output Output point cloud passed by reference
   */
  void
  dilationGray(pcl::PointCloud<PointT>& output);

  /** \brief Set operation
   * output = input1 - input2
   * \param[out] output Output point cloud passed by reference
   * \param[in] input1
   * \param[in] input2
   */
  void
  subtractionBinary(pcl::PointCloud<PointT>& output,
                    const pcl::PointCloud<PointT>& input1,
                    const pcl::PointCloud<PointT>& input2);

  /** \brief Set operation
   * \f$output = input1 \cup input2\f$
   * \param[out] output Output point cloud passed by reference
   * \param[in] input1
   * \param[in] input2
   */
  void
  unionBinary(pcl::PointCloud<PointT>& output,
              const pcl::PointCloud<PointT>& input1,
              const pcl::PointCloud<PointT>& input2);

  /** \brief Set operation \f$ output = input1 \cap input2 \f$
   * \param[out] output Output point cloud passed by reference
   * \param[in] input1
   * \param[in] input2
   */
  void
  intersectionBinary(pcl::PointCloud<PointT>& output,
                     const pcl::PointCloud<PointT>& input1,
                     const pcl::PointCloud<PointT>& input2);

  /** \brief Creates a circular structing element. The size of the kernel created is
   * 2*radius x 2*radius. Center of the structuring element is the center of the circle.
   * All values lying on the circle are 1 and the others are 0.
   *
   * \param[out] kernel structuring element kernel passed by reference
   * \param[in] radius Radius of the circular structuring element.
   */
  void
  structuringElementCircular(pcl::PointCloud<PointT>& kernel, const int radius);

  /** \brief Creates a rectangular structing element of size height x width.         *
   * All values are 1.
   *
   * \param[out] kernel structuring element kernel passed by reference
   * \param[in] height height number of rows in the structuring element
   * \param[in] width number of columns in the structuring element
   *
   */
  void
  structuringElementRectangle(pcl::PointCloud<PointT>& kernel,
                              const int height,
                              const int width);

  enum MORPHOLOGICAL_OPERATOR_TYPE {
    EROSION_GRAY,
    DILATION_GRAY,
    OPENING_GRAY,
    CLOSING_GRAY,
    EROSION_BINARY,
    DILATION_BINARY,
    OPENING_BINARY,
    CLOSING_BINARY
  };

  MORPHOLOGICAL_OPERATOR_TYPE operator_type;

  /**
   * \param[out] output Output point cloud passed by reference
   */
  void
  applyMorphologicalOperation(pcl::PointCloud<PointT>& output);

  /**
   * \param[in] structuring_element The structuring element to be used for the
   *                                 morphological operation
   */
  void
  setStructuringElement(const PointCloudInPtr& structuring_element);
};

} // namespace pcl

#include <pcl/2d/impl/morphology.hpp>
