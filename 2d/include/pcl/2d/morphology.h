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
 */

#ifndef MORPHOLOGY_H_
#define MORPHOLOGY_H_

#include <pcl/pcl_base.h>
namespace pcl
{
  namespace pcl_2d
  {
    template<typename PointT>
    class morphology
    {
private:
    typedef typename pcl::PointCloud<PointT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;

    PointCloudInPtr input_;
    PointCloudInPtr structuring_element_;

public:
    morphology  (){

    }
    /**
     *
     * @param output Output point cloud passed by reference
     *
     * This function performs erosion followed by dilation. It is useful for removing noise in the form of small blobs and patches
     */
    void openingBinary  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * This function performs dilation followed by erosion. It is useful for filling up (holes/cracks/small discontinuities)
     * in a binary segmented region
     */
    void closingBinary  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Binary dilation is similar to a logical disjunction of sets. At each pixel having value 1,
     * if for all pixels in the structuring element having value 1, the corresponding pixels in the input image are
     * also 1, the center pixel is set to 1. Otherwise, it is set to 0.
     */
    void erosionBinary  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * Binary erosion is similar to a logical addition of sets. At each pixel having value 1,
     * if at least one pixel in the structuring element is 1 and the corresponding point in the input image is 1,
     * the center pixel is set to 1. Otherwise, it is set to 0.
     */
    void dilationBinary  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * grayscale erosion followed by dilation.
     * This is used to remove small bright artifacts from the image.
     * Large bright objects are relatively undisturbed.
     */
    void openingGray  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * grayscale dilation followed by erosion.
     * This is used to remove small dark artifacts from the image.
     * bright features or large dark features are relatively undisturbed.
     */
    void closingGray  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * takes the min of the pixels where kernel is 1
     */
    void erosionGray  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     *
     * takes the max of the pixels where kernel is 1
     */
    void dilationGray  (pcl::PointCloud<PointT> &output);

    /**
     *
     * @param output Output point cloud passed by reference
     * @param input1
     * @param input2
     * Set operation
     * output = input1 - input2
     */
    void subtractionBinary  (pcl::PointCloud<PointT> &output, pcl::PointCloud<PointT> &input1, pcl::PointCloud<PointT> &input2);

    /**
     *
     * @param output Output point cloud passed by reference
     * @param input1
     * @param input2
     * Set operation
     * output = input1 <union> input2
     */
    void unionBinary  (pcl::PointCloud<PointT> &output, pcl::PointCloud<PointT> &input1, pcl::PointCloud<PointT> &input2);

    /**
     *
     * @param output Output point cloud passed by reference
     * @param input1
     * @param input2
     * Set operation
     * output = input1 <intersection> input2
     */
    void intersectionBinary  (pcl::PointCloud<PointT> &output, pcl::PointCloud<PointT> &input1, pcl::PointCloud<PointT> &input2);

    /**
     *
     * @param kernel structuring element kernel passed by reference
     * @param radius Radius of the circular structuring element.
     * Creates a circular structing element. The size of the kernel created is 2*radius x 2*radius.
     * Center of the structuring element is the center of the circle.
     * All values lying on the circle are 1 and the others are 0.
     */
    void structuringElementCircular  (pcl::PointCloud<PointT> &kernel, const int radius);

    /**
     *
     * @param kernel structuring element kernel passed by reference
     * @param height height number of rows in the structuring element
     * @param width number of columns in the structuring element
     *
     * Creates a rectangular structing element of size height x width.         *
     * All values are 1.
     */
    void structuringElementRectangle  (pcl::PointCloud<PointT> &kernel, const int height, const int width);

    enum MORPHOLOGICAL_OPERATOR_TYPE
    {
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
     *
     * @param output Output point cloud passed by reference
     */
    void applyMorphologicalOperation  (PointCloud<PointT> &output);
    /**
     *
     * @param structuring_element The structuring element to be used for the morphological operation
     */
    void setStructuringElement (PointCloudInPtr &structuring_element);
    /**
     *
     * @param input sets the input cloud for the operation
     */
    void setInputCloud (PointCloudInPtr &input);
    };
  }
}
#include <pcl/2d/impl/morphology.hpp>
#endif /* MORPHOLOGY_H_ */
