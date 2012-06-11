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
 * morphology.h
 *
 *  Created on: May 28, 2012
 *      Author: somani
 */

#ifndef MORPHOLOGY_H_
#define MORPHOLOGY_H_

namespace pcl
{
  namespace pcl_2d
  {
    class morphology
    {
      private:
      public:
        morphology  (){

        }
        /**
         *
         * \param output
         * \param kernel
         * \param input
         *
         * This function performs erosion followed by dilation. It is useful for removing noise in the form of small blobs and patches
         */
        void openingBinary  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input

         * This function performs dilation followed by erosion. It is useful for filling up (holes/cracks/small discontinuities)
         * in a binary segmented region
         */
        void closingBinary  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input
         *
         * Binary dilation is similar to a logical disjunction of sets. At each pixel having value 1,
         * if for all pixels in the structuring element having value 1, the corresponding pixels in the input image are
         * also 1, the center pixel is set to 1. Otherwise, it is set to 0.
         */
        void erosionBinary  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input
         *
         * Binary erosion is similar to a logical addition of sets. At each pixel having value 1,
         * if at least one pixel in the structuring element is 1 and the corresponding point in the input image is 1,
         * the center pixel is set to 1. Otherwise, it is set to 0.
         */
        void dilationBinary  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input
         * grayscale erosion followed by dilation.
         * This is used to remove small bright artifacts from the image.
         * Large bright objects are relatively undisturbed.
         */
        void openingGray  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input
         * grayscale dilation followed by erosion.
         * This is used to remove small dark artifacts from the image.
         * bright features or large dark features are relatively undisturbed.
         */
        void closingGray  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input
         * takes the min of the pixels where kernel is 1
         */
        void erosionGray  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param kernel
         * \param input
         * takes the max of the pixels where kernel is 1
         */
        void dilationGray  (ImageType &output, ImageType &kernel, ImageType &input);

        /**
         *
         * \param output
         * \param input1
         * \param input2
         * Set operation
         * output = input1 - input2
         */
        void subtractionBinary  (ImageType &output, ImageType &input1, ImageType &input2);

        /**
         *
         * \param output
         * \param input1
         * \param input2
         * Set operation
         * output = input1 <union> input2
         */
        void unionBinary  (ImageType &output, ImageType &input1, ImageType &input2);

        /**
         *
         * \param output
         * \param input1
         * \param input2
         * Set operation
         * output = input1 <intersection> input2
         */
        void intersectionBinary  (ImageType &output, ImageType &input1, ImageType &input2);

        /**
         *
         * \param kernel
         * \param radius Radius of the circular structuring element.
         * Creates a circular structing element. The size of the kernel created is 2*radius x 2*radius.
         * Center of the structuring element is the center of the circle.
         * All values lying on the circle are 1 and the others are 0.
         */
        void structuringElementCircular  (ImageType &kernel, const int radius);

        /**
         *
         * \param kernel
         * \param height height number of rows in the structuring element
         * \param width number of columns in the structuring element
         *
         * Creates a rectangular structing element of size height x width.         *
         * All values are 1.
         */
        void structuringElementRectangle  (ImageType &kernel, const int height, const int width);
    };
  }
}
#include <pcl/2d/impl/morphology.hpp>
#endif /* MORPHOLOGY_H_ */
