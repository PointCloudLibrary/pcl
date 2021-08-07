/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * @author: Koen Buys
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/device_array.h>
#include <pcl/gpu/people/label_common.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

namespace pcl
{
  namespace device
  {
    class ProbabilityProc;
  }

  namespace gpu
  {
    namespace people
    {
      class PCL_EXPORTS ProbabilityProcessor
      {        
        public:
          using Ptr = shared_ptr<ProbabilityProcessor>;
          using ConstPtr = shared_ptr<const ProbabilityProcessor>;
          using Depth = DeviceArray2D<unsigned short>;
          using Labels = DeviceArray2D<unsigned char>;

          ProbabilityProcessor();

          /** \brief This will merge the votes from the different trees into one final vote, including probabilistic's **/
          void
          SelectLabel (const Depth& depth, Labels& labels, pcl::device::LabelProbability& probabilities);

          /** \brief This will combine two probabilities according their weight **/
          void
          CombineProb ( const Depth& depth,
                        pcl::device::LabelProbability& probIn1,
                        float weight1,
                        pcl::device::LabelProbability& probIn2,
                        float weight2,
                        pcl::device::LabelProbability& probOut);

          /** \brief This will sum a probability multiplied with it's weight **/
          void
          WeightedSumProb ( const Depth& depth, pcl::device::LabelProbability& probIn, float weight, pcl::device::LabelProbability& probOut);

          /** \brief This will create a Gaussian Kernel **/
          float*
          CreateGaussianKernel ( float sigma,
                                 int kernelSize);

          /** \brief This will do a GaussianBlur over the LabelProbability **/
          int
          GaussianBlur( const Depth&                    depth,
                        pcl::device::LabelProbability&  probIn,
                        DeviceArray<float>&             kernel,
                        pcl::device::LabelProbability&  probOut);

          /** \brief This will do a GaussianBlur over the LabelProbability **/
          int
          GaussianBlur( const Depth&                    depth,
                        pcl::device::LabelProbability&  probIn,
                        DeviceArray<float>&             kernel,
                        pcl::device::LabelProbability&  probTemp,
                        pcl::device::LabelProbability&  probOut);

        private:
          std::shared_ptr<pcl::device::ProbabilityProc> impl_;

      };
    }
  }
}
