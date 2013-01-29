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

#include <pcl/gpu/people/face_detector.h>
#include <pcl/console/print.h>
#include <pcl/gpu/utils/safe_call.hpp>

#include <cuda_runtime_api.h>

//#include "NCVHaarObjectDetection.hpp"

pcl::gpu::people::FaceDetector::FaceDetector()
{
  cuda_dev_id_ = 0;
  cudaSafeCall ( cudaSetDevice (cuda_dev_id_));
  cudaSafeCall ( cudaGetDeviceProperties (&cuda_dev_prop_, cuda_dev_id_));
  PCL_DEBUG("(I) : FaceDetector : Using GPU: %d ( %s ), arch= %d . %d\n",cuda_dev_id_, cuda_dev_prop_.name, cuda_dev_prop_.major, cuda_dev_prop_.minor);
}

void
pcl::gpu::people::FaceDetector::setDeviceId( int id )
{
  cuda_dev_id_ = id;
  cudaSafeCall ( cudaSetDevice (cuda_dev_id_));
  cudaSafeCall ( cudaGetDeviceProperties (&cuda_dev_prop_, cuda_dev_id_));
}

void
pcl::gpu::people::FaceDetector::process()
{

}

void
pcl::gpu::people::FaceDetector::allocate_buffers(int rows, int cols)
{

}
