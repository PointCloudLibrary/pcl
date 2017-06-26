/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <iostream>
#include <map>

namespace pcl {

inline float 
Narf::getDescriptorDistance(const Narf& other) const
{
  float ret = L1_Norm(descriptor_, other.descriptor_, descriptor_size_);
  //float ret = Sublinear_Norm(descriptor_, other.descriptor_, descriptor_size_);
  ret /= static_cast<float> (descriptor_size_);
  return (ret);
}

inline void Narf::copyToNarf36(Narf36& narf36) const
{
  if (descriptor_size_ != 36)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": descriptor size is not 36!\n";
    return;
  }
  getTranslationAndEulerAngles(transformation_.inverse (), narf36.x, narf36.y, narf36.z, narf36.roll, narf36.pitch, narf36.yaw);
  memcpy(narf36.descriptor, descriptor_, 36*sizeof(*descriptor_));
}

//inline float Narf::getDescriptorDistance(const Narf& other) const
//{
  //float middle_value = 0.1f;
  //float normalization_factor1 = 1.0f/middle_value,
        //normalization_factor2 = 1.0f/(1.0f-middle_value);
  //const float* descriptor1_ptr = descriptor_;
  //const float* descriptor2_ptr = other.getDescriptor();
  //float ret = 0;
  //for (int i=0; i<descriptor_size_; ++i) {
    //float diff = fabsf(*(descriptor2_ptr++) - *(descriptor1_ptr++));
    //if (diff < middle_value)
    //{
      //diff = diff*normalization_factor1;
      //diff = 0.5f*diff*diff;
      ////diff = 0.5f*powf(diff, 2);
    //}
    //else
    //{
      //diff = (diff - middle_value)*normalization_factor2;
      //diff = 0.5f + 0.5f*diff;
      ////diff = 0.5f + 0.5f*std::sqrt(diff);
      ////diff = 0.5f + 0.5f*powf(diff, 0.3f);
    //}
    //ret += diff;
  //}
  //ret /= descriptor_size_;
  //return ret;
//}

//inline float Narf::getDescriptorDistance(const Narf& other) const
//{
  //float max_diff_between_cells = 0.25;
  
  //const float* descriptor1_ptr = descriptor_;
  //const float* descriptor2_ptr = other.getDescriptor();
  //float ret = 0;
  //for (int i=0; i<descriptor_size_; ++i) {
    //ret += (std::min)(max_diff_between_cells, fabsf(*(descriptor2_ptr++) - *(descriptor1_ptr++)));
  //}
  //ret /= descriptor_size_*max_diff_between_cells;
  //return ret;
//}

}  // namespace end
