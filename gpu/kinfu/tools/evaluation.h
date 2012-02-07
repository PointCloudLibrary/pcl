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
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#pragma once

#include <string>
#include <boost/shared_ptr.hpp>
#include <pcl/gpu/containers/kernel_containers.h>
#include "pcl/gpu/kinfu/kinfu.h"


/** \brief  class for  RGB-D SLAM Dataset and Benchmark
  * \author Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
  */
class Evaluation
{
public:
  typedef boost::shared_ptr<Evaluation> Ptr; 
  typedef pcl::gpu::KinfuTracker::PixelRGB RGB;

  Evaluation(const std::string& folder);

  bool grab (double stamp, pcl::gpu::PtrStepSz<const RGB>& rgb24);
  bool grab (double stamp, pcl::gpu::PtrStepSz<const unsigned short>& depth);
  bool grab (double stamp, pcl::gpu::PtrStepSz<const unsigned short>& depth, pcl::gpu::PtrStepSz<const RGB>& rgb24);

  const float fx, fy, cx, cy;


  void saveAllPoses(const pcl::gpu::KinfuTracker& kinfu, int frame_number = -1, const std::string& logfile = "kinfu_poses.txt") const;

private:
  std::string folder_;
  bool visualization_;

  std::vector< std::pair<double, std::string> > rgb_stamps_and_filenames_;
  std::vector< std::pair<double, std::string> > depth_stamps_and_filenames_;

  void readFile(const std::string& file, std::vector< std::pair<double, std::string> >& output);

  struct Impl;
  boost::shared_ptr<Impl> impl_;
};

