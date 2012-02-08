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

#include "evaluation.h"

#include<iostream>

using namespace pcl::gpu;
using namespace std;

#ifndef HAVE_OPENCV

struct Evaluation::Impl {};

Evaluation::Evaluation(const std::string&) : fx(0.f), fy(0.f), cx(0.f), cy(0.f) { cout << "Evaluation requires OpenCV. Please enable it in cmake-file" << endl; exit(0); }
bool Evaluation::grab (double stamp, pcl::gpu::PtrStepSz<const RGB>& rgb24) { return false; }
bool Evaluation::grab (double stamp, pcl::gpu::PtrStepSz<const unsigned short>& depth) { return false; }
bool Evaluation::grab (double stamp, pcl::gpu::PtrStepSz<const unsigned short>& depth, pcl::gpu::PtrStepSz<const RGB>& rgb24) { return false; }
void Evaluation::saveAllPoses(const pcl::gpu::KinfuTracker& kinfu, int frame_number, const std::string& logfile) const {}

#else

#include <opencv2/highgui/highgui.hpp>
#include<fstream>

using namespace cv;

struct Evaluation::Impl
{
   Mat depth_buffer;
};

Evaluation::Evaluation(const string& folder) : fx(525.f), fy(525.f), cx(319.5f), cy(239.5f), folder_(folder), visualization_(false)
{   
  impl_.reset( new Impl() );

  if (folder_[folder_.size() - 1] != '\\' && folder_[folder_.size() - 1] != '/')
      folder_.push_back('/');

  cout << "initializing evaluation from folder: " << folder_ << endl;
  string depth_file = folder_ + "depth.txt";
  string rgb_file = folder_ + "rgb.txt";
  
  readFile(depth_file, depth_stamps_and_filenames_);
  readFile(rgb_file, rgb_stamps_and_filenames_);  
}

void Evaluation::readFile(const string& file, vector< pair<double,string> >& output)
{
  char buffer[4096];
  vector< pair<double,string> > tmp;
  
  ifstream iff(file.c_str());

  // ignore three header lines
  iff.getline(buffer, sizeof(buffer));
  iff.getline(buffer, sizeof(buffer));
  iff.getline(buffer, sizeof(buffer));
	
  // each line consists of the timestamp and the filename of the depth image
  while (!iff.eof())
  {
    double time; string name;
    iff >> time >> name;
    tmp.push_back(make_pair(time, name));
  }
  tmp.swap(output);
}
  
bool Evaluation::grab (double stamp, PtrStepSz<const RGB>& rgb24)
{
  return false;
}

bool Evaluation::grab (double stamp, PtrStepSz<const unsigned short>& depth)
{  
  int i = static_cast<int>(stamp); // temporary solution, now it expects only index

  if ( i>= (int)depth_stamps_and_filenames_.size())
      return false;

  string file = folder_ + depth_stamps_and_filenames_[i].second;

  cv::Mat d_img = cv::imread(file, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
  if(d_img.empty())
      return false;
   
  if (d_img.elemSize() != sizeof(unsigned short))
  {
    cout << "Image was not opend in 16-bit format. Please use OpenCV 2.3.1 or higher" << endl;
    exit(1);
  }

  // Datasets are with factor 5000 (pixel to m) 
  // http://cvpr.in.tum.de/data/datasets/rgbd-dataset/file_formats#color_images_and_depth_maps
    
  d_img.convertTo(impl_->depth_buffer, d_img.type(), 0.2);
  depth.data = impl_->depth_buffer.ptr<ushort>();
  depth.cols = impl_->depth_buffer.cols;
  depth.rows = impl_->depth_buffer.rows;
  depth.step = impl_->depth_buffer.cols*sizeof(ushort); // 1280 = 640*2

  if (visualization_)
  {			
    cv::Mat scaled = impl_->depth_buffer/5000.0*65535;	
	cv::imshow("EvaluationDepth", scaled);
	cv::waitKey(3);
  }
  return true;
}

bool Evaluation::grab (double stamp, PtrStepSz<const unsigned short>& depth, PtrStepSz<const RGB>& rgb24)
{
  return grab(stamp, depth) && grab(stamp, rgb24);
}

void Evaluation::saveAllPoses(const pcl::gpu::KinfuTracker& kinfu, int frame_number, const std::string& logfile) const
{    
  if (frame_number < 0)
      frame_number = (int)depth_stamps_and_filenames_.size();

  frame_number = std::min(frame_number, (int)kinfu.getNumberOfPoses());

  cout << "Writing " << frame_number << " poses to " << logfile << endl;
  
  ofstream path_file_stream(logfile.c_str());
  path_file_stream.setf(ios::fixed,ios::floatfield);
  
  for(int i = 0; i < frame_number; ++i)
  {
    Eigen::Affine3f pose = kinfu.getCameraPose(i);
    Eigen::Quaternionf q(pose.rotation());
    Eigen::Vector3f t = pose.translation();

    path_file_stream << depth_stamps_and_filenames_[i].first << " ";
    path_file_stream << t[0] << " " << t[1] << " " << t[2] << " ";
    path_file_stream << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
  }
}


#endif /* HAVE_OPENCV */