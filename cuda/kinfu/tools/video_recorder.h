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

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PCL_FOURCC(c1, c2, c3, c4) (c1 & 255) + ((c2 & 255) << 8) + ((c3 & 255) << 16) + ((c4 & 255) << 24)

namespace pcl
{
  class BufferedRecorder
  {
public:
  typedef cv::Mat Mat;
  typedef cv::Size Size;

  const static int FOURCC = PCL_FOURCC ('X', 'V', 'I', 'D');
  //const static int FOURCC = PCL_FOURCC('M', 'J', 'P', 'G');
  //const static int FOURCC = -1; //select dialog

  BufferedRecorder(const Size size = Size (640, 480), size_t frames = 30 * 15) : size_ (size), total_ (0)
  {
    images_.resize (frames);
    depths_.resize (frames);
    views_.resize (frames);

    for (size_t i = 0; i < frames; ++i)
    {
      images_[i].create (size, CV_8UC3);
      depths_[i].create (size, CV_8U);
      views_[i].create (size, CV_8UC3);
    }
  }

  void
  push_back (const Mat& image, const Mat& depth, const Mat& view)
  {
    if (total_ < images_.size ())
    {
      image.copyTo (images_[total_]);
      depth.copyTo (depths_[total_]);
      view.copyTo ( views_[total_]);
    }
    else
    {
      images_.push_back (image.clone ());
      depths_.push_back (depth.clone ());
      views_.push_back ( view.clone ());
    }
    ++total_;
  }

  bool
  save (const std::string& file = "video.avi") const
  {
    if (total_ > 0)
    {
      Size sz (size_.width * 3, size_.height);

      cv::VideoWriter vw ("video.avi", FOURCC, 30, sz, true);
      if (vw.isOpened () == false)
        return std::cout << "Can't open video for writing" << std::endl, 0;

      std::cout << "Encoding";
      Mat all (sz, CV_8UC3, cv::Scalar (0));

      for (size_t k = 0; k < total_; ++k)
      {
        Mat t;
        Mat d = depths_[k];
        Mat i = images_[k];
        Mat v = views_[k];

        int pos = 0;
        t = all.colRange (pos, pos + d.cols);
        cv::cvtColor (d, t, CV_GRAY2BGR);
        pos += d.cols;

        t = all.colRange (pos, pos + i.cols);
        i.copyTo (t);
        pos += i.cols;

        t = all.colRange (pos, pos + v.cols);
        v.copyTo (t);
        pos += v.cols;

        vw << all;

        std::cout << ".";
      }
      std::cout << "Done" << std::endl;
    }
    return true;
  }
private:
  std::vector<Mat> images_;
  std::vector<Mat> depths_;
  std::vector<Mat> views_;
  Size size_;
  int total_;
  };
}