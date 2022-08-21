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
 *
 */

#include "pcl/stereo/stereo_matching.h"

//////////////////////////////////////////////////////////////////////////////
pcl::AdaptiveCostSOStereoMatching::AdaptiveCostSOStereoMatching()
{
  radius_ = 5;

  gamma_s_ = 15;
  gamma_c_ = 25;

  smoothness_strong_ = 100;
  smoothness_weak_ = 20;
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::AdaptiveCostSOStereoMatching::compute_impl(unsigned char* ref_img,
                                                unsigned char* trg_img)
{
  // int n = radius_ * 2 + 1;
  // int sad_max = std::numeric_limits<int>::max ();

  float** acc = new float*[width_];
  for (int d = 0; d < width_; d++) {
    acc[d] = new float[max_disp_]{};
  }

  // data structures for Scanline Optimization
  float** fwd = new float*[width_];
  float** bck = new float*[width_];
  for (int d = 0; d < width_; d++) {
    fwd[d] = new float[max_disp_]{};
    bck[d] = new float[max_disp_]{};
  }

  // spatial distance init
  float* ds = new float[2 * radius_ + 1];
  for (int j = -radius_; j <= radius_; j++)
    ds[j + radius_] = static_cast<float>(std::exp(-std::abs(j) / gamma_s_));

  // LUT for color distance weight computation
  float lut[256];
  for (int j = 0; j < 256; j++)
    lut[j] = float(std::exp(-j / gamma_c_));

  // left weight array alloc
  float* wl = new float[2 * radius_ + 1];

  for (int y = radius_ + 1; y < height_ - radius_; y++) {
    for (int x = x_off_ + max_disp_ + 1; x < width_; x++) {
      for (int j = -radius_; j <= radius_; j++)
        wl[j + radius_] =
            lut[std::abs(ref_img[(y + j) * width_ + x] - ref_img[y * width_ + x])] *
            ds[j + radius_];

      for (int d = 0; d < max_disp_; d++) {
        float sumw = 0.0;
        float num = 0.0;

        for (int j = -radius_; j <= radius_; j++) {
          float weight_r = lut[std::abs(trg_img[(y + j) * width_ + x - d - x_off_] -
                                        trg_img[y * width_ + x - d - x_off_])] *
                           ds[j + radius_];
          int sad = std::abs(ref_img[(y + j) * width_ + x] -
                             trg_img[(y + j) * width_ + x - d - x_off_]);
          num += wl[j + radius_] * weight_r * static_cast<float>(sad);
          sumw += wl[j + radius_] * weight_r;
        }

        acc[x][d] = num / sumw;

      } // d
    }   // x

    // Forward
    for (int d = 0; d < max_disp_; d++)
      fwd[max_disp_ + 1][d] = acc[max_disp_ + 1][d];

    for (int x = x_off_ + max_disp_ + 2; x < width_; x++) {
      float c_min = fwd[x - 1][0];
      for (int d = 1; d < max_disp_; d++)
        if (fwd[x - 1][d] < c_min)
          c_min = fwd[x - 1][d];

      fwd[x][0] =
          acc[x][0] - c_min +
          std::min(fwd[x - 1][0],
                   std::min(fwd[x - 1][1] + static_cast<float>(smoothness_weak_),
                            c_min + static_cast<float>(smoothness_strong_)));
      for (int d = 1; d < max_disp_ - 1; d++) {
        fwd[x][d] =
            acc[x][d] - c_min +
            std::min(std::min(fwd[x - 1][d],
                              fwd[x - 1][d - 1] + static_cast<float>(smoothness_weak_)),
                     std::min(fwd[x - 1][d + 1] + static_cast<float>(smoothness_weak_),
                              c_min + static_cast<float>(smoothness_strong_)));
      }
      fwd[x][max_disp_ - 1] =
          acc[x][max_disp_ - 1] - c_min +
          std::min(
              fwd[x - 1][max_disp_ - 1],
              std::min(fwd[x - 1][max_disp_ - 2] + static_cast<float>(smoothness_weak_),
                       c_min + static_cast<float>(smoothness_strong_)));
    } // x

    // Backward
    for (int d = 0; d < max_disp_; d++)
      bck[width_ - 1][d] = acc[width_ - 1][d];

    for (int x = width_ - 2; x > max_disp_ + x_off_; x--) {

      float c_min = bck[x + 1][0];
      for (int d = 1; d < max_disp_; d++)
        if (bck[x + 1][d] < c_min)
          c_min = bck[x + 1][d];

      bck[x][0] =
          acc[x][0] - c_min +
          std::min(bck[x + 1][0],
                   std::min(bck[x + 1][1] + static_cast<float>(smoothness_weak_),
                            c_min + static_cast<float>(smoothness_strong_)));
      for (int d = 1; d < max_disp_ - 1; d++)
        bck[x][d] =
            acc[x][d] - c_min +
            std::min(std::min(bck[x + 1][d],
                              bck[x + 1][d - 1] + static_cast<float>(smoothness_weak_)),
                     std::min(bck[x + 1][d + 1] + static_cast<float>(smoothness_weak_),
                              c_min + static_cast<float>(smoothness_strong_)));
      bck[x][max_disp_ - 1] =
          acc[x][max_disp_ - 1] - c_min +
          std::min(
              bck[x + 1][max_disp_ - 1],
              std::min(bck[x + 1][max_disp_ - 2] + static_cast<float>(smoothness_weak_),
                       c_min + static_cast<float>(smoothness_strong_)));
    } // x

    // last scan
    for (int x = x_off_ + max_disp_ + 1; x < width_; x++) {
      float c_min = std::numeric_limits<float>::max();
      short int dbest = 0;

      for (int d = 0; d < max_disp_; d++) {
        acc[x][d] = fwd[x][d] + bck[x][d];
        if (acc[x][d] < c_min) {
          c_min = acc[x][d];
          dbest = static_cast<short int>(d);
        }
      }

      if (ratio_filter_ > 0)
        dbest = doStereoRatioFilter(acc[x], dbest, c_min, ratio_filter_, max_disp_);
      if (peak_filter_ > 0)
        dbest = doStereoPeakFilter(acc[x], dbest, peak_filter_, max_disp_);

      disp_map_[y * width_ + x] = static_cast<short int>(dbest * 16);

      // subpixel refinement
      if (dbest > 0 && dbest < max_disp_ - 1)
        disp_map_[y * width_ + x] = computeStereoSubpixel(
            dbest, acc[x][dbest - 1], acc[x][dbest], acc[x][dbest + 1]);
    } // x
  }   // y

  for (int x = 0; x < width_; x++) {
    delete[] fwd[x];
    delete[] bck[x];
    delete[] acc[x];
  }
  delete[] fwd;
  delete[] bck;
  delete[] acc;

  delete[] wl;
  delete[] ds;
}
