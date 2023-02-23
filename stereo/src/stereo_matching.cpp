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

#include <pcl/console/print.h> // for PCL_ERROR

//////////////////////////////////////////////////////////////////////////////
pcl::StereoMatching::StereoMatching()
{
  disp_map_ = nullptr;
  disp_map_trg_ = nullptr;

  ref_img_ = nullptr;
  trg_img_ = nullptr;

  pp_ref_img_ = nullptr;
  pp_trg_img_ = nullptr;

  width_ = -1;
  height_ = -1;

  max_disp_ = -1;
  x_off_ = 0;

  ratio_filter_ = 0;
  peak_filter_ = 0;

  is_pre_proc_ = false;
  is_lr_check_ = false;
  lr_check_th_ = 1;
}

//////////////////////////////////////////////////////////////////////////////
pcl::StereoMatching::~StereoMatching()
{
  delete[] disp_map_;
  delete[] disp_map_trg_;

  delete[] ref_img_;
  delete[] trg_img_;
  delete[] pp_ref_img_;
  delete[] pp_trg_img_;
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::StereoMatching::medianFilter(int radius)
{

  // TODO: do median filter
  int side = radius * 2 + 1;

  auto* out = new short int[width_ * height_]{};

  auto* v = new short int[side * side];

  for (int y = radius; y < height_ - radius; y++) {
    for (int x = radius; x < width_ - radius; x++) {

      if (disp_map_[y * width_ + x] <= 0)
        out[y * width_ + x] = disp_map_[y * width_ + x];
      else {

        int n = 0;
        for (int j = -radius; j <= radius; j++) {
          for (int i = -radius; i <= radius; i++) {
            if (disp_map_[(y + j) * width_ + x + i] > 0) {
              v[n] = disp_map_[(y + j) * width_ + x + i];
              n++;
            }
          }
        }

        std::sort(v, v + n);
        out[y * width_ + x] = v[n / 2];
      }
    }
  }

  short int* temp_ptr = out;
  out = disp_map_;
  disp_map_ = temp_ptr;

  delete[] out;
  delete[] v;
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::StereoMatching::getVisualMap(pcl::PointCloud<pcl::RGB>::Ptr vMap)
{
  if (static_cast<int>(vMap->width) != width_ ||
      static_cast<int>(vMap->height) != height_) {
    vMap->resize(width_ * height_);
    vMap->width = width_;
    vMap->height = height_;
  }

  if (vMap->is_dense)
    vMap->is_dense = false;

  pcl::RGB invalid_val;
  invalid_val.r = 0;
  invalid_val.g = 255;
  invalid_val.b = 0;

  float scale = 255.0f / (16.0f * static_cast<float>(max_disp_));

  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      if (disp_map_[y * width_ + x] <= 0) {
        vMap->at(x, y) = invalid_val;
      }
      else {
        auto val =
            static_cast<unsigned char>(std::floor(scale * disp_map_[y * width_ + x]));
        vMap->at(x, y).r = val;
        vMap->at(x, y).g = val;
        vMap->at(x, y).b = val;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::StereoMatching::leftRightCheck()
{
  short int p1, p2, p2i;

  for (int y = 0; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      if (disp_map_[y * width_ + x] > 0) {
        p1 = disp_map_[y * width_ + x] / 16;

        p2i = static_cast<short int>(x - p1 - x_off_);

        if (p2i >= 0) {
          p2 = disp_map_trg_[y * width_ + p2i] / 16;

          if (std::abs(p1 - p2) > lr_check_th_)
            disp_map_[y * width_ + x] = -8;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::StereoMatching::getPointCloud(float u_c,
                                   float v_c,
                                   float focal,
                                   float baseline,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   pcl::PointCloud<pcl::RGB>::Ptr texture)
{
  // disp map has not been computed yet..
  if (disp_map_ == nullptr) {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: a disparity map has not "
              "been computed yet. The resulting cloud can not be computed..\n");

    return (false);
  }

  if (static_cast<int>(texture->width) != width_ ||
      static_cast<int>(texture->height) != height_) {
    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: the size of the texture "
              "cloud does not match that of the computed range map. The resulting "
              "cloud can not be computed..\n");
    return (false);
  }

  // cloud needs to be re-allocated
  if (static_cast<int>(cloud->width) != width_ ||
      static_cast<int>(cloud->height) != height_) {
    // cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(width_, height_) );
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  // Loop
  pcl::PointXYZRGB temp_point;
  /*pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.r = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.g = std::numeric_limits<unsigned char>::quiet_NaN();
  nan_point.b = std::numeric_limits<unsigned char>::quiet_NaN();*/

  // all disparities are multiplied by a constant equal to 16;
  // this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (int j = 0; j < height_; j++) {
    for (int i = 0; i < width_; i++) {
      if (disp_map_[j * width_ + i] > 0) {
        temp_point.z = (depth_scale) / (disp_map_[j * width_ + i]);
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;

        // temp_point.intensity = ( texture->at(j*width_+i).r +texture->at(j*width_+i).g
        // + texture->at(j*width_+i).b) / 3.0f;
        temp_point.r = texture->at(j * width_ + i).r;
        temp_point.g = texture->at(j * width_ + i).g;
        temp_point.b = texture->at(j * width_ + i).b;

        (*cloud)[j * width_ + i] = temp_point;
      }
      // adding NaN value
      else {
        temp_point.x = std::numeric_limits<float>::quiet_NaN();
        temp_point.y = std::numeric_limits<float>::quiet_NaN();
        temp_point.z = std::numeric_limits<float>::quiet_NaN();
        temp_point.r = texture->at(j * width_ + i).r;
        temp_point.g = texture->at(j * width_ + i).g;
        temp_point.b = texture->at(j * width_ + i).b;
        (*cloud)[j * width_ + i] = temp_point;
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
// const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
// pcl::StereoMatching::getPointCloud(float uC, float vC, float focal, float baseline)
bool
pcl::StereoMatching::getPointCloud(float u_c,
                                   float v_c,
                                   float focal,
                                   float baseline,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  // disp map has not been computed yet..
  if (disp_map_ == nullptr) {

    PCL_ERROR("[pcl::StereoMatching::getPointCloud] Error: a disparity map has not "
              "been computed yet. The resulting cloud can not be computed..\n");

    return false;
  }

  // cloud needs to be re-allocated
  if (static_cast<int>(cloud->width) != width_ ||
      static_cast<int>(cloud->height) != height_) {
    cloud->resize(width_ * height_);
    cloud->width = width_;
    cloud->height = height_;
    cloud->is_dense = false;
  }

  if (cloud->is_dense)
    cloud->is_dense = false;

  // Loop
  pcl::PointXYZ temp_point;
  pcl::PointXYZ nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  // nan_point.intensity = std::numeric_limits<float>::quiet_NaN();

  // all disparities are multiplied by a constant equal to 16;
  // this must be taken into account when computing z values
  float depth_scale = baseline * focal * 16.0f;

  for (int j = 0; j < height_; j++) {
    for (int i = 0; i < width_; i++) {
      if (disp_map_[j * width_ + i] > 0) {

        temp_point.z = depth_scale / disp_map_[j * width_ + i];
        temp_point.x = ((static_cast<float>(i) - u_c) * temp_point.z) / focal;
        temp_point.y = ((static_cast<float>(j) - v_c) * temp_point.z) / focal;
        // temp_point.intensity = 255;

        (*cloud)[j * width_ + i] = temp_point;
      }
      // adding NaN value
      else {
        (*cloud)[j * width_ + i] = nan_point;
      }
    }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
pcl::GrayStereoMatching::GrayStereoMatching() = default;

//////////////////////////////////////////////////////////////////////////////
pcl::GrayStereoMatching::~GrayStereoMatching() = default;

//////////////////////////////////////////////////////////////////////////////
void
pcl::GrayStereoMatching::preProcessing(unsigned char* img, unsigned char* pp_img)
{
  int radius = 4; // default value, could be exported
  int n = 2 * radius + 1;
  int area = n * n;
  int threshold = 31;

  int* v = new int[width_]{};

  for (int x = 0; x < n; x++)
    for (int y = 0; y < n; y++)
      v[x] += img[y * width_ + x];

  for (int x = radius + 1; x < width_ - radius; x++)
    for (int y = 0; y < n; y++)
      v[x + radius] += img[y * width_ + x + radius];

  for (int y = 0; y <= radius; y++)
    for (int x = 0; x < width_; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

  for (int y = radius + 1; y < height_ - radius; y++) {
    for (int x = 0; x <= radius; x++)
      pp_img[y * width_ + x] = img[y * width_ + x];

    int sum = 0;
    for (int x = 0; x < n; x++) {
      v[x] += img[(y + radius) * width_ + x] - img[(y - radius - 1) * width_ + x];
      sum += v[x];
    }

    for (int x = radius + 1; x < width_ - radius; x++) {
      v[x + radius] += img[(y + radius) * width_ + x + radius] -
                       img[(y - radius - 1) * width_ + x + radius];
      sum += v[x + radius] - v[x - radius - 1];

      auto temp = static_cast<short int>(img[y * width_ + x] - (sum / area));

      if (temp < -threshold)
        pp_img[y * width_ + x] = 0;
      else if (temp > threshold)
        pp_img[y * width_ + x] = static_cast<unsigned char>(threshold + threshold);
      else
        pp_img[y * width_ + x] = static_cast<unsigned char>(temp + threshold);
    }

    for (int x = width_ - radius; x < width_; x++) {
      pp_img[y * width_ + x] = img[y * width_ + x];
    }
  }

  for (int y = height_ - radius; y < height_; y++) {
    for (int x = 0; x < width_; x++) {
      pp_img[y * width_ + x] = img[y * width_ + x];
    }
  }

  delete[] v;
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::GrayStereoMatching::imgFlip(unsigned char*& img)
{
  auto* temp_row = new unsigned char[width_];

  for (int j = 0; j < height_; j++) {
    std::copy(img + j * width_, img + j * width_ + width_, temp_row);
    for (int i = 0; i < width_; i++) {
      img[j * width_ + i] = temp_row[width_ - 1 - i];
    }
  }

  delete[] temp_row;
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::GrayStereoMatching::compute(pcl::PointCloud<pcl::RGB>& ref,
                                 pcl::PointCloud<pcl::RGB>& trg)
{

  if (ref.width != trg.width || ref.height != trg.height) {

    PCL_ERROR("[pcl::GrayStereoMatching::compute] Error. The two input clouds have "
              "different sizes. Aborting..\n");
    return;
  }

  if ((ref_img_ != nullptr) && (width_ != static_cast<int>(ref.width) ||
                                height_ != static_cast<int>(ref.height))) {
    delete[] ref_img_;
    delete[] trg_img_;

    ref_img_ = nullptr;
    trg_img_ = nullptr;
  }

  if (ref_img_ == nullptr) {
    ref_img_ = new unsigned char[ref.width * ref.height];
    trg_img_ = new unsigned char[ref.width * ref.height];
  }

  float divider = 1.0f / 3.0f;
  for (unsigned int j = 0; j < ref.height; j++) {
    for (unsigned int i = 0; i < ref.width; i++) {
      ref_img_[j * ref.width + i] = static_cast<unsigned char>(
          static_cast<float>(ref[j * ref.width + i].r + ref[j * ref.width + i].g +
                             ref[j * ref.width + i].b) *
          divider);
      trg_img_[j * ref.width + i] = static_cast<unsigned char>(
          static_cast<float>(trg[j * ref.width + i].r + trg[j * ref.width + i].g +
                             trg[j * ref.width + i].b) *
          divider);
      // ref_img_[ j*ref.width + i] = ( ref(j,i).r + ref(j,i).g + ref(j,i).b) / 3;
      // trg_img_[ j*ref.width + i] = ( trg(j,i).r + trg(j,i).g + trg(j,i).b) / 3;
    }
  }

  compute(ref_img_, trg_img_, ref.width, ref.height);
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::GrayStereoMatching::compute(unsigned char* ref_img,
                                 unsigned char* trg_img,
                                 int width,
                                 int height)
{

  // Check that a suitable value of max_disp has been selected
  if (max_disp_ <= 0) {
    PCL_ERROR("[pcl::StereoMatching::compute] Error. A positive max_disparity value "
              "has not be correctly inserted. Aborting..\n");
    return;
  }

  if ((disp_map_ != nullptr) && (width_ != width || height_ != height)) {
    delete[] disp_map_;
    disp_map_ = nullptr;

    delete[] disp_map_trg_;
    disp_map_trg_ = nullptr;

    delete[] pp_ref_img_;
    delete[] pp_trg_img_;
    pp_ref_img_ = nullptr;
    pp_trg_img_ = nullptr;
  }

  if (disp_map_ == nullptr) {
    disp_map_ = new short int[width * height];

    width_ = width;
    height_ = height;
  }

  if (is_lr_check_ && disp_map_trg_ == nullptr) {
    disp_map_trg_ = new short int[width * height];
  }

  if (!is_lr_check_ && disp_map_trg_ != nullptr) {
    delete[] disp_map_trg_;
    disp_map_trg_ = nullptr;
  }

  if (is_pre_proc_ && pp_ref_img_ == nullptr) {
    pp_ref_img_ = new unsigned char[width_ * height_];
    pp_trg_img_ = new unsigned char[width_ * height_];
  }

  if (!is_pre_proc_ && pp_ref_img_ != nullptr) {
    delete[] pp_ref_img_;
    delete[] pp_trg_img_;
    pp_ref_img_ = nullptr;
    pp_trg_img_ = nullptr;
  }

  std::fill_n(disp_map_, height_ * width_, 0);

  if (is_pre_proc_) {
    preProcessing(ref_img, pp_ref_img_);
    preProcessing(trg_img, pp_trg_img_);
  }

  if (is_lr_check_) {

    if (is_pre_proc_) {
      imgFlip(pp_ref_img_);
      imgFlip(pp_trg_img_);

      compute_impl(pp_trg_img_, pp_ref_img_);

      imgFlip(pp_ref_img_);
      imgFlip(pp_trg_img_);
    }
    else {
      imgFlip(ref_img);
      imgFlip(trg_img);

      compute_impl(trg_img, ref_img);

      imgFlip(ref_img);
      imgFlip(trg_img);
    }

    for (int j = 0; j < height_; j++)
      for (int i = 0; i < width_; i++)
        disp_map_trg_[j * width_ + i] = disp_map_[j * width_ + width_ - 1 - i];
  }

  if (is_pre_proc_)
    compute_impl(pp_ref_img_, pp_trg_img_);
  else
    compute_impl(ref_img, trg_img);

  if (is_lr_check_) {
    leftRightCheck();
  }

  // at the end, x_offset (*16) needs to be added to all computed disparities,
  // so that each fixed point value of the disparity map represents the true disparity
  // value multiplied by 16
  for (int j = 0; j < height_; j++)
    for (int i = 0; i < width_; i++)
      if (disp_map_[j * width_ + i] > 0)
        disp_map_[j * width_ + i] += static_cast<short int>(x_off_ * 16);
}
