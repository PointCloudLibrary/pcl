/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */

#ifndef PCL_TRACKING_IMPL_PYRAMIDAL_KLT_HPP
#define PCL_TRACKING_IMPL_PYRAMIDAL_KLT_HPP

#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/common/utils.h>

namespace pcl {
namespace tracking {
///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
inline void
PyramidalKLTTracker<PointInT, IntensityT>::setTrackingWindowSize(int width, int height)
{
  track_width_ = width;
  track_height_ = height;
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
inline void
PyramidalKLTTracker<PointInT, IntensityT>::setPointsToTrack(
    const pcl::PointCloud<pcl::PointUV>::ConstPtr& keypoints)
{
  if (keypoints->size() <= keypoints_nbr_)
    keypoints_ = keypoints;
  else {
    pcl::PointCloud<pcl::PointUV>::Ptr p(new pcl::PointCloud<pcl::PointUV>);
    p->reserve(keypoints_nbr_);
    for (std::size_t i = 0; i < keypoints_nbr_; ++i)
      p->push_back((*keypoints)[i]);
    keypoints_ = p;
  }

  keypoints_status_.reset(new std::vector<int>);
  keypoints_status_->resize(keypoints_->size(), 0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
inline void
PyramidalKLTTracker<PointInT, IntensityT>::setPointsToTrack(
    const pcl::PointIndicesConstPtr& points)
{
  assert((input_ || ref_) && "[PyramidalKLTTracker] CALL setInputCloud FIRST!");

  pcl::PointCloud<pcl::PointUV>::Ptr keypoints(new pcl::PointCloud<pcl::PointUV>);
  keypoints->reserve(keypoints_nbr_);
  for (std::size_t i = 0; i < keypoints_nbr_; ++i) {
    pcl::PointUV uv;
    uv.u = points->indices[i] % input_->width;
    uv.v = points->indices[i] / input_->width;
    keypoints->push_back(uv);
  }
  setPointsToTrack(keypoints);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
bool
PyramidalKLTTracker<PointInT, IntensityT>::initCompute()
{
  // std::cout << ">>> [PyramidalKLTTracker::initCompute]" << std::endl;
  if (!PCLBase<PointInT>::initCompute()) {
    PCL_ERROR("[%s::initCompute] PCLBase::Init failed.\n", tracker_name_.c_str());
    return (false);
  }

  if (!input_->isOrganized()) {
    PCL_ERROR(
        "[pcl::tracking::%s::initCompute] Need an organized point cloud to proceed!\n",
        tracker_name_.c_str());
    return (false);
  }

  if (!keypoints_ || keypoints_->empty()) {
    PCL_ERROR("[pcl::tracking::%s::initCompute] No keypoints aborting!\n",
              tracker_name_.c_str());
    return (false);
  }

  // This is the first call
  if (!ref_) {
    ref_ = input_;
    // std::cout << "First run!!!" << std::endl;

    if ((track_height_ * track_width_) % 2 == 0) {
      PCL_ERROR(
          "[pcl::tracking::%s::initCompute] Tracking window (%dx%d) must be odd!\n",
          tracker_name_.c_str(),
          track_width_,
          track_height_);
      return (false);
    }

    if (track_height_ < 3 || track_width_ < 3) {
      PCL_ERROR(
          "[pcl::tracking::%s::initCompute] Tracking window (%dx%d) must be >= 3x3!\n",
          tracker_name_.c_str(),
          track_width_,
          track_height_);
      return (false);
    }

    track_width_2_ = track_width_ / 2;
    track_height_2_ = track_height_ / 2;

    if (nb_levels_ < 2) {
      PCL_ERROR("[pcl::tracking::%s::initCompute] Number of pyramid levels should be "
                "at least 2!\n",
                tracker_name_.c_str());
      return (false);
    }

    if (nb_levels_ > 5) {
      PCL_ERROR("[pcl::tracking::%s::initCompute] Number of pyramid levels should not "
                "exceed 5!\n",
                tracker_name_.c_str());
      return (false);
    }

    computePyramids(ref_, ref_pyramid_, pcl::BORDER_REFLECT_101);
    return (true);
  }

  initialized_ = true;

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::derivatives(const FloatImage& src,
                                                       FloatImage& grad_x,
                                                       FloatImage& grad_y) const
{
  // std::cout << ">>> derivatives" << std::endl;
  ////////////////////////////////////////////////////////
  // Use Shcarr operator to compute derivatives.        //
  // Vertical kernel +3 +10 +3 = [1 0 -1]T * [3 10 3]   //
  //                  0   0  0                          //
  //                 -3 -10 -3                          //
  // Horizontal kernel  +3 0  -3 = [3 10 3]T * [1 0 -1] //
  //                   +10 0 -10                        //
  //                    +3 0  -3                        //
  ////////////////////////////////////////////////////////
  if (grad_x.size() != src.size() || grad_x.width != src.width ||
      grad_x.height != src.height)
    grad_x = FloatImage(src.width, src.height);
  if (grad_y.size() != src.size() || grad_y.width != src.width ||
      grad_y.height != src.height)
    grad_y = FloatImage(src.width, src.height);

  int height = src.height, width = src.width;
  float* row0 = new float[src.width + 2];
  float* row1 = new float[src.width + 2];
  float* trow0 = row0;
  ++trow0;
  float* trow1 = row1;
  ++trow1;
  const float* src_ptr = &(src[0]);

  for (int y = 0; y < height; y++) {
    const float* srow0 = src_ptr + (y > 0 ? y - 1 : height > 1 ? 1 : 0) * width;
    const float* srow1 = src_ptr + y * width;
    const float* srow2 =
        src_ptr + (y < height - 1 ? y + 1 : height > 1 ? height - 2 : 0) * width;
    float* grad_x_row = &(grad_x[y * width]);
    float* grad_y_row = &(grad_y[y * width]);

    // do vertical convolution
    for (int x = 0; x < width; x++) {
      trow0[x] = (srow0[x] + srow2[x]) * 3 + srow1[x] * 10;
      trow1[x] = srow2[x] - srow0[x];
    }

    // make border
    int x0 = width > 1 ? 1 : 0, x1 = width > 1 ? width - 2 : 0;
    trow0[-1] = trow0[x0];
    trow0[width] = trow0[x1];
    trow1[-1] = trow1[x0];
    trow1[width] = trow1[x1];

    // do horizontal convolution and store results
    for (int x = 0; x < width; x++) {
      grad_x_row[x] = trow0[x + 1] - trow0[x - 1];
      grad_y_row[x] = (trow1[x + 1] + trow1[x - 1]) * 3 + trow1[x] * 10;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::downsample(const FloatImageConstPtr& input,
                                                      FloatImageConstPtr& output) const
{
  FloatImage smoothed(input->width, input->height);
  convolve(input, smoothed);

  int width = (smoothed.width + 1) / 2;
  int height = (smoothed.height + 1) / 2;
  std::vector<int> ii(width);
  for (int i = 0; i < width; ++i)
    ii[i] = 2 * i;

  FloatImagePtr down(new FloatImage(width, height));
  // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(down, height, output, smoothed, width) \
  firstprivate(ii) \
  num_threads(threads_)
  // clang-format on	
  for (int j = 0; j < height; ++j) {
    int jj = 2 * j;
    for (int i = 0; i < width; ++i)
      (*down)(i, j) = smoothed(ii[i], jj);
  }

  output = down;
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::downsample(
    const FloatImageConstPtr& input,
    FloatImageConstPtr& output,
    FloatImageConstPtr& output_grad_x,
    FloatImageConstPtr& output_grad_y) const
{
  downsample(input, output);
  FloatImagePtr grad_x(new FloatImage(input->width, input->height));
  FloatImagePtr grad_y(new FloatImage(input->width, input->height));
  derivatives(*output, *grad_x, *grad_y);
  output_grad_x = grad_x;
  output_grad_y = grad_y;
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::convolve(
    const FloatImageConstPtr& input, FloatImage& output) const
{
  FloatImagePtr tmp(new FloatImage(input->width, input->height));
  convolveRows(input, *tmp);
  convolveCols(tmp, output);
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::convolveRows(
    const FloatImageConstPtr& input, FloatImage& output) const
{
  int width = input->width;
  int height = input->height;
  int last = input->width - kernel_size_2_;
  int w = last - 1;

  // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(input, height, last, output, w, width) \
  num_threads(threads_)
  // clang-format on
  for (int j = 0; j < height; ++j) {
    for (int i = kernel_size_2_; i < last; ++i) {
      double result = 0;
      for (int k = kernel_last_, l = i - kernel_size_2_; k > -1; --k, ++l)
        result += kernel_[k] * (*input)(l, j);

      output(i, j) = static_cast<float>(result);
    }

    for (int i = last; i < width; ++i)
      output(i, j) = output(w, j);

    for (int i = 0; i < kernel_size_2_; ++i)
      output(i, j) = output(kernel_size_2_, j);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::convolveCols(const FloatImageConstPtr& input,
                                                        FloatImage& output) const
{
  output = FloatImage(input->width, input->height);

  int width = input->width;
  int height = input->height;
  int last = input->height - kernel_size_2_;
  int h = last - 1;

  // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(input, h, height, last, output, width) \
  num_threads(threads_)
  // clang-format on
  for (int i = 0; i < width; ++i) {
    for (int j = kernel_size_2_; j < last; ++j) {
      double result = 0;
      for (int k = kernel_last_, l = j - kernel_size_2_; k > -1; --k, ++l)
        result += kernel_[k] * (*input)(i, l);
      output(i, j) = static_cast<float>(result);
    }

    for (int j = last; j < height; ++j)
      output(i, j) = output(i, h);

    for (int j = 0; j < kernel_size_2_; ++j)
      output(i, j) = output(i, kernel_size_2_);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::computePyramids(
    const PointCloudInConstPtr& input,
    std::vector<FloatImageConstPtr>& pyramid,
    pcl::InterpolationType border_type) const
{
  int step = 3;
  pyramid.resize(step * nb_levels_);

  FloatImageConstPtr previous;
  FloatImagePtr tmp(new FloatImage(input->width, input->height));
  // clang-format off
#pragma omp parallel for \
  default(none) \
  shared(input, tmp) \
  num_threads(threads_)
  // clang-format on
  for (int i = 0; i < static_cast<int>(input->size()); ++i)
    (*tmp)[i] = intensity_((*input)[i]);
  previous = tmp;

  FloatImagePtr img(new FloatImage(previous->width + 2 * track_width_,
                                   previous->height + 2 * track_height_));

  pcl::copyPointCloud(*tmp,
                      *img,
                      track_height_,
                      track_height_,
                      track_width_,
                      track_width_,
                      border_type,
                      0.f);
  pyramid[0] = img;

  // compute first level gradients
  FloatImagePtr g_x(new FloatImage(input->width, input->height));
  FloatImagePtr g_y(new FloatImage(input->width, input->height));
  derivatives(*img, *g_x, *g_y);
  // copy to bigger clouds
  FloatImagePtr grad_x(new FloatImage(previous->width + 2 * track_width_,
                                      previous->height + 2 * track_height_));
  pcl::copyPointCloud(*g_x,
                      *grad_x,
                      track_height_,
                      track_height_,
                      track_width_,
                      track_width_,
                      pcl::BORDER_CONSTANT,
                      0.f);
  pyramid[1] = grad_x;

  FloatImagePtr grad_y(new FloatImage(previous->width + 2 * track_width_,
                                      previous->height + 2 * track_height_));
  pcl::copyPointCloud(*g_y,
                      *grad_y,
                      track_height_,
                      track_height_,
                      track_width_,
                      track_width_,
                      pcl::BORDER_CONSTANT,
                      0.f);
  pyramid[2] = grad_y;

  for (int level = 1; level < nb_levels_; ++level) {
    // compute current level and current level gradients
    FloatImageConstPtr current;
    FloatImageConstPtr g_x;
    FloatImageConstPtr g_y;
    downsample(previous, current, g_x, g_y);
    // copy to bigger clouds
    FloatImagePtr image(new FloatImage(current->width + 2 * track_width_,
                                       current->height + 2 * track_height_));
    pcl::copyPointCloud(*current,
                        *image,
                        track_height_,
                        track_height_,
                        track_width_,
                        track_width_,
                        border_type,
                        0.f);
    pyramid[level * step] = image;
    FloatImagePtr gradx(
        new FloatImage(g_x->width + 2 * track_width_, g_x->height + 2 * track_height_));
    pcl::copyPointCloud(*g_x,
                        *gradx,
                        track_height_,
                        track_height_,
                        track_width_,
                        track_width_,
                        pcl::BORDER_CONSTANT,
                        0.f);
    pyramid[level * step + 1] = gradx;
    FloatImagePtr grady(
        new FloatImage(g_y->width + 2 * track_width_, g_y->height + 2 * track_height_));
    pcl::copyPointCloud(*g_y,
                        *grady,
                        track_height_,
                        track_height_,
                        track_width_,
                        track_width_,
                        pcl::BORDER_CONSTANT,
                        0.f);
    pyramid[level * step + 2] = grady;
    // set the new level
    previous = current;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::spatialGradient(
    const FloatImage& img,
    const FloatImage& grad_x,
    const FloatImage& grad_y,
    const Eigen::Array2i& location,
    const Eigen::Array4f& weight,
    Eigen::ArrayXXf& win,
    Eigen::ArrayXXf& grad_x_win,
    Eigen::ArrayXXf& grad_y_win,
    Eigen::Array3f& covariance) const
{
  const int step = img.width;
  covariance.setZero();

  for (int y = 0; y < track_height_; y++) {
    const float* img_ptr = &(img[0]) + (y + location[1]) * step + location[0];
    const float* grad_x_ptr = &(grad_x[0]) + (y + location[1]) * step + location[0];
    const float* grad_y_ptr = &(grad_y[0]) + (y + location[1]) * step + location[0];

    float* win_ptr = win.data() + y * win.cols();
    float* grad_x_win_ptr = grad_x_win.data() + y * grad_x_win.cols();
    float* grad_y_win_ptr = grad_y_win.data() + y * grad_y_win.cols();

    for (int x = 0; x < track_width_; ++x, ++grad_x_ptr, ++grad_y_ptr) {
      *win_ptr++ = img_ptr[x] * weight[0] + img_ptr[x + 1] * weight[1] +
                   img_ptr[x + step] * weight[2] + img_ptr[x + step + 1] * weight[3];
      float ixval = grad_x_ptr[0] * weight[0] + grad_x_ptr[1] * weight[1] +
                    grad_x_ptr[step] * weight[2] + grad_x_ptr[step + 1] * weight[3];
      float iyval = grad_y_ptr[0] * weight[0] + grad_y_ptr[1] * weight[1] +
                    grad_y_ptr[step] * weight[2] + grad_y_ptr[step + 1] * weight[3];
      //!!! store those
      *grad_x_win_ptr++ = ixval;
      *grad_y_win_ptr++ = iyval;
      // covariance components
      covariance[0] += ixval * ixval;
      covariance[1] += ixval * iyval;
      covariance[2] += iyval * iyval;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::mismatchVector(
    const Eigen::ArrayXXf& prev,
    const Eigen::ArrayXXf& prev_grad_x,
    const Eigen::ArrayXXf& prev_grad_y,
    const FloatImage& next,
    const Eigen::Array2i& location,
    const Eigen::Array4f& weight,
    Eigen::Array2f& b) const
{
  const int step = next.width;
  b.setZero();
  for (int y = 0; y < track_height_; y++) {
    const float* next_ptr = &(next[0]) + (y + location[1]) * step + location[0];
    const float* prev_ptr = prev.data() + y * prev.cols();
    const float* prev_grad_x_ptr = prev_grad_x.data() + y * prev_grad_x.cols();
    const float* prev_grad_y_ptr = prev_grad_y.data() + y * prev_grad_y.cols();

    for (int x = 0; x < track_width_; ++x, ++prev_grad_y_ptr, ++prev_grad_x_ptr) {
      float diff = next_ptr[x] * weight[0] + next_ptr[x + 1] * weight[1] +
                   next_ptr[x + step] * weight[2] + next_ptr[x + step + 1] * weight[3] -
                   prev_ptr[x];
      b[0] += *prev_grad_x_ptr * diff;
      b[1] += *prev_grad_y_ptr * diff;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::track(
    const PointCloudInConstPtr& prev_input,
    const PointCloudInConstPtr& input,
    const std::vector<FloatImageConstPtr>& prev_pyramid,
    const std::vector<FloatImageConstPtr>& pyramid,
    const pcl::PointCloud<pcl::PointUV>::ConstPtr& prev_keypoints,
    pcl::PointCloud<pcl::PointUV>::Ptr& keypoints,
    std::vector<int>& status,
    Eigen::Affine3f& motion) const
{
  std::vector<Eigen::Array2f, Eigen::aligned_allocator<Eigen::Array2f>> next_pts(
      prev_keypoints->size());
  Eigen::Array2f half_win((track_width_ - 1) * 0.5f, (track_height_ - 1) * 0.5f);
  pcl::TransformationFromCorrespondences transformation_computer;
  const int nb_points = prev_keypoints->size();
  for (int level = nb_levels_ - 1; level >= 0; --level) {
    const FloatImage& prev = *(prev_pyramid[level * 3]);
    const FloatImage& next = *(pyramid[level * 3]);
    const FloatImage& grad_x = *(prev_pyramid[level * 3 + 1]);
    const FloatImage& grad_y = *(prev_pyramid[level * 3 + 2]);

    Eigen::ArrayXXf prev_win(track_height_, track_width_);
    Eigen::ArrayXXf grad_x_win(track_height_, track_width_);
    Eigen::ArrayXXf grad_y_win(track_height_, track_width_);
    float ratio(1. / (1 << level));
    for (int ptidx = 0; ptidx < nb_points; ptidx++) {
      Eigen::Array2f prev_pt((*prev_keypoints)[ptidx].u * ratio,
                             (*prev_keypoints)[ptidx].v * ratio);
      Eigen::Array2f next_pt;
      if (level == nb_levels_ - 1)
        next_pt = prev_pt;
      else
        next_pt = next_pts[ptidx] * 2.f;

      next_pts[ptidx] = next_pt;

      Eigen::Array2i iprev_point;
      prev_pt -= half_win;
      iprev_point[0] = std::floor(prev_pt[0]);
      iprev_point[1] = std::floor(prev_pt[1]);

      if (iprev_point[0] < -track_width_ ||
          (std::uint32_t)iprev_point[0] >= grad_x.width ||
          iprev_point[1] < -track_height_ ||
          (std::uint32_t)iprev_point[1] >= grad_y.height) {
        if (level == 0)
          status[ptidx] = -1;
        continue;
      }

      float a = prev_pt[0] - iprev_point[0];
      float b = prev_pt[1] - iprev_point[1];
      Eigen::Array4f weight;
      weight[0] = (1.f - a) * (1.f - b);
      weight[1] = a * (1.f - b);
      weight[2] = (1.f - a) * b;
      weight[3] = 1 - weight[0] - weight[1] - weight[2];

      Eigen::Array3f covar = Eigen::Array3f::Zero();
      spatialGradient(prev,
                      grad_x,
                      grad_y,
                      iprev_point,
                      weight,
                      prev_win,
                      grad_x_win,
                      grad_y_win,
                      covar);

      float det = covar[0] * covar[2] - covar[1] * covar[1];
      float min_eigenvalue = (covar[2] + covar[0] -
                              std::sqrt((covar[0] - covar[2]) * (covar[0] - covar[2]) +
                                        4.f * covar[1] * covar[1])) /
                             2.f;

      if (min_eigenvalue < min_eigenvalue_threshold_ ||
          det < std::numeric_limits<float>::epsilon()) {
        status[ptidx] = -2;
        continue;
      }

      det = 1.f / det;
      next_pt -= half_win;

      Eigen::Array2f prev_delta(0, 0);
      for (unsigned int j = 0; j < max_iterations_; j++) {
        Eigen::Array2i inext_pt = next_pt.floor().cast<int>();

        if (inext_pt[0] < -track_width_ || (std::uint32_t)inext_pt[0] >= next.width ||
            inext_pt[1] < -track_height_ || (std::uint32_t)inext_pt[1] >= next.height) {
          if (level == 0)
            status[ptidx] = -1;
          break;
        }

        a = next_pt[0] - inext_pt[0];
        b = next_pt[1] - inext_pt[1];
        weight[0] = (1.f - a) * (1.f - b);
        weight[1] = a * (1.f - b);
        weight[2] = (1.f - a) * b;
        weight[3] = 1 - weight[0] - weight[1] - weight[2];
        // compute mismatch vector
        Eigen::Array2f beta = Eigen::Array2f::Zero();
        mismatchVector(prev_win, grad_x_win, grad_y_win, next, inext_pt, weight, beta);
        // optical flow resolution
        Eigen::Vector2f delta((covar[1] * beta[1] - covar[2] * beta[0]) * det,
                              (covar[1] * beta[0] - covar[0] * beta[1]) * det);
        // update position
        next_pt[0] += delta[0];
        next_pt[1] += delta[1];
        next_pts[ptidx] = next_pt + half_win;

        if (delta.squaredNorm() <= epsilon_)
          break;

        if (j > 0 && std::abs(delta[0] + prev_delta[0]) < 0.01 &&
            std::abs(delta[1] + prev_delta[1]) < 0.01) {
          next_pts[ptidx][0] -= delta[0] * 0.5f;
          next_pts[ptidx][1] -= delta[1] * 0.5f;
          break;
        }
        // update delta
        prev_delta = delta;
      }

      // update tracked points
      if (level == 0 && !status[ptidx]) {
        Eigen::Array2f next_point = next_pts[ptidx] - half_win;
        Eigen::Array2i inext_point;

        inext_point[0] = std::floor(next_point[0]);
        inext_point[1] = std::floor(next_point[1]);

        if (inext_point[0] < -track_width_ ||
            (std::uint32_t)inext_point[0] >= next.width ||
            inext_point[1] < -track_height_ ||
            (std::uint32_t)inext_point[1] >= next.height) {
          status[ptidx] = -1;
          continue;
        }
        // insert valid keypoint
        pcl::PointUV n;
        n.u = next_pts[ptidx][0];
        n.v = next_pts[ptidx][1];
        keypoints->push_back(n);
        // add points pair to compute transformation
        inext_point[0] = std::floor(next_pts[ptidx][0]);
        inext_point[1] = std::floor(next_pts[ptidx][1]);
        iprev_point[0] = std::floor((*prev_keypoints)[ptidx].u);
        iprev_point[1] = std::floor((*prev_keypoints)[ptidx].v);
        const PointInT& prev_pt =
            (*prev_input)[iprev_point[1] * prev_input->width + iprev_point[0]];
        const PointInT& next_pt =
            (*input)[inext_point[1] * input->width + inext_point[0]];
        transformation_computer.add(
            prev_pt.getVector3fMap(), next_pt.getVector3fMap(), 1.0);
      }
    }
  }
  motion = transformation_computer.getTransformation();
}

///////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename IntensityT>
void
PyramidalKLTTracker<PointInT, IntensityT>::computeTracking()
{
  if (!initialized_)
    return;

  std::vector<FloatImageConstPtr> pyramid;
  computePyramids(input_, pyramid, pcl::BORDER_REFLECT_101);
  pcl::PointCloud<pcl::PointUV>::Ptr keypoints(new pcl::PointCloud<pcl::PointUV>);
  keypoints->reserve(keypoints_->size());
  std::vector<int> status(keypoints_->size(), 0);
  track(ref_, input_, ref_pyramid_, pyramid, keypoints_, keypoints, status, motion_);
  // swap reference and input
  ref_ = input_;
  ref_pyramid_ = pyramid;
  keypoints_ = keypoints;
  *keypoints_status_ = status;
}

} // namespace tracking
} // namespace pcl

#endif
