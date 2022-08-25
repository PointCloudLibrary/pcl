/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
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

#include <pcl/filters/pyramid.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h> // for squaredEuclideanDistance
#include <pcl/common/point_tests.h> // for isFinite

namespace pcl {
namespace filters {
template <>
inline void
Pyramid<pcl::PointXYZRGB>::compute(
    std::vector<Pyramid<pcl::PointXYZRGB>::PointCloudPtr>& output)
{
  std::cout << "PointXYZRGB" << std::endl;
  if (!initCompute()) {
    PCL_ERROR("[pcl::%s::compute] initCompute failed!\n", getClassName().c_str());
    return;
  }

  int kernel_rows = static_cast<int>(kernel_.rows());
  int kernel_cols = static_cast<int>(kernel_.cols());
  int kernel_center_x = kernel_cols / 2;
  int kernel_center_y = kernel_rows / 2;

  output.resize(levels_ + 1);
  output[0].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  *(output[0]) = *input_;

  if (input_->is_dense) {
    for (int l = 1; l <= levels_; ++l) {
      output[l].reset(new pcl::PointCloud<pcl::PointXYZRGB>(output[l - 1]->width / 2,
                                                            output[l - 1]->height / 2));
      const PointCloud<pcl::PointXYZRGB>& previous = *output[l - 1];
      PointCloud<pcl::PointXYZRGB>& next = *output[l];
#pragma omp parallel for default(none) shared(next, previous, kernel_rows, kernel_cols, kernel_center_x, kernel_center_y) num_threads(threads_)
      for (int i = 0; i < static_cast<int>(next.height); ++i) {  // rows
        for (int j = 0; j < static_cast<int>(next.width); ++j) {  // columns
          float r = 0, g = 0, b = 0;
          for (int m = 0; m < kernel_rows; ++m) // kernel rows
          {
            int mm = kernel_rows - 1 - m;         // row index of flipped kernel
            for (int n = 0; n < kernel_cols; ++n) // kernel columns
            {
              int nn = kernel_cols - 1 - n; // column index of flipped kernel
              // index of input signal, used for checking boundary
              int ii = 2 * i + (m - kernel_center_y);
              int jj = 2 * j + (n - kernel_center_x);

              // ignore input samples which are out of bound
              if (ii < 0)
                ii = 0;
              if (ii >= static_cast<int>(previous.height))
                ii = previous.height - 1;
              if (jj < 0)
                jj = 0;
              if (jj >= static_cast<int>(previous.width))
                jj = previous.width - 1;
              next.at(j, i).x += previous.at(jj, ii).x * kernel_(mm, nn);
              next.at(j, i).y += previous.at(jj, ii).y * kernel_(mm, nn);
              next.at(j, i).z += previous.at(jj, ii).z * kernel_(mm, nn);
              b += previous.at(jj, ii).b * kernel_(mm, nn);
              g += previous.at(jj, ii).g * kernel_(mm, nn);
              r += previous.at(jj, ii).r * kernel_(mm, nn);
            }
          }
          next.at(j, i).b = static_cast<std::uint8_t>(b);
          next.at(j, i).g = static_cast<std::uint8_t>(g);
          next.at(j, i).r = static_cast<std::uint8_t>(r);
        }
      }
    }
  }
  else {
    for (int l = 1; l <= levels_; ++l) {
      output[l].reset(new pcl::PointCloud<pcl::PointXYZRGB>(output[l - 1]->width / 2,
                                                            output[l - 1]->height / 2));
      const PointCloud<pcl::PointXYZRGB>& previous = *output[l - 1];
      PointCloud<pcl::PointXYZRGB>& next = *output[l];
#pragma omp parallel for default(none) shared(next, previous, kernel_rows, kernel_cols, kernel_center_x, kernel_center_y) num_threads(threads_)
      for (int i = 0; i < static_cast<int>(next.height); ++i) {  // rows
        for (int j = 0; j < static_cast<int>(next.width); ++j) {  // columns
          float weight = 0;
          float r = 0, g = 0, b = 0;
          for (int m = 0; m < kernel_rows; ++m) {
            int mm = kernel_rows - 1 - m;
            for (int n = 0; n < kernel_cols; ++n) {
              int nn = kernel_cols - 1 - n;
              int ii = 2 * i + (m - kernel_center_y);
              int jj = 2 * j + (n - kernel_center_x);
              if (ii < 0)
                ii = 0;
              if (ii >= static_cast<int>(previous.height))
                ii = previous.height - 1;
              if (jj < 0)
                jj = 0;
              if (jj >= static_cast<int>(previous.width))
                jj = previous.width - 1;
              if (!isFinite(previous.at(jj, ii)))
                continue;
              if (pcl::squaredEuclideanDistance(previous.at(2 * j, 2 * i),
                                                previous.at(jj, ii)) < threshold_) {
                next.at(j, i).x += previous.at(jj, ii).x * kernel_(mm, nn);
                next.at(j, i).y += previous.at(jj, ii).y * kernel_(mm, nn);
                next.at(j, i).z += previous.at(jj, ii).z * kernel_(mm, nn);
                b += previous.at(jj, ii).b * kernel_(mm, nn);
                g += previous.at(jj, ii).g * kernel_(mm, nn);
                r += previous.at(jj, ii).r * kernel_(mm, nn);
                weight += kernel_(mm, nn);
              }
            }
          }
          if (weight == 0)
            nullify(next.at(j, i));
          else {
            weight = 1.f / weight;
            r *= weight;
            g *= weight;
            b *= weight;
            next.at(j, i).x *= weight;
            next.at(j, i).y *= weight;
            next.at(j, i).z *= weight;
            next.at(j, i).b = static_cast<std::uint8_t>(b);
            next.at(j, i).g = static_cast<std::uint8_t>(g);
            next.at(j, i).r = static_cast<std::uint8_t>(r);
          }
        }
      }
    }
  }
}

template <>
inline void
Pyramid<pcl::PointXYZRGBA>::compute(
    std::vector<Pyramid<pcl::PointXYZRGBA>::PointCloudPtr>& output)
{
  std::cout << "PointXYZRGBA" << std::endl;
  if (!initCompute()) {
    PCL_ERROR("[pcl::%s::compute] initCompute failed!\n", getClassName().c_str());
    return;
  }

  int kernel_rows = static_cast<int>(kernel_.rows());
  int kernel_cols = static_cast<int>(kernel_.cols());
  int kernel_center_x = kernel_cols / 2;
  int kernel_center_y = kernel_rows / 2;

  output.resize(levels_ + 1);
  output[0].reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
  *(output[0]) = *input_;

  if (input_->is_dense) {
    for (int l = 1; l <= levels_; ++l) {
      output[l].reset(new pcl::PointCloud<pcl::PointXYZRGBA>(
          output[l - 1]->width / 2, output[l - 1]->height / 2));
      const PointCloud<pcl::PointXYZRGBA>& previous = *output[l - 1];
      PointCloud<pcl::PointXYZRGBA>& next = *output[l];
#pragma omp parallel for default(none) shared(next, previous, kernel_rows, kernel_cols, kernel_center_x, kernel_center_y) num_threads(threads_)
      for (int i = 0; i < static_cast<int>(next.height); ++i) {  // rows
        for (int j = 0; j < static_cast<int>(next.width); ++j) {  // columns
          float r = 0, g = 0, b = 0, a = 0;
          for (int m = 0; m < kernel_rows; ++m) // kernel rows
          {
            int mm = kernel_rows - 1 - m;         // row index of flipped kernel
            for (int n = 0; n < kernel_cols; ++n) // kernel columns
            {
              int nn = kernel_cols - 1 - n; // column index of flipped kernel
              // index of input signal, used for checking boundary
              int ii = 2 * i + (m - kernel_center_y);
              int jj = 2 * j + (n - kernel_center_x);

              // ignore input samples which are out of bound
              if (ii < 0)
                ii = 0;
              if (ii >= static_cast<int>(previous.height))
                ii = previous.height - 1;
              if (jj < 0)
                jj = 0;
              if (jj >= static_cast<int>(previous.width))
                jj = previous.width - 1;
              next.at(j, i).x += previous.at(jj, ii).x * kernel_(mm, nn);
              next.at(j, i).y += previous.at(jj, ii).y * kernel_(mm, nn);
              next.at(j, i).z += previous.at(jj, ii).z * kernel_(mm, nn);
              b += previous.at(jj, ii).b * kernel_(mm, nn);
              g += previous.at(jj, ii).g * kernel_(mm, nn);
              r += previous.at(jj, ii).r * kernel_(mm, nn);
              a += previous.at(jj, ii).a * kernel_(mm, nn);
            }
          }
          next.at(j, i).b = static_cast<std::uint8_t>(b);
          next.at(j, i).g = static_cast<std::uint8_t>(g);
          next.at(j, i).r = static_cast<std::uint8_t>(r);
          next.at(j, i).a = static_cast<std::uint8_t>(a);
        }
      }
    }
  }
  else {
    for (int l = 1; l <= levels_; ++l) {
      output[l].reset(new pcl::PointCloud<pcl::PointXYZRGBA>(
          output[l - 1]->width / 2, output[l - 1]->height / 2));
      const PointCloud<pcl::PointXYZRGBA>& previous = *output[l - 1];
      PointCloud<pcl::PointXYZRGBA>& next = *output[l];
#pragma omp parallel for default(none) shared(next, previous, kernel_rows, kernel_cols, kernel_center_x, kernel_center_y) num_threads(threads_)
      for (int i = 0; i < static_cast<int>(next.height); ++i) {  // rows
        for (int j = 0; j < static_cast<int>(next.width); ++j) {  // columns
          float weight = 0;
          float r = 0, g = 0, b = 0, a = 0;
          for (int m = 0; m < kernel_rows; ++m) {
            int mm = kernel_rows - 1 - m;
            for (int n = 0; n < kernel_cols; ++n) {
              int nn = kernel_cols - 1 - n;
              int ii = 2 * i + (m - kernel_center_y);
              int jj = 2 * j + (n - kernel_center_x);
              if (ii < 0)
                ii = 0;
              if (ii >= static_cast<int>(previous.height))
                ii = previous.height - 1;
              if (jj < 0)
                jj = 0;
              if (jj >= static_cast<int>(previous.width))
                jj = previous.width - 1;
              if (!isFinite(previous.at(jj, ii)))
                continue;
              if (pcl::squaredEuclideanDistance(previous.at(2 * j, 2 * i),
                                                previous.at(jj, ii)) < threshold_) {
                next.at(j, i).x += previous.at(jj, ii).x * kernel_(mm, nn);
                next.at(j, i).y += previous.at(jj, ii).y * kernel_(mm, nn);
                next.at(j, i).z += previous.at(jj, ii).z * kernel_(mm, nn);
                b += previous.at(jj, ii).b * kernel_(mm, nn);
                g += previous.at(jj, ii).g * kernel_(mm, nn);
                r += previous.at(jj, ii).r * kernel_(mm, nn);
                a += previous.at(jj, ii).a * kernel_(mm, nn);
                weight += kernel_(mm, nn);
              }
            }
          }
          if (weight == 0)
            nullify(next.at(j, i));
          else {
            weight = 1.f / weight;
            r *= weight;
            g *= weight;
            b *= weight;
            a *= weight;
            next.at(j, i).x *= weight;
            next.at(j, i).y *= weight;
            next.at(j, i).z *= weight;
            next.at(j, i).b = static_cast<std::uint8_t>(b);
            next.at(j, i).g = static_cast<std::uint8_t>(g);
            next.at(j, i).r = static_cast<std::uint8_t>(r);
            next.at(j, i).a = static_cast<std::uint8_t>(a);
          }
        }
      }
    }
  }
}

template <>
inline void
Pyramid<pcl::RGB>::nullify(pcl::RGB& p) const
{
  p.r = 0; p.g = 0; p.b = 0;
}

template <>
inline void
Pyramid<pcl::RGB>::compute(std::vector<Pyramid<pcl::RGB>::PointCloudPtr>& output)
{
  std::cout << "RGB" << std::endl;
  if (!initCompute()) {
    PCL_ERROR("[pcl::%s::compute] initCompute failed!\n", getClassName().c_str());
    return;
  }

  int kernel_rows = static_cast<int>(kernel_.rows());
  int kernel_cols = static_cast<int>(kernel_.cols());
  int kernel_center_x = kernel_cols / 2;
  int kernel_center_y = kernel_rows / 2;

  output.resize(levels_ + 1);
  output[0].reset(new pcl::PointCloud<pcl::RGB>);
  *(output[0]) = *input_;

  if (input_->is_dense) {
    for (int l = 1; l <= levels_; ++l) {
      output[l].reset(new pcl::PointCloud<pcl::RGB>(output[l - 1]->width / 2,
                                                    output[l - 1]->height / 2));
      const PointCloud<pcl::RGB>& previous = *output[l - 1];
      PointCloud<pcl::RGB>& next = *output[l];
#pragma omp parallel for default(none) shared(next, previous, kernel_rows, kernel_cols, kernel_center_x, kernel_center_y) num_threads(threads_)
      for (int i = 0; i < static_cast<int>(next.height); ++i) {  // rows
        for (int j = 0; j < static_cast<int>(next.width); ++j) {  // columns
          float r = 0, g = 0, b = 0;
          for (int m = 0; m < kernel_rows; ++m) {
            int mm = kernel_rows - 1 - m;
            for (int n = 0; n < kernel_cols; ++n) {
              int nn = kernel_cols - 1 - n;
              int ii = 2 * i + (m - kernel_center_y);
              int jj = 2 * j + (n - kernel_center_x);
              if (ii < 0)
                ii = 0;
              if (ii >= static_cast<int>(previous.height))
                ii = previous.height - 1;
              if (jj < 0)
                jj = 0;
              if (jj >= static_cast<int>(previous.width))
                jj = previous.width - 1;
              b += previous.at(jj, ii).b * kernel_(mm, nn);
              g += previous.at(jj, ii).g * kernel_(mm, nn);
              r += previous.at(jj, ii).r * kernel_(mm, nn);
            }
          }
          next.at(j, i).b = static_cast<std::uint8_t>(b);
          next.at(j, i).g = static_cast<std::uint8_t>(g);
          next.at(j, i).r = static_cast<std::uint8_t>(r);
        }
      }
    }
  }
  else {
    for (int l = 1; l <= levels_; ++l) {
      output[l].reset(new pcl::PointCloud<pcl::RGB>(output[l - 1]->width / 2,
                                                    output[l - 1]->height / 2));
      const PointCloud<pcl::RGB>& previous = *output[l - 1];
      PointCloud<pcl::RGB>& next = *output[l];
#pragma omp parallel for default(none) shared(next, previous, kernel_rows, kernel_cols, kernel_center_x, kernel_center_y) num_threads(threads_)
      for (int i = 0; i < static_cast<int>(next.height); ++i) {  // rows
        for (int j = 0; j < static_cast<int>(next.width); ++j) {  // columns
          float weight = 0;
          float r = 0, g = 0, b = 0;
          for (int m = 0; m < kernel_rows; ++m) {
            int mm = kernel_rows - 1 - m;
            for (int n = 0; n < kernel_cols; ++n) {
              int nn = kernel_cols - 1 - n;
              int ii = 2 * i + (m - kernel_center_y);
              int jj = 2 * j + (n - kernel_center_x);
              if (ii < 0)
                ii = 0;
              if (ii >= static_cast<int>(previous.height))
                ii = previous.height - 1;
              if (jj < 0)
                jj = 0;
              if (jj >= static_cast<int>(previous.width))
                jj = previous.width - 1;
              if (!isFinite(previous.at(jj, ii)))
                continue;
              /*if (pcl::squaredEuclideanDistance(previous.at(2 * j, 2 * i),
                                                previous.at(jj, ii)) < threshold_)*/ {
                b += previous.at(jj, ii).b * kernel_(mm, nn);
                g += previous.at(jj, ii).g * kernel_(mm, nn);
                r += previous.at(jj, ii).r * kernel_(mm, nn);
                weight += kernel_(mm, nn);
              }
            }
          }
          if (weight == 0)
            nullify(next.at(j, i));
          else {
            weight = 1.f / weight;
            r *= weight;
            g *= weight;
            b *= weight;
            next.at(j, i).b = static_cast<std::uint8_t>(b);
            next.at(j, i).g = static_cast<std::uint8_t>(g);
            next.at(j, i).r = static_cast<std::uint8_t>(r);
          }
        }
      }
    }
  }
}
} // namespace filters
} // namespace pcl
