/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2012-, Open Perception, Inc.
 * Copyright (c) 2004, Sylvain Paris and Francois Sillion

 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: fast_bilateral_omp.hpp 8381 2013-01-02 23:12:44Z sdmiller $
 *
 */
#ifndef PCL_FILTERS_IMPL_FAST_BILATERAL_OMP_HPP_
#define PCL_FILTERS_IMPL_FAST_BILATERAL_OMP_HPP_

#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/common/io.h>
#include <pcl/console/time.h>
#include <assert.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::FastBilateralFilterOMP<PointT>::setNumberOfThreads (unsigned int nr_threads)
{
  if (nr_threads == 0)
#ifdef _OPENMP
    threads_ = omp_get_num_procs();
#else
    threads_ = 1;
#endif
  else
    threads_ = nr_threads;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::FastBilateralFilterOMP<PointT>::applyFilter (PointCloud &output)
{
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::FastBilateralFilterOMP] Input cloud needs to be organized.\n");
    return;
  }

  copyPointCloud (*input_, output);
  float base_max = -std::numeric_limits<float>::max (),
        base_min = std::numeric_limits<float>::max ();
  bool found_finite = false;
  for (size_t x = 0; x < output.width; ++x)
  {
    for (size_t y = 0; y < output.height; ++y)
    {
      if (pcl_isfinite (output (x, y).z))
      {
        if (base_max < output (x, y).z)
          base_max = output (x, y).z;
        if (base_min > output (x, y).z)
          base_min = output (x, y).z;
        found_finite = true;
      }
    }
  }
  if (!found_finite)
  {
    PCL_WARN ("[pcl::FastBilateralFilterOMP] Given an empty cloud. Doing nothing.\n");
    return;
  }
#ifdef _OPENMP
#pragma omp parallel for num_threads (threads_)
#endif
  for (long int i = 0; i < static_cast<long int> (output.size ()); ++i)
    if (!pcl_isfinite (output.at(i).z))
      output.at(i).z = base_max;

  const float base_delta = base_max - base_min;

  const size_t padding_xy = 2;
  const size_t padding_z  = 2;

  const size_t small_width  = static_cast<size_t> (static_cast<float> (input_->width  - 1) / sigma_s_) + 1 + 2 * padding_xy;
  const size_t small_height = static_cast<size_t> (static_cast<float> (input_->height - 1) / sigma_s_) + 1 + 2 * padding_xy;
  const size_t small_depth  = static_cast<size_t> (base_delta / sigma_r_)   + 1 + 2 * padding_z;

  Array3D data (small_width, small_height, small_depth);
#ifdef _OPENMP
#pragma omp parallel for num_threads (threads_)
#endif
  for (long int i = 0; i < static_cast<long int> (small_width * small_height); ++i)
  {
    size_t small_x = static_cast<size_t> (i % small_width);
    size_t small_y = static_cast<size_t> (i / small_width);
    size_t start_x = static_cast<size_t>( 
        std::max ((static_cast<float> (small_x) - static_cast<float> (padding_xy) - 0.5f) * sigma_s_ + 1, 0.f));
    size_t end_x = static_cast<size_t>( 
      std::max ((static_cast<float> (small_x) - static_cast<float> (padding_xy) + 0.5f) * sigma_s_ + 1, 0.f));
    size_t start_y = static_cast<size_t>( 
      std::max ((static_cast<float> (small_y) - static_cast<float> (padding_xy) - 0.5f) * sigma_s_ + 1, 0.f));
    size_t end_y = static_cast<size_t>( 
      std::max ((static_cast<float> (small_y) - static_cast<float> (padding_xy) + 0.5f) * sigma_s_ + 1, 0.f));
    for (size_t x = start_x; x < end_x && x < input_->width; ++x)
    {
      for (size_t y = start_y; y < end_y && y < input_->height; ++y)
      {
        const float z = output (x,y).z - base_min;
        const size_t small_z = static_cast<size_t> (static_cast<float> (z) / sigma_r_ + 0.5f) + padding_z;
        Eigen::Vector2f& d = data (small_x, small_y, small_z);
        d[0] += output (x,y).z;
        d[1] += 1.0f;
      }
    }
  }

  std::vector<long int> offset (3);
  offset[0] = &(data (1,0,0)) - &(data (0,0,0));
  offset[1] = &(data (0,1,0)) - &(data (0,0,0));
  offset[2] = &(data (0,0,1)) - &(data (0,0,0));

  Array3D buffer (small_width, small_height, small_depth);
  
  for (size_t dim = 0; dim < 3; ++dim)
  {
    for (size_t n_iter = 0; n_iter < 2; ++n_iter)
    {
      Array3D* current_buffer = (n_iter % 2 == 1 ? &buffer : &data);
      Array3D* current_data =(n_iter % 2 == 1 ? &data : &buffer);
#ifdef _OPENMP
#pragma omp parallel for num_threads (threads_)
#endif
      for(long int i = 0; i < static_cast<long int> ((small_width - 2)*(small_height - 2)); ++i)
      {
        size_t x = static_cast<size_t> (i % (small_width - 2) + 1);
        size_t y = static_cast<size_t> (i / (small_width - 2) + 1);
        const long int off = offset[dim];
        Eigen::Vector2f* d_ptr = &(current_data->operator() (x,y,1));
        Eigen::Vector2f* b_ptr = &(current_buffer->operator() (x,y,1));

        for(size_t z = 1; z < small_depth - 1; ++z, ++d_ptr, ++b_ptr)
          *d_ptr = (*(b_ptr - off) + *(b_ptr + off) + 2.0 * (*b_ptr)) / 4.0;
      }
    }
  }
  // Note: this works because there are an even number of iterations. 
  // If there were an odd number, we would need to end with a:
  // std::swap (data, buffer);

  if (early_division_)
  {
    for (std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> >::iterator d = data.begin (); d != data.end (); ++d)
      *d /= ((*d)[0] != 0) ? (*d)[1] : 1;

#ifdef _OPENMP
#pragma omp parallel for num_threads (threads_)
#endif
    for (long int i = 0; i < static_cast<long int> (input_->size ()); ++i)
    {
      size_t x = static_cast<size_t> (i % input_->width);
      size_t y = static_cast<size_t> (i / input_->width);
      const float z = output (x,y).z - base_min;
      const Eigen::Vector2f D = data.trilinear_interpolation (static_cast<float> (x) / sigma_s_ + padding_xy,
                                                              static_cast<float> (y) / sigma_s_ + padding_xy,
                                                              z / sigma_r_ + padding_z);
      output(x,y).z = D[0];
    }
  }
  else
  {
#ifdef _OPENMP
#pragma omp parallel for num_threads (threads_)
#endif
    for (long i = 0; i < static_cast<long int> (input_->size ()); ++i)
    {
      size_t x = static_cast<size_t> (i % input_->width);
      size_t y = static_cast<size_t> (i / input_->width);
      const float z = output (x,y).z - base_min;
      const Eigen::Vector2f D = data.trilinear_interpolation (static_cast<float> (x) / sigma_s_ + padding_xy,
                                                              static_cast<float> (y) / sigma_s_ + padding_xy,
                                                              z / sigma_r_ + padding_z);
      output (x,y).z = D[0] / D[1];
    }
  }
}



#endif /* PCL_FILTERS_IMPL_FAST_BILATERAL_OMP_HPP_ */

