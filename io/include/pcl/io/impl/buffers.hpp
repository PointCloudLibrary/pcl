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

#ifndef PCL_IO_IMPL_BUFFERS_HPP
#define PCL_IO_IMPL_BUFFERS_HPP

#include <iostream>

#include <pcl/pcl_macros.h>


template <typename T>
struct buffer_traits
{
  static T invalid () { return 0; }
  static bool is_invalid (T value) { return value == invalid (); };
};

template <>
struct buffer_traits <float>
{
  static float invalid () { return std::numeric_limits<float>::quiet_NaN (); };
  static bool is_invalid (float value) { return std::isnan (value); };
};

template <>
struct buffer_traits <double>
{
  static double invalid () { return std::numeric_limits<double>::quiet_NaN (); };
  static bool is_invalid (double value) { return std::isnan (value); };
};


namespace pcl
{

namespace io
{

template <typename T>
Buffer<T>::Buffer (std::size_t size)
: size_ (size)
{
}

template <typename T>
Buffer<T>::~Buffer ()
= default;

template <typename T>
SingleBuffer<T>::SingleBuffer (std::size_t size)
: Buffer<T> (size)
, data_ (size, buffer_traits<T>::invalid ())
{
}

template <typename T>
SingleBuffer<T>::~SingleBuffer ()
= default;

template <typename T> T
SingleBuffer<T>::operator[] (std::size_t idx) const
{
  assert (idx < size_);
  return (data_[idx]);
}

template <typename T> void
SingleBuffer<T>::push (std::vector<T>& data)
{
  assert (data.size () == size_);
  std::lock_guard<std::mutex> lock (data_mutex_);
  data_.swap (data);
  data.clear ();
}

template <typename T>
MedianBuffer<T>::MedianBuffer (std::size_t size,
                               unsigned char window_size)
: Buffer<T> (size)
, window_size_ (window_size)
, midpoint_ (window_size_ / 2)
, data_current_idx_ (window_size_ - 1)
{
  assert (size_ > 0);
  assert (window_size_ > 0);

  data_.resize (window_size_);
  for (std::size_t i = 0; i < window_size_; ++i)
    data_[i].resize (size_, buffer_traits<T>::invalid ());

  data_argsort_indices_.resize (size_);
  for (std::size_t i = 0; i < size_; ++i)
  {
    data_argsort_indices_[i].resize (window_size_);
    for (std::size_t j = 0; j < window_size_; ++j)
      data_argsort_indices_[i][j] = j;
  }

  data_invalid_count_.resize (size_, window_size_);
}

template <typename T>
MedianBuffer<T>::~MedianBuffer ()
= default;

template <typename T> T
MedianBuffer<T>::operator[] (std::size_t idx) const
{
  assert (idx < size_);
  int midpoint = (window_size_ - data_invalid_count_[idx]) / 2;
  return (data_[data_argsort_indices_[idx][midpoint]][idx]);
}

template <typename T> void
MedianBuffer<T>::push (std::vector<T>& data)
{
  assert (data.size () == size_);
  std::lock_guard<std::mutex> lock (data_mutex_);

  if (++data_current_idx_ >= window_size_)
    data_current_idx_ = 0;

  // New data will replace the column with index data_current_idx_. Before
  // overwriting it, we go through all the new-old value pairs and update
  // data_argsort_indices_ to maintain sorted order.
  for (std::size_t i = 0; i < size_; ++i)
  {
    const T& new_value = data[i];
    const T& old_value = data_[data_current_idx_][i];
    bool new_is_invalid = buffer_traits<T>::is_invalid (new_value);
    bool old_is_invalid = buffer_traits<T>::is_invalid (old_value);
    if (compare (new_value, old_value) == 0)
      continue;
    std::vector<unsigned char>& argsort_indices = data_argsort_indices_[i];
    // Rewrite the argsort indices before or after the position where we insert
    // depending on the relation between the old and new values
    if (compare (new_value, old_value) == 1)
    {
      for (int j = 0; j < window_size_; ++j)
        if (argsort_indices[j] == data_current_idx_)
        {
          int k = j + 1;
          while (k < window_size_ && compare (new_value, data_[argsort_indices[k]][i]) == 1)
          {
            std::swap (argsort_indices[k - 1], argsort_indices[k]);
            ++k;
          }
          break;
        }
    }
    else
    {
      for (int j = window_size_ - 1; j >= 0; --j)
        if (argsort_indices[j] == data_current_idx_)
        {
          int k = j - 1;
          while (k >= 0 && compare (new_value, data_[argsort_indices[k]][i]) == -1)
          {
            std::swap (argsort_indices[k], argsort_indices[k + 1]);
            --k;
          }
          break;
        }
    }

    if (new_is_invalid && !old_is_invalid)
      ++data_invalid_count_[i];
    else if (!new_is_invalid && old_is_invalid)
      --data_invalid_count_[i];
  }

  // Finally overwrite the data
  data_[data_current_idx_].swap (data);
  data.clear ();
}

template <typename T> int
MedianBuffer<T>::compare (T a, T b)
{
  bool a_is_invalid = buffer_traits<T>::is_invalid (a);
  bool b_is_invalid = buffer_traits<T>::is_invalid (b);
  if (a_is_invalid && b_is_invalid)
    return 0;
  if (a_is_invalid)
    return 1;
  if (b_is_invalid)
    return -1;
  if (a == b)
    return 0;
  return a > b ? 1 : -1;
}

template <typename T>
AverageBuffer<T>::AverageBuffer (std::size_t size,
                                 unsigned char window_size)
: Buffer<T> (size)
, window_size_ (window_size)
, data_current_idx_ (window_size_ - 1)
{
  assert (size_ > 0);
  assert (window_size_ > 0);

  data_.resize (window_size_);
  for (std::size_t i = 0; i < window_size_; ++i)
    data_[i].resize (size_, buffer_traits<T>::invalid ());

  data_sum_.resize (size_, 0);
  data_invalid_count_.resize (size_, window_size_);
}

template <typename T>
AverageBuffer<T>::~AverageBuffer ()
= default;

template <typename T> T
AverageBuffer<T>::operator[] (std::size_t idx) const
{
  assert (idx < size_);
  if (data_invalid_count_[idx] == window_size_)
    return (buffer_traits<T>::invalid ());
  return (data_sum_[idx] / static_cast<T> (window_size_ - data_invalid_count_[idx]));
}

template <typename T> void
AverageBuffer<T>::push (std::vector<T>& data)
{
  assert (data.size () == size_);
  std::lock_guard<std::mutex> lock (data_mutex_);

  if (++data_current_idx_ >= window_size_)
    data_current_idx_ = 0;

  // New data will replace the column with index data_current_idx_. Before
  // overwriting it, we go through the old values and subtract them from the
  // data_sum_
  for (std::size_t i = 0; i < size_; ++i)
  {
    const float& new_value = data[i];
    const float& old_value = data_[data_current_idx_][i];
    bool new_is_invalid = buffer_traits<T>::is_invalid (new_value);
    bool old_is_invalid = buffer_traits<T>::is_invalid (old_value);

    if (!old_is_invalid)
      data_sum_[i] -= old_value;
    if (!new_is_invalid)
      data_sum_[i] += new_value;

    if (new_is_invalid && !old_is_invalid)
      ++data_invalid_count_[i];
    else if (!new_is_invalid && old_is_invalid)
      --data_invalid_count_[i];
  }

  // Finally overwrite the data
  data_[data_current_idx_].swap (data);
  data.clear ();
}

} // namespace io
} // namespace pcl

#endif /* PCL_IO_IMPL_BUFFERS_HPP */

