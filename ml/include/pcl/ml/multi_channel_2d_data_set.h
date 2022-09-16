/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#pragma once

#include <pcl/common/common.h>

#include <istream>
#include <ostream>

namespace pcl {

/** Holds two-dimensional multi-channel data. */
template <class DATA_TYPE, std::size_t NUM_OF_CHANNELS>
class PCL_EXPORTS MultiChannel2DData {
public:
  /** Constructor. */
  inline MultiChannel2DData() : data_(NULL), width_(0), height_(0) {}

  /** Resizes the internal data storage.
   *
   * \param[in] width the width of the resized 2D data array
   * \param[in] height the height of the resized 2D data array
   */
  inline void
  resize(std::size_t width, std::size_t height)
  {
    data_.resize(NUM_OF_CHANNELS * width * height);
    width_ = width;
    height_ = height;
  }

  /** Clears the internal data storage and sets width and height to 0. */
  void
  clear()
  {
    width_ = 0;
    height_ = 0;
    data_.clear();
  }

  /** Returns a pointer to the internal data at the specified location.
   *
   * \param[in] col_index the column index
   * \param[in] row_index the row index
   */
  inline DATA_TYPE*
  operator()(const std::size_t col_index, const std::size_t row_index)
  {
    return &(data_[NUM_OF_CHANNELS * (row_index * width_ + col_index)]);
  };

  /** Returns a pointer to the internal data at the specified location.
   *
   * \param[in] col_index the column index
   * \param[in] row_index the row index
   */
  inline const DATA_TYPE*
  operator()(const std::size_t col_index, const std::size_t row_index) const
  {
    return &(data_[NUM_OF_CHANNELS * (row_index * width_ + col_index)]);
  };

  /** Returns a reference to the internal data at the specified location.
   *
   * \param[in] col_index the column index
   * \param[in] row_index the row index
   * \param[in] channel the channel index
   */
  inline DATA_TYPE&
  operator()(const std::size_t col_index,
             const std::size_t row_index,
             const std::size_t channel)
  {
    return data_[NUM_OF_CHANNELS * (row_index * width_ + col_index) + channel];
  };

  /** Returns a reference to the internal data at the specified location.
   *
   * \param[in] col_index the column index
   * \param[in] row_index the row index
   * \param[in] channel the channel index
   */
  inline const DATA_TYPE&
  operator()(const std::size_t col_index,
             const std::size_t row_index,
             const std::size_t channel) const
  {
    return data_[NUM_OF_CHANNELS * (row_index * width_ + col_index) + channel];
  };

private:
  /** The internal data storage. */
  std::vector<DATA_TYPE> data_;
  /** The width of the data storage. */
  std::size_t width_;
  /** The height of the data storage. */
  std::size_t height_;
};

/** Holds a set of two-dimensional multi-channel data. */
template <class DATA_TYPE, std::size_t NUM_OF_CHANNELS>
class PCL_EXPORTS MultiChannel2DDataSet {
public:
  /** Constructor. */
  inline MultiChannel2DDataSet() : data_set_() {}

  /** Adds a new two-dimensional data block to the data set.
   *
   * \param[in] width the width of the new data block
   * \param[in] height the height of the new data block
   */
  void
  addData(const std::size_t width, const std::size_t height)
  {
    MultiChannel2DData<DATA_TYPE, NUM_OF_CHANNELS>* data =
        new MultiChannel2DData<DATA_TYPE, NUM_OF_CHANNELS>();
    data->resize(width, height);

    data_set_.push_back(data);
  };

  /** Releases the data stored in the data set. */
  void
  releaseDataSet()
  {
    for (std::size_t data_set_index = 0; data_set_index < data_set_.size();
         ++data_set_index) {
      delete data_set_[data_set_index];
    }
  }

  /** Releases the data stored in the data set. */
  void
  clear()
  {
    releaseDataSet();
  }

  /** Returns a pointer to the specified data block at the specified location.
   *
   * \param[in] data_set_id the index of the data block
   * \param[in] col the column of the desired location
   * \param[in] row the row of the desired location
   */
  inline DATA_TYPE*
  operator()(const std::size_t data_set_id,
             const std::size_t col,
             const std::size_t row)
  {
    return (*data_set_[data_set_id])(col, row);
  };

  /** Returns a pointer to the specified data block at the specified location.
   *
   * \param[in] data_set_id the index of the data block
   * \param[in] col the column of the desired location
   * \param[in] row the row of the desired location
   */
  inline const DATA_TYPE*
  operator()(const std::size_t data_set_id,
             const std::size_t col,
             const std::size_t row) const
  {
    return (*data_set_[data_set_id])(col, row);
  };

  /** Returns a reference to the specified data block at the specified location.
   *
   * \param[in] data_set_id the index of the data block
   * \param[in] col the column of the desired location
   * \param[in] row the row of the desired location
   * \param[in] channel the channel index
   */
  inline DATA_TYPE&
  operator()(const std::size_t data_set_id,
             const std::size_t col,
             const std::size_t row,
             const std::size_t channel)
  {
    return (*data_set_[data_set_id])(col, row, channel);
  };

  /** Returns a reference to the specified data block at the specified location.
   *
   * \param[in] data_set_id the index of the data block
   * \param[in] col the column of the desired location
   * \param[in] row the row of the desired location
   * \param[in] channel the channel index
   */
  inline const DATA_TYPE&
  operator()(const std::size_t data_set_id,
             const std::size_t col,
             const std::size_t row,
             const std::size_t channel) const
  {
    return (*data_set_[data_set_id])(col, row, channel);
  };

private:
  /** The data set. */
  std::vector<MultiChannel2DData<DATA_TYPE, NUM_OF_CHANNELS>*> data_set_;
};

using Depth2DDataSet = MultiChannel2DDataSet<float, 1>;
using IntensityDepth2DDataSet = MultiChannel2DDataSet<float, 2>;
using RGB2DDataSet = MultiChannel2DDataSet<float, 3>;
using RGBD2DDataSet = MultiChannel2DDataSet<float, 4>;

} // namespace pcl
