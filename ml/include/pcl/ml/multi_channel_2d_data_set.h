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
  
#ifndef PCL_ML_MULTI_CHANNEL_2D_DATA_SET_H_
#define PCL_ML_MULTI_CHANNEL_2D_DATA_SET_H_

#include <pcl/common/common.h>

#include <istream>
#include <ostream>

namespace pcl
{

  /** \brief Holds two-dimensional multi-channel data. */
  template <class DATA_TYPE, size_t NUM_OF_CHANNELS>
  class PCL_EXPORTS MultiChannel2DData
  {
    public:

      /** \brief Constructor. */
      inline MultiChannel2DData () : data_ (NULL), width_ (0), height_ (0) {}
      /** \brief Destructor. */
      virtual ~MultiChannel2DData () {}

      /** \brief Resizes the internal data storage.
        * \param[in] width The width of the resized 2D data array.
        * \param[in] height The height of the resized 2D data array.
        */
      inline void
      resize (size_t width, size_t height)
      {
        data_.resize (NUM_OF_CHANNELS*width*height);
        width_ = width;
        height_ = height;
      }

      /** \brief Clears the internal data storage and sets width and height to 0.
        */
      void
      clear ()
      {
        width_ = 0;
        height_ = 0;
        data_.clear ();
      }
  
      /** \brief Returns a pointer to the internal data at the specified location.
        * \param[in] col_index The column index.
        * \param[in] row_index The row index.
        */
      inline DATA_TYPE *
      operator() (const size_t col_index, const size_t row_index)
      {
        return &(data_[NUM_OF_CHANNELS*(row_index*width_ + col_index)]);
      };

      /** \brief Returns a pointer to the internal data at the specified location.
        * \param[in] col_index The column index.
        * \param[in] row_index The row index.
        */
      inline const DATA_TYPE *
      operator() (const size_t col_index, const size_t row_index) const
      {
        return &(data_[NUM_OF_CHANNELS*(row_index*width_ + col_index)]);
      };

      /** \brief Returns a reference to the internal data at the specified location.
        * \param[in] col_index The column index.
        * \param[in] row_index The row index.
        * \param[in] channel The channel index.
        */
      inline DATA_TYPE &
      operator() (const size_t col_index, const size_t row_index, const size_t channel)
      {
        return data_[NUM_OF_CHANNELS*(row_index*width_ + col_index) + channel];
      };

      /** \brief Returns a reference to the internal data at the specified location.
        * \param[in] col_index The column index.
        * \param[in] row_index The row index.
        * \param[in] channel The channel index.
        */
      inline const DATA_TYPE &
      operator() (const size_t col_index, const size_t row_index, const size_t channel) const
      {
        return data_[NUM_OF_CHANNELS*(row_index*width_ + col_index) + channel];
      };

    private:

      /** \brief The internal data storage. */
      std::vector<DATA_TYPE> data_;

      /** \brief The width of the data storage. */
      size_t width_;
      /** \brief The height of the data storage. */
      size_t height_;
  };


  /** \brief Holds a set of two-dimensional multi-channel data. */
  template <class DATA_TYPE, size_t NUM_OF_CHANNELS>
  class PCL_EXPORTS MultiChannel2DDataSet
  {
    public:

      /** \brief Constructor. */
      inline MultiChannel2DDataSet () : data_set_ () {}
      /** \brief Destructor. */
      virtual ~MultiChannel2DDataSet () {}

      /** \brief Adds a new two-dimensional data block to the data set. 
        * \param[in] width The width of the new data block.
        * \param[in] height The height of the new data block.
        */
      void 
      addData (const size_t width, const size_t height)
      {
        MultiChannel2DData<DATA_TYPE, NUM_OF_CHANNELS> * data = new MultiChannel2DData<DATA_TYPE, NUM_OF_CHANNELS> ();
        data->resize (width, height);

        data_set_.push_back (data);
      };

      /** \brief Releases the data stored in the data set. */
      void
      releaseDataSet ()
      {
        for (size_t data_set_index = 0; data_set_index < data_set_.size (); ++data_set_index)
        {
          delete data_set_[data_set_index];
        }
      }

      /** \brief Releases the data stored in the data set. */
      void
      clear ()
      {
        releaseDataSet ();
      }

      /** \brief Returns a pointer to the specified data block at the specified location.
        * \param[in] data_set_id The index of the data block.
        * \param[in] col The column of the desired location.
        * \param[in] row The row of the desired location.
        */
      inline DATA_TYPE * 
      operator() (const size_t data_set_id, const size_t col, const size_t row)
      {
        return (*data_set_[data_set_id]) (col, row);
      };

      /** \brief Returns a pointer to the specified data block at the specified location.
        * \param[in] data_set_id The index of the data block.
        * \param[in] col The column of the desired location.
        * \param[in] row The row of the desired location.
        */
      inline const DATA_TYPE * 
      operator() (const size_t data_set_id, const size_t col, const size_t row) const
      {
        return (*data_set_[data_set_id]) (col, row);
      };

      /** \brief Returns a reference to the specified data block at the specified location.
        * \param[in] data_set_id The index of the data block.
        * \param[in] col The column of the desired location.
        * \param[in] row The row of the desired location.
        * \param[in] channel The channel index.
        */
      inline DATA_TYPE & 
      operator() (const size_t data_set_id, const size_t col, const size_t row, const size_t channel)
      {
        return (*data_set_[data_set_id]) (col, row, channel);
      };

      /** \brief Returns a reference to the specified data block at the specified location.
        * \param[in] data_set_id The index of the data block.
        * \param[in] col The column of the desired location.
        * \param[in] row The row of the desired location.
        * \param[in] channel The channel index.
        */
      inline const DATA_TYPE & 
      operator() (const size_t data_set_id, const size_t col, const size_t row, const size_t channel) const
      {
        return (*data_set_[data_set_id]) (col, row, channel);
      };

    private:

      /** \brief The data set. */
      std::vector<MultiChannel2DData<DATA_TYPE, NUM_OF_CHANNELS>*> data_set_;
  };

  typedef MultiChannel2DDataSet<float, 1> Depth2DDataSet;
  typedef MultiChannel2DDataSet<float, 2> IntensityDepth2DDataSet;
  typedef MultiChannel2DDataSet<float, 3> RGB2DDataSet;
  typedef MultiChannel2DDataSet<float, 4> RGBD2DDataSet;

}

#endif
