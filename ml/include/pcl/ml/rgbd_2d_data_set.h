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
  
#ifndef PCL_ML_RGBD_2D_DATA_SET_H_
#define PCL_ML_RGBD_2D_DATA_SET_H_

#include <pcl/common/common.h>

#include <istream>
#include <ostream>

namespace pcl
{

  /** \brief Holds two-dimensional RGBD data. */
  class PCL_EXPORTS RGBD2DData
  {
    public:

      /** \brief Constructor. */
      inline RGBD2DData () : data_ (NULL), width_ (0), height_ (0) {}
      /** \brief Destructor. */
      virtual ~RGBD2DData () {}

      /** \brief Resizes the internal data storage.
        * \param[in] width The width of the resized 2D data array.
        * \param[in] height The height of the resized 2D data array.
        */
      void
      resize (size_t width, size_t height);
  
      /** \brief Returns a pointer to the internal data at the specified location.
        * \param[in] col_index The column index.
        * \param[in] row_index The row index.
        */
      inline float *
      operator() (const size_t col_index, const size_t row_index)
      {
        return &(data_[4*(row_index*width_ + col_index)]);
      };

    private:

      /** \brief The internal data storage. */
      std::vector<float> data_;

      /** \brief The width of the data storage. */
      size_t width_;
      /** \brief The height of the data storage. */
      size_t height_;
  };


  /** \brief Holds a set of two-dimensional RGBD data. */
  class PCL_EXPORTS RGBD2DDataSet
  {
    public:

      /** \brief Constructor. */
      inline RGBD2DDataSet () : data_set_ () {}
      /** \brief Destructor. */
      virtual ~RGBD2DDataSet () {}

      /** \brief Adds a new two-dimensional data block to the data set. 
        * \param[in] width The width of the new data block.
        * \param[in] height The height of the new data block.
        */
      void 
      addData (const size_t width, const size_t height)
      {
        RGBD2DData * data = new RGBD2DData();
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

      /** \brief Returns a pointer to the specified data block at the specified location.
        * \param[in] data_set_id The index of the data block.
        * \param[in] col The column of the desired location.
        * \param[in] row The row of the desired location.
        */
      inline float * 
      operator() (const size_t data_set_id, const size_t col, const size_t row)
      {
        return (*data_set_[data_set_id]) (col, row);
      };

    private:

      /** \brief The data set. */
      std::vector<RGBD2DData*> data_set_;
  };

}

#endif
