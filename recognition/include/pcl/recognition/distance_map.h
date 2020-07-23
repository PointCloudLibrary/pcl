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

namespace pcl
{

  /** \brief Represents a distance map obtained from a distance transformation. 
    * \author Stefan Holzer
    */
  class DistanceMap
  {
    public:
      /** \brief Constructor. */
      DistanceMap () : data_ (0), width_ (0), height_ (0) {}
      /** \brief Destructor. */
      virtual ~DistanceMap () = default;

      /** \brief Returns the width of the map. */
      inline std::size_t 
      getWidth () const
      {
        return (width_); 
      }

      /** \brief Returns the height of the map. */
      inline std::size_t 
      getHeight () const
      { 
        return (height_); 
      }
    
      /** \brief Returns a pointer to the beginning of map. */
      inline float * 
      getData () 
      { 
        return (&data_[0]); 
      }

      /** \brief Resizes the map to the specified size.
        * \param[in] width the new width of the map.
        * \param[in] height the new height of the map.
        */
      void 
      resize (const std::size_t width, const std::size_t height)
      {
        data_.resize (width*height);
        width_ = width;
        height_ = height;
      }

      /** \brief Operator to access an element of the map.
        * \param[in] col_index the column index of the element to access.
        * \param[in] row_index the row index of the element to access.
        */
      inline float & 
      operator() (const std::size_t col_index, const std::size_t row_index)
      {
        return (data_[row_index*width_ + col_index]);
      }

      /** \brief Operator to access an element of the map.
        * \param[in] col_index the column index of the element to access.
        * \param[in] row_index the row index of the element to access.
        */
      inline const float & 
      operator() (const std::size_t col_index, const std::size_t row_index) const
      {
        return (data_[row_index*width_ + col_index]);
      }

    protected:
      /** \brief The storage for the distance map data. */
      std::vector<float> data_;
      /** \brief The width of the map. */
      std::size_t width_;
      /** \brief The height of the map. */
      std::size_t height_;
  };

}
