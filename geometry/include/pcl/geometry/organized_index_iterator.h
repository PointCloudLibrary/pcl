/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

namespace pcl {
/** \brief base class for iterators on 2-dimensional maps like images/organized clouds
 * etc.
 * \author Suat Gedikli <gedikli@willowgarage.com>
 * \ingroup  geometry
 */
class OrganizedIndexIterator {
public:
  /**
   * \brief constructor
   * \param[in] width the width of the image/organized cloud
   */
  OrganizedIndexIterator(unsigned width);

  /** \brief virtual destructor*/
  virtual ~OrganizedIndexIterator();

  /** \brief go to next pixel/point in image/cloud*/
  virtual void
  operator++() = 0;

  /** \brief go to next pixel/point in image/cloud*/
  virtual void
  operator++(int);

  /** \brief returns the pixel/point index in the linearized memory of the image/cloud
   * \return the pixel/point index in the linearized memory of the image/cloud
   */
  unsigned
  operator*() const;

  /** \brief returns the pixel/point index in the linearized memory of the image/cloud
   * \return the pixel/point index in the linearized memory of the image/cloud
   */
  virtual unsigned
  getIndex() const;

  /** \brief returns the row index (y-coordinate) of the current pixel/point
   * \return  the row index (y-coordinate) of the current pixel/point
   */
  virtual unsigned
  getRowIndex() const;

  /** \brief returns the col index (x-coordinate) of the current pixel/point
   * \return  the col index (x-coordinate) of the current pixel/point
   */
  virtual unsigned
  getColumnIndex() const;

  /** \brief return whether the current visited pixel/point is valid or not.
   * \return true if the current pixel/point is within the points to be iterated over,
   * false otherwise
   */
  virtual bool
  isValid() const = 0;

  /** \brief resets the iterator to the beginning of the line
   */
  virtual void
  reset() = 0;

protected:
  /** \brief the width of the image/cloud*/
  unsigned width_;

  /** \brief the index of the current pixel/point*/
  unsigned index_;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
inline OrganizedIndexIterator::OrganizedIndexIterator(unsigned width)
: width_(width), index_(0)
{}

////////////////////////////////////////////////////////////////////////////////
inline OrganizedIndexIterator::~OrganizedIndexIterator() = default;

////////////////////////////////////////////////////////////////////////////////
inline void
OrganizedIndexIterator::operator++(int)
{
  return operator++();
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
pcl::OrganizedIndexIterator::operator*() const
{
  return index_;
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
pcl::OrganizedIndexIterator::getIndex() const
{
  return index_;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief default implementation. Should be overloaded
 */
inline unsigned
pcl::OrganizedIndexIterator::getRowIndex() const
{
  return index_ / width_;
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
pcl::OrganizedIndexIterator::getColumnIndex() const
{
  return index_ % width_;
}
} // namespace pcl
