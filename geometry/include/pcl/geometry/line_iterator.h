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

#ifndef __PCL_LINE_ITERATOR__
#define __PCL_LINE_ITERATOR__
#include "organized_index_iterator.h"

namespace pcl
{
/** \brief Organized Index Iterator for iterating over the "pixels" for a given line using the Bresenham algorithm.
  * Supports 4 and 8 neighborhood connectivity
  * \note iterator does not visit the given end-point (on purpose).
  * \author Suat Gedikli <gedikli@willowgarage.com>
  * \ingroup  geometry
  */
class LineIterator : public OrganizedIndexIterator
{
  public:
    /** \brief Neighborhood connectivity  */
    typedef enum {Neighbor4 = 4, Neighbor8 = 8} Neighborhood;
  public:
    /** \brief Constructor
      * \param x_start column of the start pixel of the line
      * \param y_start row of the start pixel of the line
      * \param x_end column of the end pixel of the line
      * \param y_end row of the end pixel of the line
      * \param width width of the organized structure e.g. image/cloud/map etc..
      * \param neighborhood connectivity of the neighborhood
      */
    LineIterator (unsigned x_start, unsigned y_start, unsigned x_end, unsigned y_end, unsigned width, const Neighborhood& neighborhood = Neighbor8);
    
    /** \brief Destructor*/
    virtual ~LineIterator ();
    
    virtual void operator ++ ();
    virtual unsigned getRowIndex () const;
    virtual unsigned getColumnIndex () const;
    virtual bool isValid () const;
    virtual void reset ();
  protected:
    /** \brief initializes the variables for the Bresenham algorithm
      * \param[in] neighborhood connectivity to the neighborhood. Either 4 or 8
      */
    void init (const Neighborhood& neighborhood);
    
    /** \brief current column index*/
    unsigned x_;
    
    /** \brief current row index*/
    unsigned y_;
    
    /** \brief column index of first pixel/point*/
    unsigned x_start_;
    
    /** \brief row index of first pixel/point*/
    unsigned y_start_;
    
    /** \brief column index of end pixel/point*/
    unsigned x_end_;
    
    /** \brief row index of end pixel/point*/
    unsigned y_end_;
    
    // calculated values
    /** \brief current distance to the line*/
    int error_;
    
    /** \brief error threshold*/
    int error_max_;
    
    /** \brief increment of error (distance) value in case of an y-step (if dx > dy)*/
    int error_minus_;
    
    /** \brief increment of error (distance) value in case of just an x-step (if dx > dy)*/
    int error_plus_;
    
    /** \brief increment of column index in case of just an x-step (if dx > dy)*/
    int x_plus_;

    /** \brief increment of row index in case of just an x-step (if dx > dy)*/
    int y_plus_;
    
    /** \brief increment of column index in case of just an y-step (if dx > dy)*/
    int x_minus_;

    /** \brief increment of row index in case of just an y-step (if dx > dy)*/
    int y_minus_;
    
    /** \brief increment pixel/point index in case of just an x-step (if dx > dy)*/
    int index_plus_;

    /** \brief increment pixel/point index in case of just an y-step (if dx > dy)*/
    int index_minus_;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
inline LineIterator::LineIterator (unsigned x_start, unsigned y_start, unsigned x_end, unsigned y_end, unsigned width, const Neighborhood& neighborhood)
: OrganizedIndexIterator (width)
, x_start_ (x_start)
, y_start_ (y_start)
, x_end_ (x_end)
, y_end_ (y_end)
{
  init (neighborhood);
}

////////////////////////////////////////////////////////////////////////////////
inline LineIterator::~LineIterator ()
{  
}

////////////////////////////////////////////////////////////////////////////////
inline void
LineIterator::init (const Neighborhood& neighborhood)
{
  x_ = x_start_;
  y_ = y_start_;
  index_ = x_ * width_ + y_;
  error_ = 0;

  int delta_x = x_end_ - x_start_;
  int delta_y = y_end_ - y_start_;
  
  int x_dir = ( (delta_x > 0) ? 1 : -1 ) ;
  int y_dir = ( (delta_y > 0) ? 1 : -1 ) ;

  delta_x *= x_dir;
  delta_y *= y_dir;
  
  if(delta_x >= delta_y)
  {
    if( neighborhood == Neighbor4 )
    {
      error_max_ = delta_x - delta_y;
      x_minus_ = 0;
      y_minus_ = y_dir;
      x_plus_  = x_dir;
      y_plus_  = 0;

      error_minus_ = -(delta_x * 2);
      error_plus_  = (delta_y * 2);
    }
    else
    {
      error_max_ = delta_x - (delta_y * 2);
      x_minus_ = x_dir;
      y_minus_ = y_dir;
      x_plus_  = x_dir;
      y_plus_  = 0;
        
      error_minus_ = (delta_y - delta_x) * 2;
      error_plus_  = (delta_y * 2);
    }
  }
  else
  {
    if( neighborhood == Neighbor4 )
    {
      error_max_ = delta_y - delta_x;
      x_minus_ = x_dir;
      y_minus_ = 0;
      x_plus_  = 0;
      y_plus_  = y_dir;

      error_minus_ = -(delta_y * 2);
      error_plus_  = (delta_x * 2);
    }
    else
    {
      error_max_ = delta_y - (delta_x * 2);
      x_minus_ = x_dir;
      y_minus_ = y_dir;
      x_plus_  = 0;
      y_plus_  = y_dir;

      error_minus_ = (delta_x - delta_y) * 2;
      error_plus_  = (delta_x * 2);
    }
  }

  index_minus_ = x_minus_ + y_minus_ * width_;
  index_plus_ = x_plus_ + y_plus_ * width_;  
}

////////////////////////////////////////////////////////////////////////////////
inline void
LineIterator::operator ++ ()
{
  if (error_ >= error_max_ )
  {
    x_ += x_minus_;
    y_ += y_minus_;
    error_ += error_minus_;
    index_ += index_minus_;
  }
  else
  {
    x_ += x_plus_;
    y_ += y_plus_;
    error_ += error_plus_;
    index_ += index_plus_;
  }  
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
LineIterator::getRowIndex () const
{
  return y_;
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
LineIterator::getColumnIndex () const
{
  return x_;
}

////////////////////////////////////////////////////////////////////////////////
inline bool
LineIterator::isValid () const
{
  return (x_ != x_end_ || y_ != y_end_);
}

////////////////////////////////////////////////////////////////////////////////
inline void
LineIterator::reset ()
{
  x_ = x_start_;
  y_ = y_start_;
  error_ = 0;
  index_ = x_ * width_ + y_;
}

} // namespace pcl

#endif // __PCL_LINE_ITERATOR__
