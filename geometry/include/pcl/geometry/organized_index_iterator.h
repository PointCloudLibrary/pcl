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

#ifndef __PCL_ORGANIZED_IMAGE_ITERATOR__
#define __PCL_ORGANIZED_IMAGE_ITERATOR__

namespace pcl
{
class OrganizedIndexIterator
{
  public:
    OrganizedIndexIterator (unsigned width);
    virtual ~OrganizedIndexIterator ();
    
    virtual void operator ++ () = 0;
    virtual void operator ++ (int);
    unsigned operator* () const;
    virtual unsigned getIndex () const;
    virtual unsigned getRowIndex () const;
    virtual unsigned getColumnIndex () const;
    virtual bool isValid () const = 0;
    virtual void reset () = 0;
  protected:
    unsigned width_;
    unsigned index_;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
inline OrganizedIndexIterator::OrganizedIndexIterator (unsigned width)
: width_ (width)
, index_ (0)
{  
}

////////////////////////////////////////////////////////////////////////////////
inline OrganizedIndexIterator::~OrganizedIndexIterator ()
{  
}

////////////////////////////////////////////////////////////////////////////////
inline void
OrganizedIndexIterator::operator++ (int)
{
  return operator ++();
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
pcl::OrganizedIndexIterator::operator * () const
{
  return index_;
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
pcl::OrganizedIndexIterator::getIndex () const
{
  return index_;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief default implementation. Should be overloaded 
 */
inline unsigned
pcl::OrganizedIndexIterator::getRowIndex () const
{
  return index_ / width_;
}

////////////////////////////////////////////////////////////////////////////////
inline unsigned
pcl::OrganizedIndexIterator::getColumnIndex () const
{
  return index_ % width_;
}
} // namespace pcl

#endif // __PCL_ORGANIZED_IMAGE_ITERATOR__