///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///

/// @file copyBuffer.h
/// @details A CopyBuffer object provides a buffer to store the points
/// copied from the cloud.
/// @author Yue Li and Matthew Hielsberg

#ifndef COPY_BUFFER_H_
#define COPY_BUFFER_H_

#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/cloud.h>

/// @brief a buffer holding the points being copied and a set of operations for
/// manipulating the buffer.
class CopyBuffer : public Statistics
{
  public:
    /// @brief Default Constructor
    /// @details This creates an empty buffer
    CopyBuffer (bool register_stats=false)
    {
      if (register_stats)
        registerStats();
    }
    
    /// @brief Copy Constructor
    /// @details create a copy buffer by copying all the internal states of the
    /// passed copy buffer.
    /// @param copy the copy buffer object used to initialize this object
    CopyBuffer (const CopyBuffer& copy);

    /// @brief Destructor
    ~CopyBuffer ()
    {
    }
    
    /// @brief Equal Operator
    /// @details Copy all the internal states to the this copy buffer object.
    /// @param copy_buffer the copy buffer object used to update the this object
    /// @return A reference to this.
    CopyBuffer&
    operator= (const CopyBuffer& copy_buffer);

    /// @brief Sets the points in the copy buffer.
    /// @details The passed selection pointer is used to get specified points
    /// from the stored cloud pointer and copy them into the internal buffer.
    /// Any points that currently exist in this buffer are removed and replaced
    /// with those passed.  Note that this buffer is cleared prior to any
    /// checking of the state of the passed parameters.
    /// @param cloud_ptr a pointer to a cloud object whose points are to be
    /// copied
    /// @param selection a const reference to the selected points object
    void
    set (ConstCloudPtr cloud_ptr, const Selection& selection);

    /// @brief Returns the points stored in the internal buffer as a const Cloud
    const Cloud&
    get() const;

    /// @brief Returns the points stored in the internal buffer as a Cloud
    Cloud&
    get();
    
    /// @brief Removes all the points from the copy buffer.
    void
    clean ();
        
    /// @brief Get the statistics of the copied points in string.
    std::string
    getStat () const;

    /// @brief Returns true if the buffer is empty, false otherwise
    bool
    empty() const
    {
      return (buffer_.size() == 0);
    }

  private:
    /// a cloud object holding all the copied points.
    Cloud buffer_; 
};
#endif // COPY_BUFFER_H_
