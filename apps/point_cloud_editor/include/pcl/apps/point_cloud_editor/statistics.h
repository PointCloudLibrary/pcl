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

/// @file statistics.h
/// @details The statistics of the current editing. These statistics will be
/// displayed in a pop-up dialog.
/// @author Yue Li and Matthew Hielsberg

#pragma once

#include <vector>
#include <string>
#include <pcl/apps/point_cloud_editor/localTypes.h>

class Statistics
{
  public:
    /// @brief Destructor
    virtual ~Statistics ()
    = default;

    /// @brief Returns the strings of the statistics.
    static
    std::string
    getStats();
    
    static
    void
    clear();
    
  protected:
    /// @brief The default constructor.
    Statistics ()
    = default;

    /// @brief Copy Constructor
    Statistics (const Statistics&)
    {
      assert(false); 
    }

    /// @brief Equal Operator
    virtual
    Statistics&
    operator= (const Statistics&)
    {
      assert(false); return (*this);
    }

    /// @brief Returns the statistics in string.
    virtual
    std::string
    getStat () const = 0;

    /// @brief Register a statistics
    void
    registerStats ();
    
  private:
    static std::vector<Statistics*> stat_vec_;
};
