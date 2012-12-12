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

/// @file   selection.h
/// @details A Selection object maintains the set of indices of points from a
/// point cloud that have been identifed by the selection tools.
/// @author  Yue Li and Matthew Hielsberg


#ifndef SELECTION_H_
#define SELECTION_H_

#include <set>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/statistics.h>

/// @brief This class serves as a sort of mask for performing operations on a
/// point cloud.  It keeps track of the indices of identified/selected points
/// and provides methods for accessing those indices and modifying them.
class Selection : public Statistics
{
  public:
    /// @brief Constructor.
    /// @param cloud_ptr A pointer to the const cloud object for which this
    /// object is to maintain selections.
    Selection (ConstCloudPtr cloud_ptr, bool register_stats=false)
      : cloud_ptr_(cloud_ptr)
    {
      if (register_stats)
        registerStats();
    }

    /// @brief Copy constructor
    /// @param copy The selection object to be copied
    Selection (const Selection& copy)
      : cloud_ptr_(copy.cloud_ptr_), selected_indices_(copy.selected_indices_)
    {
    }

    /// @brief Destructor.
    ~Selection ()
    {
    }

    /// @brief Equal operator
    /// @param selection a const reference to a selection object whose
    /// properties will be copied.
    Selection&
    operator= (const Selection& selection);

    /// @brief Adds the index of the selected point to the selection table.
    /// @param index The index of the point that is selected.
    /// @pre Assumes the passed index is valid with respect to the current
    /// cloud.
    void
    addIndex (unsigned int index);

    /// @brief Removes the index of a point from the selection table.
    /// @param index The index of the point to be removed from the table.
    void
    removeIndex (unsigned int index);

    /// @brief Adds a vector of indices of the selected points to the table.
    /// @param indices A vector of indices of points to be added to the table.
    /// @pre Assumes the passed index is valid with respect to the current
    /// cloud.
    void
    addIndex (const IndexVector &indices);
  
    /// @brief Removes a vector of indices from the table
    /// @param indices A vector of indices of points to be removed from the
    /// table.
    void
    removeIndex (const IndexVector &indices);

    /// @brief Adds a range of consecutive indices into the selection table.
    /// @param start the first index in the range.
    /// @param num the total number of indices in the range.
    /// @pre Assumes the passed index is valid with respect to the current
    /// cloud.
    void
    addIndexRange (unsigned int start, unsigned int num);

    /// @brief Removes a range of consecutive indices into the selection table.
    /// @param start the first index in the range.
    /// @param num the total number of indices in the range.
    void
    removeIndexRange (unsigned int start, unsigned int num);

    /// @brief Removes all the indices from the selection table.
    void
    clear ()
    {
      selected_indices_.clear();
    }

    typedef std::set<unsigned int>::iterator iterator;
    typedef std::set<unsigned int>::const_iterator const_iterator;

    /// @brief Get the begin iterator of the selection.
    const_iterator
    begin () const
    {
      return (selected_indices_.begin());
    }

    /// @brief Get the end iterator of the selection.
    const_iterator
    end () const
    {
      return (selected_indices_.end());
    }

    typedef std::set<unsigned int>::const_reverse_iterator
      const_reverse_iterator;

    /// @brief Get the begin iterator of the selection.
    const_reverse_iterator
    rbegin () const
    {
      return (selected_indices_.rbegin());
    }

    /// @brief Get the end iterator of the selection.
    const_reverse_iterator
    rend () const
    {
      return (selected_indices_.rend());
    }

    /// @brief Returns true if the passed index is selected.
    bool
    isSelected (unsigned int index) const;

    /// @brief Returns true if no point is selected.
    inline
    bool
    empty () const
    {
      return (selected_indices_.empty());
    }

    /// @brief Returns the number of points in the selection
    inline
    unsigned int
    size () const
    {
      return (selected_indices_.size());
    }

    /// @brief Invert selection
    /// @details Make the unselected points selected and deselect the previously
    /// selected points.
    void
    invertSelect ();

    /// @brief Get the statistics of the selected points in string.
    std::string
    getStat () const;

  private:
    /// @brief Default constructor - object is not default constructable
    Selection ()
    {
    }

    /// a pointer to the cloud
    ConstCloudPtr cloud_ptr_;

    /// A set of unique indices that have been selected in the cloud.
    std::set<unsigned int> selected_indices_;
};

#endif // SELECTION_H_
