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

/// @file denoiseCommand.h
/// @details A DenoiseCommand object provides functionalities for removing the
/// outliers in the input cloud. This class also serves an example for developers
/// on extending the editors with cloud processing algorithms.  See
/// http://pointclouds.org/documentation/tutorials/statistical_outlier.php
/// @author Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/apps/point_cloud_editor/command.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>

#include <pcl/memory.h>  // for pcl::shared_ptr

class DenoiseCommand : public Command
{
public:
  /// The type for shared pointer pointing to a selection buffer
  using SelectionPtr = pcl::shared_ptr<Selection>;

  /// @brief Constructor
  /// @param selection_ptr a shared pointer pointing to the selection object.
  /// @param cloud_ptr a shared pointer pointing to the cloud object.
  /// @param mean the number of points to use for mean distance estimation.
  /// @param threshold the standard deviation multiplier threshold
  DenoiseCommand (SelectionPtr selection_ptr, const CloudPtr& cloud_ptr,
                  float mean, float threshold)
    : selection_ptr_(std::move(selection_ptr)), cloud_ptr_(cloud_ptr), mean_(mean),
      threshold_(threshold), removed_indices_(cloud_ptr)
  {
  }

  /// @brief Copy constructor - commands are non-copyable
  DenoiseCommand (const DenoiseCommand&) = delete;

  /// @brief Equal operator - commands are non-copyable
  DenoiseCommand&
  operator= (const DenoiseCommand&) = delete;

protected:
  /// @brief Runs the denois algorithm to remove all the outliers.
  void
  execute () override;

  /// @brief Adds the removed noisy points back to the cloud
  void
  undo () override;

private:
  /// A shared pointer pointing to the selection object of the widget
  SelectionPtr selection_ptr_;

  /// A pointer pointing to the cloud of the widget
  CloudPtr cloud_ptr_;

  /// The number of points to use for mean distance estimation.
  float mean_;

  /// The standard deviation multiplier threshold
  float threshold_;

  /// A copy buffer which backs up the noisy point removed after denoising.
  CopyBuffer removed_points_;

  /// A selection object which backs up the indices of the noisy points removed.
  Selection removed_indices_;
};
