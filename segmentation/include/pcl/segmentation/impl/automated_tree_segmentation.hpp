/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: automated_tree_segmentation.hpp $
 *
 */

#ifndef PCL_SEGMENTATION_IMPL_AUTOMATED_TREE_SEGMENTATION_H_
#define PCL_SEGMENTATION_IMPL_AUTOMATED_TREE_SEGMENTATION_H_

#include <pcl/segmentation/automated_tree_segmentation.h>
#include <pcl/common/io.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// I am not sure if I am going to continue using this: -FF
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace pcl
{
  /** \brief A helper functor that can specify if the given field exists. */
  template<typename PointT>
  struct CheckIfFieldExists
  {
      /** \brief Constructor.
        * \param[in] field the name of the field
        * \param[out] exists set to true if the field exists, false otherwise
        */
      CheckIfFieldExists (const std::string &field, bool &exists) :
          name_ (field), exists_ (exists)
      {
      }

      /** \brief Operator. */
      template<typename Key>
        inline void
        operator() ()
        {
          if (name_ == pcl::traits::name<PointT, Key>::value)
            exists_ = true;
        }

    private:
      const std::string &name_;
      bool &exists_;
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void
pcl::AutomatedTreeSegmentation<PointT>::applySegmentation ()
{
  bool field_exists;

  pcl::for_each_type<FieldList> (pcl::CheckIfFieldExists<PointT> ("intensity", field_exists));
  if (field_exists)
  {
    // Perform intensity classification
    // TODO adapt to varying minimum and maximum ranges of intensity
    float intensity;
    for (int i_it = 0; i_it < static_cast<int> (indices_->size ()); ++i_it)
    {
      pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type> (pcl::CopyIfFieldExists<PointT, float> (input_->points[(*indices_)[i_it]], "intensity", intensity));
      if (intensity > 175)
        (*segmentation_probabilities_)[(*indices_)[i_it]] -= 0.4f;
      else if (intensity > 125)
        (*segmentation_probabilities_)[(*indices_)[i_it]] -= 0.2f;
      else if (intensity > 75)
        (*segmentation_probabilities_)[(*indices_)[i_it]] += 0.1f;
      else
        (*segmentation_probabilities_)[(*indices_)[i_it]] += 0.3f;
    }
  }

  pcl::for_each_type<FieldList> (pcl::CheckIfFieldExists<PointT> ("curvature", field_exists));
  if (!field_exists)
  {
    // Perform curvature estimation

    // Work in progress
  }
  // Perform curvature classification

  // Work in progress
}

#define PCL_INSTANTIATE_AutomatedTreeSegmentation(T) template class PCL_EXPORTS pcl::AutomatedTreeSegmentation<T>;

#endif  //#ifndef PCL_SEGMENTATION_IMPL_AUTOMATED_TREE_SEGMENTATION_H_

