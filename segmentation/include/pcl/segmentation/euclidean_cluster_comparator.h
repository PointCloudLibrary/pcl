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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 *
 */

#pragma once

#include <set> // for std::set
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/comparator.h>


namespace pcl
{
  /** \brief EuclideanClusterComparator is a comparator used for finding clusters based on euclidian distance.
  *
  * \author Alex Trevor
  */
  template<typename PointT, typename PointLT = pcl::Label>
  class EuclideanClusterComparator : public ::pcl::Comparator<PointT>
  {
    protected:

      using pcl::Comparator<PointT>::input_;

    public:
      using typename Comparator<PointT>::PointCloud;
      using typename Comparator<PointT>::PointCloudConstPtr;

      using PointCloudL = pcl::PointCloud<PointLT>;
      using PointCloudLPtr = typename PointCloudL::Ptr;
      using PointCloudLConstPtr = typename PointCloudL::ConstPtr;

      using Ptr = shared_ptr<EuclideanClusterComparator<PointT, PointLT> >;
      using ConstPtr = shared_ptr<const EuclideanClusterComparator<PointT, PointLT> >;

      using ExcludeLabelSet = std::set<std::uint32_t>;
      using ExcludeLabelSetPtr = shared_ptr<ExcludeLabelSet>;
      using ExcludeLabelSetConstPtr = shared_ptr<const ExcludeLabelSet>;

      /** \brief Default constructor for EuclideanClusterComparator. */
      EuclideanClusterComparator() = default;

      void
      setInputCloud (const PointCloudConstPtr& cloud) override
      {
        input_ = cloud;
        Eigen::Matrix3f rot = input_->sensor_orientation_.toRotationMatrix ();
        z_axis_ = rot.col (2);
      }

      /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
        * \param[in] distance_threshold the tolerance in meters
        * \param depth_dependent
        */
      inline void
      setDistanceThreshold (float distance_threshold, bool depth_dependent)
      {
        distance_threshold_ = distance_threshold;
        depth_dependent_ = depth_dependent;
      }

      /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
      inline float
      getDistanceThreshold () const
      {
        return distance_threshold_;
      }

      /** \brief Get if depth dependent */
      inline bool
      getDepthDependent() const
      {
        return depth_dependent_;
      }

      /** \brief Set label cloud
        * \param[in] labels The label cloud
        */
      void
      setLabels (const PointCloudLPtr& labels)
      {
        labels_ = labels;
      }

      /** \brief Get labels */
      const PointCloudLPtr&
      getLabels() const
      {
        return labels_;
      }

      /** \brief Get exlude labels */
      const ExcludeLabelSetConstPtr&
      getExcludeLabels () const
      {
        return exclude_labels_;
      }

      /** \brief Set labels in the label cloud to exclude.
        * \param exclude_labels a vector of bools corresponding to whether or not a given label should be considered
        */
      void
      setExcludeLabels (const ExcludeLabelSetConstPtr &exclude_labels)
      {
        exclude_labels_ = exclude_labels;
      }

      /** \brief Compare points at two indices by their euclidean distance
        * \param idx1 The first index for the comparison
        * \param idx2 The second index for the comparison
        */
      bool
      compare (int idx1, int idx2) const override
      {
        if (labels_ && exclude_labels_)
        {
          assert (labels_->size () == input_->size ());
          const std::uint32_t &label1 = (*labels_)[idx1].label;
          const std::uint32_t &label2 = (*labels_)[idx2].label;

          const std::set<std::uint32_t>::const_iterator it1 = exclude_labels_->find (label1);
          if (it1 == exclude_labels_->end ())
            return false;

          const std::set<std::uint32_t>::const_iterator it2 = exclude_labels_->find (label2);
          if (it2 == exclude_labels_->end ())
            return false;
        }

        float dist_threshold = distance_threshold_;
        if (depth_dependent_)
        {
          Eigen::Vector3f vec = (*input_)[idx1].getVector3fMap ();
          float z = vec.dot (z_axis_);
          dist_threshold *= z * z;
        }

        const float dist = ((*input_)[idx1].getVector3fMap ()
                              - (*input_)[idx2].getVector3fMap ()).norm ();
        return (dist < dist_threshold);
      }

    protected:


      /** \brief Set of labels with similar size as the input point cloud,
        * aggregating points into groups based on a similar label identifier.
        *
        * It needs to be set in conjunction with the \ref exclude_labels_
        * member in order to provided a masking functionality.
        */
      PointCloudLPtr labels_;

      /** \brief Specifies which labels should be excluded com being clustered.
        *
        * If a label is not specified, it's assumed by default that it's
        * intended be excluded
        */
      ExcludeLabelSetConstPtr exclude_labels_;

      float distance_threshold_ = 0.005f;

      bool depth_dependent_ = false;

      Eigen::Vector3f z_axis_;
  };
}
