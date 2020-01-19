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

#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/comparator.h>
#include <pcl/point_types.h>

namespace pcl
{
  namespace experimental
  {
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
        EuclideanClusterComparator ()
          : distance_threshold_ (0.005f)
          , depth_dependent_ ()
        {}

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
          return (distance_threshold_);
        }

        /** \brief Set label cloud
          * \param[in] labels The label cloud
          */
        void
        setLabels (const PointCloudLPtr& labels)
        {
          labels_ = labels;
        }

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
            Eigen::Vector3f vec = input_->points[idx1].getVector3fMap ();
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

        float distance_threshold_;

        bool depth_dependent_;

        Eigen::Vector3f z_axis_;
    };
  } // namespace experimental


  /** \brief EuclideanClusterComparator is a comparator used for finding clusters based on euclidian distance.
    *
    * \author Alex Trevor
    */
  template<typename PointT, typename PointNT, typename PointLT = deprecated::T>
  class EuclideanClusterComparator : public experimental::EuclideanClusterComparator<PointT, PointLT>
  {
    protected:

      using experimental::EuclideanClusterComparator<PointT, PointLT>::exclude_labels_;

    public:

      using PointCloudN = pcl::PointCloud<PointNT>;
      using PointCloudNPtr = typename PointCloudN::Ptr;
      using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

      using Ptr = shared_ptr<EuclideanClusterComparator<PointT, PointNT, PointLT> >;
      using ConstPtr = shared_ptr<const EuclideanClusterComparator<PointT, PointNT, PointLT> >;

      using experimental::EuclideanClusterComparator<PointT, PointLT>::setExcludeLabels;

      /** \brief Default constructor for EuclideanClusterComparator. */
      [[deprecated("remove PointNT from template parameters")]]
      EuclideanClusterComparator ()
        : normals_ ()
        , angular_threshold_ (0.0f)
      {}

      /** \brief Provide a pointer to the input normals.
       * \param[in] normals the input normal cloud
       */
      [[deprecated("EuclideadClusterComparator never actually used normals and angular threshold, this function has no effect on the behavior of the comparator. It is deprecated and will be removed in future releases.")]]
      inline void
      setInputNormals (const PointCloudNConstPtr& normals) { normals_ = normals; }

      /** \brief Get the input normals. */
      [[deprecated("EuclideadClusterComparator never actually used normals and angular threshold, this function has no effect on the behavior of the comparator. It is deprecated and will be removed in future releases.")]]
      inline PointCloudNConstPtr
      getInputNormals () const { return (normals_); }

      /** \brief Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
        * \param[in] angular_threshold the tolerance in radians
        */
      [[deprecated("EuclideadClusterComparator never actually used normals and angular threshold, this function has no effect on the behavior of the comparator. It is deprecated and will be removed in future releases.")]]
      inline void
      setAngularThreshold (float angular_threshold)
      {
        angular_threshold_ = std::cos (angular_threshold);
      }

      /** \brief Get the angular threshold in radians for difference in normal direction between neighboring points, to be considered part of the same plane. */
      [[deprecated("EuclideadClusterComparator never actually used normals and angular threshold, this function has no effect on the behavior of the comparator. It is deprecated and will be removed in future releases.")]]
      inline float
      getAngularThreshold () const { return (std::acos (angular_threshold_) ); }

      /** \brief Set labels in the label cloud to exclude.
        * \param[in] exclude_labels a vector of bools corresponding to whether or not a given label should be considered
        */
      [[deprecated("use setExcludeLabels(const ExcludeLabelSetConstPtr &) instead")]]
      void
      setExcludeLabels (const std::vector<bool>& exclude_labels)
      {
        exclude_labels_ = boost::make_shared<std::set<std::uint32_t> > ();
        for (std::size_t i = 0; i < exclude_labels.size (); ++i)
          if (exclude_labels[i])
            exclude_labels_->insert (i);
      }

    protected:

      PointCloudNConstPtr normals_;

      float angular_threshold_;
  };

  template<typename PointT, typename PointLT>
  class EuclideanClusterComparator<PointT, PointLT, deprecated::T>
    : public experimental::EuclideanClusterComparator<PointT, PointLT> {};
}
