/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b CorrespondenceEstimationNormalShooting computes correspondences as points in the target cloud which
      *  have minimum distance to normals computed on the input cloud
      * \author Aravindhan K Krishnan
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename NormalT>
    class CorrespondenceEstimationNormalShooting : public CorrespondenceEstimation <PointSource, PointTarget>
    {
      public:
        using PCLBase<PointSource>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;
        using CorrespondenceEstimation<PointSource, PointTarget>::getClassName;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;
        typedef typename pcl::PointCloud<NormalT>::Ptr NormalsPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimationNormalShooting ()
        {
          corr_name_ = "NormalShooting";
        }

        /** \brief Set the normals computed on the input point cloud
          * \param[in] normals the normals computed for the input cloud
          */
        inline void
        setSourceNormals (const NormalsPtr &normals) { source_normals_ = normals; }

        /** \brief Get the normals of the input point cloud
          */
        inline NormalsPtr
        getSourceNormals () const { return (source_normals_); }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum distance between the normal on the source point cloud and the corresponding point in the target
          * point cloud
          */
        void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  float max_distance = std::numeric_limits<float>::max ());

        /** \brief Set the number of nearest neighbours to be considered in the target point cloud
          * \param[in] k the number of nearest neighbours to be considered
          */
        inline void
        setKSearch (unsigned int k) { k_ = k; }

        /** \brief Get the number of nearest neighbours considered in the target point cloud for computing correspondence
          */
        inline void
        getKSearch () const { return (k_); }

      protected:

        using CorrespondenceEstimation<PointSource, PointTarget>::corr_name_;
        using CorrespondenceEstimation<PointSource, PointTarget>::tree_;
        using CorrespondenceEstimation<PointSource, PointTarget>::target_;

      private:

        /** \brief The normals computed at each point in the input cloud */
        NormalsPtr source_normals_; 

        /** \brief The number of neighbours to be considered in the target point cloud */
        unsigned int k_;
    };
  }
}

#include <pcl/registration/impl/correspondence_estimation_normal_shooting.hpp>
#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_NORMAL_SHOOTING_H_ */
