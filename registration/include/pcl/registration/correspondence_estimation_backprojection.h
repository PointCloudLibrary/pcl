/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_BACK_PROJECTION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_BACK_PROJECTION_H_

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b CorrespondenceEstimationBackprojection computes
      * correspondences as points in the target cloud which have minimum
      * \author Suat Gedikli
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename NormalT>
    class CorrespondenceEstimationBackProjection : public CorrespondenceEstimationBase <PointSource, PointTarget>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimationBackProjection<PointSource, PointTarget, NormalT> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimationBackProjection<PointSource, PointTarget, NormalT> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget>::getClassName;
        using CorrespondenceEstimationBase<PointSource, PointTarget>::point_representation_;
        using CorrespondenceEstimationBase<PointSource, PointTarget>::target_indices_;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename pcl::PointCloud<NormalT>::Ptr NormalsPtr;
				typedef typename pcl::PointCloud<NormalT>::ConstPtr NormalsConstPtr;

        /** \brief Empty constructor. 
          *
          * \note
          * Sets the number of neighbors to be considered in the target point cloud (k_) to 10.
          */
        CorrespondenceEstimationBackProjection ()
          : source_normals_ ()
          , target_normals_ ()
          , k_ (10)
        {
          corr_name_ = "CorrespondenceEstimationNormalShooting";
        }

        /** \brief Set the normals computed on the source point cloud
          * \param[in] normals the normals computed for the source cloud
          */
        inline void
        setSourceNormals (const NormalsConstPtr &normals) { source_normals_ = normals; }

        /** \brief Get the normals of the source point cloud
          */
        inline NormalsConstPtr
        getSourceNormals () const { return (source_normals_); }

        /** \brief Set the normals computed on the target point cloud
          * \param[in] normals the normals computed for the target cloud
          */
        inline void
        setTargetNormals (const NormalsConstPtr &normals) { target_normals_ = normals; }

        /** \brief Get the normals of the target point cloud
          */
        inline NormalsConstPtr
        getTargetNormals () const { return (target_normals_); }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum distance between the normal on the source point cloud and the corresponding point in the target
          * point cloud
          */
        void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ());

        /** \brief Set the number of nearest neighbours to be considered in the target 
          * point cloud. By default, we use k = 10 nearest neighbors.
          *
          * \param[in] k the number of nearest neighbours to be considered
          */
        inline void
        setKSearch (unsigned int k) { k_ = k; }

        /** \brief Get the number of nearest neighbours considered in the target point 
          * cloud for computing correspondences. By default we use k = 10 nearest 
          * neighbors.
          */
        inline void
        getKSearch () const { return (k_); }

        /** \brief Return true if the source normals are needed for correspondence estimation. */
        inline bool
        needsSourceNormals ()
        {
          return (true);
        }

        /** \brief Return true if the target normals are needed for correspondence estimation. */
        inline bool
        needsTargetNormals ()
        {
          return (true);
        }

      protected:

        using CorrespondenceEstimationBase<PointSource, PointTarget>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget>::tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget>::target_;

        /** \brief Internal computation initalization. */
        bool
        initCompute ();

       private:

        /** \brief The normals computed at each point in the source cloud */
        NormalsConstPtr source_normals_; 

        /** \brief The normals computed at each point in the target cloud */
        NormalsConstPtr target_normals_; 

        /** \brief The number of neighbours to be considered in the target point cloud */
        unsigned int k_;
    };
  }
}

#include <pcl/registration/impl/correspondence_estimation_backprojection.hpp>
#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_BACK_PROJECTION_H_ */
