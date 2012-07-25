/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b CorrespondenceEstimation represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class CorrespondenceEstimation : public PCLBase<PointSource>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimation<PointSource, PointTarget> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimation<PointSource, PointTarget> > ConstPtr;
        using PCLBase<PointSource>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;
        using PCLBase<PointSource>::setIndices;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimation () 
          : corr_name_ ("CorrespondenceEstimation")
          , tree_ (new pcl::KdTreeFLANN<PointTarget>)
          , target_ ()
          , target_indices_ ()
          , point_representation_ ()
        {
        }

        /** \brief Provide a pointer to the input source 
          * (e.g., the point cloud that we want to align to the target)
          *
          * \param[in] cloud the input point cloud source
          */
        inline void 
        setInputSource (const PointCloudSourceConstPtr &cloud)
        {
          setInputCloud (cloud);
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudSourceConstPtr const 
        getInputSource () { return (input_ ); }

        /** \brief Provide a pointer to the input target 
          * (e.g., the point cloud that we want to align the input source to)
          * \param[in] cloud the input point cloud target
          */
        inline void 
        setInputTarget (const PointCloudTargetConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudTargetConstPtr const 
        getInputTarget () { return (target_ ); }

        /** \brief Provide a pointer to the vector of indices that represent the 
          * input source point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesSource (const IndicesPtr &indices)
        {
          setIndices (indices);
        }

        /** \brief Get a pointer to the vector of indices used for the source dataset. */
        inline IndicesPtr const 
        getIndicesSource () { return (indices_); }

        /** \brief Provide a pointer to the vector of indices that represent the 
          * input target point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesTarget (const IndicesPtr &indices)
        {
          target_indices_ = indices;
        }

        /** \brief Get a pointer to the vector of indices used for the target dataset. */
        inline IndicesPtr const 
        getIndicesTarget () { return (target_indices_); }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
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

        /** \brief Provide a boost shared pointer to the PointRepresentation to be used 
          * when searching for nearest neighbors.
          *
          * \param[in] point_representation the PointRepresentation to be used by the 
          * k-D tree for nearest neighbor search
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
        }

      protected:
        /** \brief The correspondence estimation method name. */
        std::string corr_name_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** \brief The input point cloud dataset target. */
        PointCloudTargetConstPtr target_;

        /** \brief The target point cloud dataset indices. */
        IndicesPtr target_indices_;

        /** \brief The point representation used (internal). */
        PointRepresentationConstPtr point_representation_;

        /** \brief Abstract class get name method. */
        inline const std::string& 
        getClassName () const { return (corr_name_); }

        /** \brief Internal computation initalization. */
        bool
        initCompute ();
     };
  }
}

#include <pcl/registration/impl/correspondence_estimation.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_ */
