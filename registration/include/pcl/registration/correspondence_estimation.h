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
      * \author Radu Bogdan Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class CorrespondenceEstimation : public PCLBase<PointSource>
    {
      public:
        using PCLBase<PointSource>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;

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
        CorrespondenceEstimation () : 
          corr_name_ (),
          tree_ (new pcl::KdTreeFLANN<PointTarget>),
          target_ (),
          point_representation_ ()
        {
        }

        /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the 
          * input source to)
          * \param[in] cloud the input point cloud target
          */
        virtual inline void 
        setInputTarget (const PointCloudTargetConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudTargetConstPtr const 
        getInputTarget () { return (target_ ); }

        /** \brief Provide a boost shared pointer to the PointRepresentation to be used when comparing points
          * \param[in] point_representation the PointRepresentation to be used by the k-D tree
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  float max_distance = std::numeric_limits<float>::max ());

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences);

      protected:
        /** \brief The correspondence estimation method name. */
        std::string corr_name_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** \brief The input point cloud dataset target. */
        PointCloudTargetConstPtr target_;

        /** \brief Abstract class get name method. */
        inline const std::string& 
        getClassName () const { return (corr_name_); }

      private:
        /** \brief The point representation used (internal). */
        PointRepresentationConstPtr point_representation_;
     };
  }
}

#include <pcl/registration/impl/correspondence_estimation.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_ */
