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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_H_
#define PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_H_

#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_validation.h>

namespace pcl
{
  namespace registration
  {
    /** \brief TransformationValidationEuclidean computes an L2SQR norm between a source and target
      * dataset.
      * 
      * To prevent points with bad correspondences to contribute to the overall score, the class also 
      * accepts a maximum_range parameter given via \ref setMaxRange that is used as a cutoff value for
      * nearest neighbor distance comparisons.
      * 
      * The output score is normalized with respect to the number of valid correspondences found.
      *
      * Usage example:
      * \code
      * pcl::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> tve;
      * tve.setMaxRange (0.01);  // 1cm
      * double score = tve.validateTransformation (source, target, transformation);
      * \endcode
      *
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class TransformationValidationEuclidean
    {
      public:
        typedef boost::shared_ptr<TransformationValidation<PointSource, PointTarget> > Ptr;
        typedef boost::shared_ptr<const TransformationValidation<PointSource, PointTarget> > ConstPtr;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        typedef typename TransformationValidation<PointSource, PointTarget>::PointCloudSourceConstPtr PointCloudSourceConstPtr;
        typedef typename TransformationValidation<PointSource, PointTarget>::PointCloudTargetConstPtr PointCloudTargetConstPtr;

        /** \brief Constructor.
          * Sets the \a max_range parameter to double::max, and initializes the internal search \a tree
          * to a FLANN kd-tree.
          */
        TransformationValidationEuclidean () : 
          max_range_ (std::numeric_limits<double>::max ()),
          tree_ (new pcl::KdTreeFLANN<PointTarget>)
        {
        }

        virtual ~TransformationValidationEuclidean () {};

        /** \brief Set the maximum allowable distance between a point and its correspondence in the 
          * target in order for a correspondence to be considered \a valid. Default: double::max.
          * \param[in] max_range the new maximum allowable distance
          */
        inline void
        setMaxRange (double max_range)
        {
          max_range_ = max_range;
        }

        /** \brief Validate the given transformation with respect to the input cloud data, and return a score.
          *
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          *
          * \return the score or confidence measure for the given
          * transformation_matrix with respect to the input data
          */
        double
        validateTransformation (
            const PointCloudSourceConstPtr &cloud_src,
            const PointCloudTargetConstPtr &cloud_tgt,
            const Eigen::Matrix4f &transformation_matrix);

      protected:
        /** \brief The maximum allowable distance between a point and its correspondence in the target 
          * in order for a correspondence to be considered \a valid. Default: double::max.
          */
        double max_range_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#include <pcl/registration/impl/transformation_validation_euclidean.hpp>

#endif /* PCL_REGISTRATION_TRANSFORMATION_VALIDATION_EUCLIDEAN_H_ */
