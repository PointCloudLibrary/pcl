/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Jin Wu, Ming Liu, Yilong Zhu
 *                      Hong Kong University of Science and Technology (HKUST)
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

#pragma once

#include <pcl/registration/transformation_estimation.h>
#include <pcl/cloud_iterator.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationEIG implements eigendecomposition-based (EIG) estimation of
      * the transformation aligning the given correspondences.
      * Besl, P., & McKay, N. (1992). A Method for Registration of 3-D Shapes. 
      *              IEEE Transactions on Pattern Analysis and Machine Intelligence. https://doi.org/10.1109/34.121791
      *
      * It also include a recent improvement on the computational efficiency by over 50%:
      * Wu, J., Liu, M., Zhou, Z., & Li, R. (2020). Fast Symbolic 3-D Registration Solution. 
      *              IEEE Transactions on Automation Science and Engineering, 17(2), 761–770. https://doi.org/10.1109/TASE.2019.2942324
      * 
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Jin Wu
      * \author Ming Liu
      * \author Yilong Zhu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class TransformationEstimationEIG : public TransformationEstimation<PointSource, PointTarget, Scalar>
    {
    public:
        using Ptr = shared_ptr<TransformationEstimationEIG<PointSource, PointTarget, Scalar> >;
        using ConstPtr = shared_ptr<const TransformationEstimationEIG<PointSource, PointTarget, Scalar> >;

        using Matrix4 = typename TransformationEstimation<PointSource, PointTarget, Scalar>::Matrix4;
        using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

      /** \brief Constructor
      */
      TransformationEstimationEIG ( ) {}

      virtual ~TransformationEstimationEIG () {};


      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud 
        *        using EIG or Fast Symbolic 3D Registration (FS3R), which is a variant of EIG.
        * \param[in] cloud_src the source point cloud dataset
        * \param[in] cloud_tgt the target point cloud dataset
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      inline void
        estimateRigidTransformation (
          const pcl::PointCloud<PointSource> &cloud_src,
          const pcl::PointCloud<PointTarget> &cloud_tgt,
          Matrix4 &transformation_matrix) const;


      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using EIG.
        * \param[in] cloud_src the source point cloud dataset
        * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
        * \param[in] cloud_tgt the target point cloud dataset
        * \param[in] use_fs3r enables the use of FS3R
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      inline void
        estimateRigidTransformation (
          const pcl::PointCloud<PointSource> &cloud_src,
          const std::vector<int> &indices_src,
          const pcl::PointCloud<PointTarget> &cloud_tgt,
          Matrix4 &transformation_matrix) const;

      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using EIG.
        * \param[in] cloud_src the source point cloud dataset
        * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
        * \param[in] cloud_tgt the target point cloud dataset
        * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from \a indices_src
        * \param[in] use_fs3r enables the use of FS3R
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      inline void
        estimateRigidTransformation (
          const pcl::PointCloud<PointSource> &cloud_src,
          const std::vector<int> &indices_src,
          const pcl::PointCloud<PointTarget> &cloud_tgt,
          const std::vector<int> &indices_tgt,
          Matrix4 &transformation_matrix) const;

      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using EIG.
        * \param[in] cloud_src the source point cloud dataset
        * \param[in] cloud_tgt the target point cloud dataset
        * \param[in] correspondences the vector of correspondences between source and target point cloud
        * \param[in] use_fs3r enables the use of FS3R
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      void
        estimateRigidTransformation (
          const pcl::PointCloud<PointSource> &cloud_src,
          const pcl::PointCloud<PointTarget> &cloud_tgt,
          const pcl::Correspondences &correspondences,
          Matrix4 &transformation_matrix) const;

    protected:

      /** \brief Estimate a rigid rotation transformation between a source and a target
        * \param[in] source_it an iterator over the source point cloud dataset
        * \param[in] target_it an iterator over the target point cloud dataset
        * \param[in] use_fs3r enables the use of FS3R
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      void
        estimateRigidTransformation ( ConstCloudIterator<PointSource> &source_it,
          ConstCloudIterator<PointTarget> &target_it,
          Matrix4 &transformation_matrix) const;

    };
    bool use_fs3r = true;
  }
}

#include <pcl/registration/impl/transformation_estimation_eig.hpp>

