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
 * $Id$
 *
 */

#pragma once

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_types.h>

namespace pcl {
namespace registration {
/** \brief @b CorrespondenceEstimationNormalShooting computes
 * correspondences as points in the target cloud which have minimum
 * distance to normals computed on the input cloud
 *
 * Code example:
 *
 * \code
 * pcl::PointCloud<pcl::PointNormal>::Ptr source, target;
 * // ... read or fill in source and target
 * pcl::CorrespondenceEstimationNormalShooting
 *   <pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
 * est.setInputSource (source);
 * est.setSourceNormals (source);
 *
 * est.setInputTarget (target);
 *
 * // Test the first 10 correspondences for each point in source, and return the best
 * est.setKSearch (10);
 *
 * pcl::Correspondences all_correspondences;
 * // Determine all correspondences
 * est.determineCorrespondences (all_correspondences);
 * \endcode
 *
 * \author Aravindhan K. Krishnan, Radu B. Rusu
 * \ingroup registration
 */
template <typename PointSource,
          typename PointTarget,
          typename NormalT,
          typename Scalar = float>
class CorrespondenceEstimationNormalShooting
: public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> {
public:
  using Ptr = shared_ptr<CorrespondenceEstimationNormalShooting<PointSource,
                                                                PointTarget,
                                                                NormalT,
                                                                Scalar>>;
  using ConstPtr = shared_ptr<const CorrespondenceEstimationNormalShooting<PointSource,
                                                                           PointTarget,
                                                                           NormalT,
                                                                           Scalar>>;

  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::
      initComputeReciprocal;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::
      input_transformed_;
  using PCLBase<PointSource>::deinitCompute;
  using PCLBase<PointSource>::input_;
  using PCLBase<PointSource>::indices_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::
      point_representation_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;

  using KdTree = pcl::search::KdTree<PointTarget>;
  using KdTreePtr = typename KdTree::Ptr;

  using PointCloudSource = pcl::PointCloud<PointSource>;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = pcl::PointCloud<PointTarget>;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using PointCloudNormals = pcl::PointCloud<NormalT>;
  using NormalsPtr = typename PointCloudNormals::Ptr;
  using NormalsConstPtr = typename PointCloudNormals::ConstPtr;

  /** \brief Empty constructor.
   *
   * \note
   * Sets the number of neighbors to be considered in the target point cloud (k_) to 10.
   */
  CorrespondenceEstimationNormalShooting()
  : source_normals_(), source_normals_transformed_(), k_(10)
  {
    corr_name_ = "CorrespondenceEstimationNormalShooting";
  }

  /** \brief Empty destructor */
  ~CorrespondenceEstimationNormalShooting() {}

  /** \brief Set the normals computed on the source point cloud
   * \param[in] normals the normals computed for the source cloud
   */
  inline void
  setSourceNormals(const NormalsConstPtr& normals)
  {
    source_normals_ = normals;
  }

  /** \brief Get the normals of the source point cloud
   */
  inline NormalsConstPtr
  getSourceNormals() const
  {
    return (source_normals_);
  }

  /** \brief See if this rejector requires source normals */
  bool
  requiresSourceNormals() const override
  {
    return (true);
  }

  /** \brief Blob method for setting the source normals */
  void
  setSourceNormals(pcl::PCLPointCloud2::ConstPtr cloud2) override
  {
    NormalsPtr cloud(new PointCloudNormals);
    fromPCLPointCloud2(*cloud2, *cloud);
    setSourceNormals(cloud);
  }

  /** \brief Determine the correspondences between input and target cloud.
   * \param[out] correspondences the found correspondences (index of query point, index
   * of target point, distance) \param[in] max_distance maximum distance between the
   * normal on the source point cloud and the corresponding point in the target point
   * cloud
   */
  void
  determineCorrespondences(
      pcl::Correspondences& correspondences,
      double max_distance = std::numeric_limits<double>::max()) override;

  /** \brief Determine the reciprocal correspondences between input and target cloud.
   * A correspondence is considered reciprocal if both Src_i has Tgt_i as a
   * correspondence, and Tgt_i has Src_i as one.
   *
   * \param[out] correspondences the found correspondences (index of query and target
   * point, distance) \param[in] max_distance maximum allowed distance between
   * correspondences
   */
  void
  determineReciprocalCorrespondences(
      pcl::Correspondences& correspondences,
      double max_distance = std::numeric_limits<double>::max()) override;

  /** \brief Set the number of nearest neighbours to be considered in the target
   * point cloud. By default, we use k = 10 nearest neighbors.
   *
   * \param[in] k the number of nearest neighbours to be considered
   */
  inline void
  setKSearch(unsigned int k)
  {
    k_ = k;
  }

  /** \brief Get the number of nearest neighbours considered in the target point
   * cloud for computing correspondences. By default we use k = 10 nearest
   * neighbors.
   */
  inline unsigned int
  getKSearch() const
  {
    return (k_);
  }

  /** \brief Clone and cast to CorrespondenceEstimationBase */
  typename CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::Ptr
  clone() const override
  {
    Ptr copy(new CorrespondenceEstimationNormalShooting<PointSource,
                                                        PointTarget,
                                                        NormalT,
                                                        Scalar>(*this));
    return (copy);
  }

protected:
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::
      tree_reciprocal_;
  using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;

  /** \brief Internal computation initialization. */
  bool
  initCompute();

private:
  /** \brief The normals computed at each point in the source cloud */
  NormalsConstPtr source_normals_;

  /** \brief The normals computed at each point in the source cloud */
  NormalsPtr source_normals_transformed_;

  /** \brief The number of neighbours to be considered in the target point cloud */
  unsigned int k_;
};
} // namespace registration
} // namespace pcl

#include <pcl/registration/impl/correspondence_estimation_normal_shooting.hpp>
