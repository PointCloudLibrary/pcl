/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#pragma once

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/gicp.h>
#include <pcl/memory.h>
#include <pcl/pcl_exports.h> // for PCL_EXPORTS
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>

namespace pcl {
/** \brief GeneralizedIterativeClosestPoint6D integrates L*a*b* color space information
 * into the Generalized Iterative Closest Point (GICP) algorithm.
 *
 * The suggested input is PointXYZRGBA.
 *
 * \note If you use this code in any academic work, please cite:
 *
 * - M. Korn, M. Holzkothen, J. Pauli
 * Color Supported Generalized-ICP.
 * In Proceedings of VISAPP 2014 - International Conference on Computer Vision Theory
 * and Applications, Lisbon, Portugal, January 2014.
 *
 * \author Martin Holzkothen, Michael Korn
 * \ingroup registration
 */
class PCL_EXPORTS GeneralizedIterativeClosestPoint6D
: public GeneralizedIterativeClosestPoint<PointXYZRGBA, PointXYZRGBA> {
  using PointSource = PointXYZRGBA;
  using PointTarget = PointXYZRGBA;

public:
  /** \brief constructor.
   *
   * \param[in] lab_weight the color weight
   */
  GeneralizedIterativeClosestPoint6D(float lab_weight = 0.032f);

  /** \brief Provide a pointer to the input source
   * (e.g., the point cloud that we want to align to the target)
   *
   * \param[in] cloud the input point cloud source
   */
  void
  setInputSource(const PointCloudSourceConstPtr& cloud) override;

  /** \brief Provide a pointer to the input target
   * (e.g., the point cloud that we want to align the input source to)
   *
   * \param[in] target the input point cloud target
   */
  void
  setInputTarget(const PointCloudTargetConstPtr& target) override;

protected:
  /** \brief Rigid transformation computation method  with initial guess.
   * \param output the transformed input point cloud dataset using the rigid
   * transformation found \param guess the initial guess of the transformation to
   * compute
   */
  void
  computeTransformation(PointCloudSource& output,
                        const Eigen::Matrix4f& guess) override;

  /** \brief Search for the closest nearest neighbor of a given point.
   * \param query the point to search a nearest neighbour for
   * \param index vector of size 1 to store the index of the nearest neighbour found
   * \param distance vector of size 1 to store the distance to nearest neighbour found
   */
  inline bool
  searchForNeighbors(const PointXYZLAB& query,
                     pcl::Indices& index,
                     std::vector<float>& distance);

protected:
  /** \brief Holds the converted (LAB) data cloud. */
  pcl::PointCloud<PointXYZLAB>::Ptr cloud_lab_;

  /** \brief Holds the converted (LAB) model cloud. */
  pcl::PointCloud<PointXYZLAB>::Ptr target_lab_;

  /** \brief 6d-tree to search in model cloud. */
  KdTreeFLANN<PointXYZLAB> target_tree_lab_;

  /** \brief The color weight. */
  float lab_weight_;

  /**  \brief Custom point representation to perform kdtree searches in more than 3
   * (i.e. in all 6) dimensions. */
  class MyPointRepresentation : public PointRepresentation<PointXYZLAB> {
    using PointRepresentation<PointXYZLAB>::nr_dimensions_;
    using PointRepresentation<PointXYZLAB>::trivial_;

  public:
    using Ptr = shared_ptr<MyPointRepresentation>;
    using ConstPtr = shared_ptr<const MyPointRepresentation>;

    MyPointRepresentation()
    {
      nr_dimensions_ = 6;
      trivial_ = false;
    }

    ~MyPointRepresentation() {}

    inline Ptr
    makeShared() const
    {
      return Ptr(new MyPointRepresentation(*this));
    }

    void
    copyToFloatArray(const PointXYZLAB& p, float* out) const override
    {
      // copy all of the six values
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
      out[3] = p.L;
      out[4] = p.a;
      out[5] = p.b;
    }
  };

  /** \brief Enables 6d searches with kd-tree class using the color weight. */
  MyPointRepresentation point_rep_;
};
} // namespace pcl
