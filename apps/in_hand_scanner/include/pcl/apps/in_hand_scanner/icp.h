/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_APPS_IN_HAND_SCANNER_ICP_H
#define PCL_APPS_IN_HAND_SCANNER_ICP_H

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/boost.h>
#include <pcl/apps/in_hand_scanner/eigen.h>
#include <pcl/apps/in_hand_scanner/common_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <typename PointT>
  class KdTree;
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// ICP
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    class PCL_EXPORTS ICP
    {
      public:

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::ihs::Mesh         Mesh;
        typedef pcl::ihs::MeshPtr      MeshPtr;
        typedef pcl::ihs::MeshConstPtr MeshConstPtr;

        ICP ();

        bool
        findTransformation (const MeshConstPtr&              mesh_model,
                            const CloudXYZRGBNormalConstPtr& cloud_data,
                            const Eigen::Matrix4f&           T_init,
                            Eigen::Matrix4f&                 T_final);

      private:

        typedef pcl::PointNormal              PointNormal;
        typedef pcl::PointCloud <PointNormal> CloudNormal;
        typedef CloudNormal::Ptr              CloudNormalPtr;
        typedef CloudNormal::ConstPtr         CloudNormalConstPtr;

        typedef pcl::KdTree <PointNormal>        KdTree;
        typedef boost::shared_ptr <KdTree>       KdTreePtr;
        typedef boost::shared_ptr <const KdTree> KdTreeConstPtr;


        CloudNormalConstPtr
        selectModelPoints (const MeshConstPtr&    mesh_model,
                           const Eigen::Matrix4f& T_init_inv) const;

        CloudNormalConstPtr
        selectDataPoints (const CloudXYZRGBNormalConstPtr& cloud_data) const;

        bool
        minimizePointPlane (const CloudNormalConstPtr& cloud_source,
                            const CloudNormalConstPtr& cloud_target,
                            Eigen::Matrix4f&           T) const;

        // Nearest neighbor search
        KdTreePtr    kd_tree_;

        // Convergence
        float        epsilon_; // in m^2

        // Registration failure
        unsigned int max_iterations_;
        float        min_overlap_; // [0 1]
        float        max_fitness_; // in m^2

        // Correspondence rejection
        float        squared_distance_threshold_factor_;
        float        normals_threshold_; // cos(angle)
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_ICP_H
