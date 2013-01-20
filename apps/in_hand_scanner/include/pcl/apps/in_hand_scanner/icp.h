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
    /** \brief Iterative Closest Point registration.
      * \author Martin Saelzle
      * \ingroup apps
      */
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

        /** \brief Constructor */
        ICP ();

        /** @{ */
        /** \brief Convergence is detected when the change of the fitness between the current and previous iteration becomes smaller than the given epsilon (set in cm^2). The fitness is the mean squared euclidean distance between corresponding points.
          * \note Only accepted if it is greater than 0.
          */
        void
        setEpsilon (const float epsilon);

        float
        getEpsilon () const;
        /** @} */

        /** @{ */
        /** \brief The registration fails if the number of iterations exceeds the maximum number of iterations.
          * \note Must be greater than 0. Smaller values are set to 1.
          */
        void
        setMaxIterations (const unsigned int max_iter);

        unsigned int
        getMaxIterations () const;
        /** @} */

        /** @{ */
        /** \brief The registration fails at the state of convergence if the overlap between the model and data shape is smaller than a minimum overlap. The overlap is the fraction of correspondences (after rejection) to the initial number of data points.
          * \note Must be between zero and one. Values outside this range are clamped to the nearest valid value.
          */
        void
        setMinOverlap (const float overlap);

        float
        getMinOverlap () const;
        /** @} */

        /** @{ */
        /** \brief The registration fails at the state of convergence if the fitness is bigger than this threshold (set in cm^2)
          * \note Must be greater than zero.
          */
        void
        setMaxFitness (const float fitness);

        float
        getMaxFitness () const;
        /** @} */

        /** @{ */
        /** \brief Correspondences are rejected if the squared distance is above a threshold. This threshold is initialized with infinity (all correspondences are accepted in the first iteration). The threshold of the next iterations is set to the fitness of the current iteration multiplied by the factor set by this method.
          * \note Must be greater or equal one. Smaller values are set to one.
          */
        void
        setCorrespondenceRejectionFactor (const float factor);

        float
        getCorrespondenceRejectionFactor () const;
        /** @} */

        /** @{ */
        /** \brief Correspondences are rejected if the angle between the normals is bigger than this threshold. Set in degrees.
          * \note Must be between 180 degrees and 0. Values outside this range are clamped to the nearest valid value.
          */
        void
        setMaxAngle (const float angle);

        float
        getMaxAngle () const;
        /** @} */

        /** \brief Find the transformation that aligns the data cloud (source) to the model mesh (target).
          * \param[in] mesh_model Model mesh (target).
          * \param[in] cloud_data Data cloud (source).
          * \param[in] T_init Initial guess for the transformation.
          * \paran[out] T_final The computed transformation.
          * \return true if success.
          */
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

        /** \brief Selects the model points that are pointing towards to the camera (data coordinate system = camera coordinate system).
          * \param[in] mesh_model Input mesh.
          * \param[in] T_inv Transformation that brings the model mesh into the camera coordinate system.
          * \return Cloud containing the selected points (the connectivity information of the mesh is currently not used during the registration).
          */
        CloudNormalConstPtr
        selectModelPoints (const MeshConstPtr&    mesh_model,
                           const Eigen::Matrix4f& T_inv) const;

        /** \brief Selects the valid data points. The input cloud is organized -> contains nans which are removed
          * \param[in] cloud_data Input cloud.
          * \return Cloud containing the selected points.
          */
        CloudNormalConstPtr
        selectDataPoints (const CloudXYZRGBNormalConstPtr& cloud_data) const;

        /** \brief Finds the transformation that minimizes the point to plane distance from the source to the target cloud. The input clouds must be arranged to have one to one correspondences (point 0 in source corresponds to point 0 in target, point 1 in source to point 1 in target and so on).
          * \param[in] cloud_source Source cloud (data).
          * \param[in] cloud_target Target cloud (model).
          * \param[out] T The computed transformation.
          * \return true if success
          */
        bool
        minimizePointPlane (const CloudNormal& cloud_source,
                            const CloudNormal& cloud_target,
                            Eigen::Matrix4f&   T) const;

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        KdTreePtr kd_tree_;

        // Convergence
        float epsilon_; // in cm^2

        // Registration failure
        unsigned int max_iterations_;
        float min_overlap_; // [0 1]
        float max_fitness_; // in cm^2

        // Correspondence rejection
        float factor_;
        float max_angle_; // in degrees
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_ICP_H
