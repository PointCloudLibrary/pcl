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
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationLM implements Levenberg Marquardt-based
      * estimation of the transformation aligning the given correspondences.
      *
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class TransformationEstimationLM : public TransformationEstimation<PointSource, PointTarget>
    {
      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      public:
        TransformationEstimationLM () {};
        virtual ~TransformationEstimationLM () {};

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Eigen::Matrix4f &transformation_matrix);

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
         * \param[in] cloud_src the source point cloud dataset
         * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
         * \param[in] cloud_tgt the target point cloud dataset
         * \param[out] transformation_matrix the resultant transformation matrix
         */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Eigen::Matrix4f &transformation_matrix);

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
         * \param[in] cloud_src the source point cloud dataset
         * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
         * \param[in] cloud_tgt the target point cloud dataset
         * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
         * \param[out] transformation_matrix the resultant transformation matrix
         */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Eigen::Matrix4f &transformation_matrix);

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
         * \param[in] cloud_src the source point cloud dataset
         * \param[in] cloud_tgt the target point cloud dataset
         * \param[in] correspondences the vector of correspondences between source and target point cloud
         * \param[out] transformation_matrix the resultant transformation matrix
         */
        inline void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<pcl::Correspondence> &correspondences,
            Eigen::Matrix4f &transformation_matrix);

        /** \brief Set the function we use to warp points. Defaults to rigid 6D warp.
          * \param[in] shared pointer to object that warps points
          */
        void
        setWarpFunction (const boost::shared_ptr<WarpPointRigid<PointSource, PointTarget> > &warp_fcn)
        {
          warp_point_ = warp_fcn;
        }

      private:
        /** \brief Cost function to be minimized
          * \param[in] p a pointer to our data structure array
          * \param[in] m the number of functions
          * \param[in] n the number of variables
          * \param[out] x a pointer to the variables array
          * \param[out] fvec a pointer to the resultant functions evaluations
          * \param[in] iflag set to -1 inside the function to terminate execution
          */
        static int 
        functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);

        /** \brief Cost function to be minimized
          * \param[in] p a pointer to our data structure array
          * \param[in] m the number of functions
          * \param[in] n the number of variables
          * \param[out] x a pointer to the variables array
          * \param[out] fvec a pointer to the resultant functions evaluations
          * \param[in] iflag set to -1 inside the function to terminate execution
          */
        static int 
        functionToOptimizeIndices (void *p, int m, int n, const double *x, double *fvec, int iflag);

        /** \brief Compute the median value from a set of doubles
          * \param[in] fvec the set of doubles
          * \param[in] m the number of doubles in the set
          */
        inline double 
        computeMedian (double *fvec, int m);

        /** \brief Use a Huber kernel to estimate the distance between two vectors
          * \param[in] p_src the first eigen vector
          * \param[in] p_tgt the second eigen vector
          * \param[in] sigma the sigma value
          */
        inline double
        distHuber (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt, double sigma)
        {
          Eigen::Array4f diff = (p_tgt.array () - p_src.array ()).abs ();
          double norm = 0.0;
          for (int i = 0; i < 3; ++i)
          {
            if (diff[i] < sigma)
              norm += diff[i] * diff[i];
            else
              norm += 2.0 * sigma * diff[i] - sigma * sigma;
          }
          return (norm);
        }

        /** \brief Use a Huber kernel to estimate the distance between two vectors
          * \param[in] diff the norm difference between two vectors
          * \param[in] sigma the sigma value
          */
        inline double
        distHuber (double diff, double sigma)
        {
          double norm = 0.0;
          if (diff < sigma)
            norm += diff * diff;
          else
            norm += 2.0 * sigma * diff - sigma * sigma;
          return (norm);
        }

        /** \brief Use a Gedikli kernel to estimate the distance between two vectors
          * (for more information, see 
          * \param[in] val the norm difference between two vectors
          * \param[in] clipping the clipping value
          */
        inline double
        distGedikli (double val, double clipping)
        {
          clipping *= 1.5;
          return (1.0 / (1.0 + pow (val / clipping, 4)));
        }

        /** \brief Compute the Manhattan distance between two eigen vectors.
          * \param[in] p_src the first eigen vector
          * \param[in] p_tgt the second eigen vector
          */
        inline double
        distL1 (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt)
        {
          return ((p_src.array () - p_tgt.array ()).abs ().sum ());
        }

        /** \brief Compute the squared Euclidean distance between two eigen vectors.
          * \param[in] p_src the first eigen vector
          * \param[in] p_tgt the second eigen vector
          */
        inline double
        distL2Sqr (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt)
        {
          return ((p_src - p_tgt).squaredNorm ());
        }

        /** \brief The vector of residual weights. Used internall in the LM loop. */
        std::vector<double> weights_;

        /** \brief Temporary pointer to the source dataset. */
        const PointCloudSource *tmp_src_;

        /** \brief Temporary pointer to the target dataset. */
        const PointCloudTarget  *tmp_tgt_;

        /** \brief Temporary pointer to the source dataset indices. */
        const std::vector<int> *tmp_idx_src_;

        /** \brief Temporary pointer to the target dataset indices. */
        const std::vector<int> *tmp_idx_tgt_;

        /** \brief Temporary pointer to the target dataset indices. */
        boost::shared_ptr<WarpPointRigid<PointSource, PointTarget> > warp_point_;
    };
  }
}

#include <pcl/registration/impl/transformation_estimation_lm.hpp>

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_LM_H_ */

