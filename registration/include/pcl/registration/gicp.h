/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_GICP_H_
#define PCL_GICP_H_

// PCL includes
#include "pcl/registration/icp.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_registration.h"

#include <cminpack.h>

namespace pcl
{
  /** \brief GeneralizedIterativeClosestPoint is an ICP variant that implements the 
    * generalized iterative closest point algorithm as described by Alex Segal et al. in 
    * http://www.stanford.edu/~avsegal/resources/papers/Generalized_ICP.pdf
    * The approach is based on using anistropic cost functions to optimize the alignment 
    * after closest point assignments have been made.
    * The original code uses GSL and ANN while in ours BFGS is replaced with a LM and 
    * ANN with FLANN.
    * \author Nizar Sallem
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget>
  class GeneralizedIterativeClosestPoint : public IterativeClosestPoint<PointSource, PointTarget>
  {
    using IterativeClosestPoint<PointSource, PointTarget>::reg_name_;
    using IterativeClosestPoint<PointSource, PointTarget>::getClassName;
    using IterativeClosestPoint<PointSource, PointTarget>::indices_;
    using IterativeClosestPoint<PointSource, PointTarget>::target_;
    using IterativeClosestPoint<PointSource, PointTarget>::input_;
    using IterativeClosestPoint<PointSource, PointTarget>::tree_;
    using IterativeClosestPoint<PointSource, PointTarget>::nr_iterations_;
    using IterativeClosestPoint<PointSource, PointTarget>::max_iterations_;
    using IterativeClosestPoint<PointSource, PointTarget>::previous_transformation_;
    using IterativeClosestPoint<PointSource, PointTarget>::final_transformation_;
    using IterativeClosestPoint<PointSource, PointTarget>::transformation_;
    using IterativeClosestPoint<PointSource, PointTarget>::transformation_epsilon_;
    using IterativeClosestPoint<PointSource, PointTarget>::converged_;
    using IterativeClosestPoint<PointSource, PointTarget>::corr_dist_threshold_;
    using IterativeClosestPoint<PointSource, PointTarget>::inlier_threshold_;
    using IterativeClosestPoint<PointSource, PointTarget>::min_number_correspondences_;

    using IterativeClosestPoint<PointSource, PointTarget>::rigid_transformation_estimation_;

    typedef pcl::PointCloud<PointSource> PointCloudSource;
    typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
    typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

    typedef pcl::PointCloud<PointTarget> PointCloudTarget;
    typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
    typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

    typedef PointIndices::Ptr PointIndicesPtr;
    typedef PointIndices::ConstPtr PointIndicesConstPtr;

    typedef typename pcl::KdTree<PointSource> InputKdTree;
    typedef typename pcl::KdTree<PointSource>::Ptr InputKdTreePtr;

    public:
      /** \brief Empty constructor. */
      GeneralizedIterativeClosestPoint () : k_correspondences_(20), 
      gicp_epsilon_(0.0004), rotation_epsilon_(2e-3),
      input_covariances_(0), target_covariances_(0)
      {
        min_number_correspondences_ = 4;
        reg_name_ = "GeneralizedIterativeClosestPoint";
        max_iterations_ = 200;
        transformation_epsilon_ = 5e-4;
        rigid_transformation_estimation_ = 
          boost::bind (&GeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationLM, 
              this, _1, _2, _3, _4, _5);
      }

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      setInputCloud (const PointCloudTargetConstPtr &cloud)
      {
        if (cloud->points.empty ())
        {
          PCL_ERROR ("[pcl::%s::setInputInput] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
          return;
        }
        input_ = cloud;
        input_tree_->setInputCloud (input_);
        input_covariances_.reserve (cloud->size ());
      }

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
       * \param cloud the input point cloud target
       */
      inline void 
      setInputTarget (const PointCloudTargetConstPtr &target)
      {
        pcl::Registration<PointSource, PointTarget>::setInputTarget(target);
        target_covariances_.reserve (target->size ());
      }

      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
        * non-linear Levenberg-Marquardt approach.
        * \param cloud_src the source point cloud dataset
        * \param cloud_tgt the target point cloud dataset
        * \param transformation_matrix the resultant transformation matrix
        */
      void 
      estimateRigidTransformationLM (const PointCloudSource &cloud_src, 
                                     const PointCloudTarget &cloud_tgt, 
                                     Eigen::Matrix4f &transformation_matrix);

      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
        * non-linear Levenberg-Marquardt approach.
        * \param cloud_src the source point cloud dataset
        * \param indices_src the vector of indices describing the points of interest in \a cloud_src
        * \param cloud_tgt the target point cloud dataset
        * \param indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
        * \param transformation_matrix the resultant transformation matrix
        */
      void 
      estimateRigidTransformationLM (const PointCloudSource &cloud_src, 
                                     const std::vector<int> &indices_src, 
                                     const PointCloudTarget &cloud_tgt, 
                                     const std::vector<int> &indices_tgt, 
                                     Eigen::Matrix4f &transformation_matrix);
      
      /** \brief \return computed mahalanobis distance matrix for the given point index */
      inline const Eigen::Matrix3d& mahalanobis(size_t index) const
      {
        assert(is_mahalanobis_done_ && (index < mahalanobis_.size()));
        return mahalanobis_[index];
      }

      /** \brief computes rotation matrix derivative.
        * rotation matrix is obtainded from quaternion components x[3], x[4] and x[5]
        * \return d/d_rx, d/d_ry and d/d_rz respectively in g[3], g[4] and g[5]
        * param x array representing 3D transformation
        * param R rotation matrix
        * param g gradient vector
        */
      void
      computeRDerivative(const double x[], const Eigen::Matrix3d &R, double g[]);

      /** \brief Set the rotation epsilon (maximum allowable difference between two 
        * consecutive rotations) in order for an optimization to be considered as having 
        * converged to the final solution.
        * \param epsilon the rotation epsilon
        */
      inline void 
      setRotationEpsilon (double epsilon) { rotation_epsilon_ = epsilon; }

      /** \brief Get the rotation epsilon (maximum allowable difference between two 
        * consecutive rotations) as set by the user.
        */
      inline double 
      getRotationEpsilon () { return (rotation_epsilon_); }

      /** \brief Set the number of neighbors used when selecting a point neighbourhood
        * to compute covariances. 
        * A higher value will bring more accurate covariance matrix but will make 
        * covariances computation slower.
        * \param k the number of neighbors to use when computing covariances
        */
      void
      setCorrespondenceRandomness (int k) { k_correspondences_ = k; }

      /** \brief Get the number of neighbors used when computing covariances as set by 
        * the user 
        */
      void
      getCorrespondenceRandomness () { return (k_correspondences_); }

    private:

      /** \brief The number of neighbors used for covariances computation. 
        * \default 20
        */
      int k_correspondences_;
      /** epsilon constant for gicp paper; this is NOT the convergence tolerence 
        * \default 0.0004
        */
      double gicp_epsilon_;
      /** epsilon constant for rotation error. (In GICP the transformation epsilon is 
        * split in rotation part and translation part).
        * \default 2e-3
        */
      double rotation_epsilon_;
      /** \brief Cost function to be minimized
        * \param p a pointer to our data structure array
        * \param m the number of functions
        * \param n the number of variables
        * \param x a pointer to the variables array
        * \param fvec a pointer to the resultant functions evaluations
        * \param iflag set to -1 inside the function to terminate execution
        */
      static int 
      functionToOptimize (void *p, int m, int n, const double *x, double *fvec, double *fjac, int ldfjac, int iflag);

      /** \brief Cost function to be minimized
        * \param p a pointer to our data structure array
        * \param m the number of functions
        * \param n the number of variables
        * \param x a pointer to the variables array
        * \param fvec a pointer to the resultant functions evaluations
        * \param iflag set to -1 inside the function to terminate execution
        */
      static int 
      functionToOptimizeIndices (void *p, int m, int n, const double *x, double *fvec, double *fjac, int ldfjac, int iflag);

      /** \brief The vector of residual weights. Used internall in the LM loop. */
      std::vector<double> weights_;

      /** \brief Temporary boost mutex for \a tmp_src_ and \a tmp_tgt_*/
      boost::mutex tmp_mutex_;

      /** \brief Temporary pointer to the source dataset. */
      const PointCloudSource *tmp_src_;

      /** \brief Temporary pointer to the target dataset. */
      const PointCloudTarget  *tmp_tgt_;

      /** \brief Temporary pointer to the source dataset indices. */
      const std::vector<int> *tmp_idx_src_;

      /** \brief Temporary pointer to the target dataset indices. */
      const std::vector<int> *tmp_idx_tgt_;

      /** \brief KD tree pointer of the input cloud. */
      InputKdTreePtr input_tree_;
      
      /** \brief Input cloud points covariances. */
      std::vector<Eigen::Matrix3d> input_covariances_;

      /** \brief Target cloud points covariances. */
      std::vector<Eigen::Matrix3d> target_covariances_;

      /** \brief Mahalanobis matrices holder. */
      std::vector<Eigen::Matrix3d> mahalanobis_;
      
      /** \brief tells if mahalanobis matrices were computed */
      bool is_mahalanobis_done_;
      
      /** \brief compute points covariances matrices according to the K nearest 
        * neighbors. K is set via setCorrespondenceRandomness() methode.
        * \param cloud pointer to point cloud
        * \param tree KD tree performer for nearest neighbors search
        * \return cloud_covariance covariances matrices for each point in the cloud
        */
      template<typename PointT>
      void computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                              const typename pcl::KdTree<PointT>::Ptr tree,
                              std::vector<Eigen::Matrix3d>& cloud_covariances);

      /** \return trace of mat1^t . mat2 
        * \param mat1 matrix of dimension nxm
        * \param mat2 matrix of dimension pxn
        * \remark mat1 
        */
      inline double matricesInnerProd(const Eigen::MatrixXd& mat1, 
                                      const Eigen::MatrixXd& mat2)
      {
        double r = 0.;
        size_t n = mat1.rows();
        // tr(mat1^t.mat2)
        for(size_t i = 0; i < n; i++)
          for(size_t j = 0; j < n; j++)
            r += mat1 (j, i) * mat2 (i,j);
        return r;
      }

      std::vector<int> nn_indices_;
      std::vector<float> nn_distances_;
      
      /** \brief Rigid transformation computation method.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        */
      void 
      computeTransformation (PointCloudSource &output)
      {
        computeTransformation (output, Eigen::Matrix4f::Identity());
      }

      /** \brief Rigid transformation computation method  with initial guess.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess the initial guess of the transformation to compute
        */
      void 
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess);

      /** \brief Search for the closest nearest neighbor of a given point.
        * \param cloud the point cloud dataset to use for nearest neighbor search
        * \param point_index the index of the query point
        */
      inline bool
      searchForNeighbors (const PointSource &query, int point_index)
      {
        std::vector<int> index (1, -1);
        std::vector<float> distance (1, std::numeric_limits<float>::max());
        int k = tree_->nearestKSearch (query, 1, index, distance);
        nn_indices_[point_index] = index[0];
        nn_distances_[point_index] = distance[0];

        if (k == 0)
          return (false);
        return (true);
      }

  };
}

#include "pcl/registration/impl/gicp.hpp"

#endif  //#ifndef PCL_GICP_H_
