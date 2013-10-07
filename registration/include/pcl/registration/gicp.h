/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_GICP_H_
#define PCL_GICP_H_

#include <pcl/registration/icp.h>
#include <pcl/registration/bfgs.h>

namespace pcl
{
  /** \brief GeneralizedIterativeClosestPoint is an ICP variant that implements the 
    * generalized iterative closest point algorithm as described by Alex Segal et al. in 
    * http://www.stanford.edu/~avsegal/resources/papers/Generalized_ICP.pdf
    * The approach is based on using anistropic cost functions to optimize the alignment 
    * after closest point assignments have been made.
    * The original code uses GSL and ANN while in ours we use an eigen mapped BFGS and 
    * FLANN.
    * \author Nizar Sallem
    * \ingroup registration
    */
  template <typename PointSource, typename PointTarget>
  class GeneralizedIterativeClosestPoint : public IterativeClosestPoint<PointSource, PointTarget>
  {
    public:
      using IterativeClosestPoint<PointSource, PointTarget>::reg_name_;
      using IterativeClosestPoint<PointSource, PointTarget>::getClassName;
      using IterativeClosestPoint<PointSource, PointTarget>::indices_;
      using IterativeClosestPoint<PointSource, PointTarget>::target_;
      using IterativeClosestPoint<PointSource, PointTarget>::input_;
      using IterativeClosestPoint<PointSource, PointTarget>::tree_;
      using IterativeClosestPoint<PointSource, PointTarget>::tree_reciprocal_;
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
      using IterativeClosestPoint<PointSource, PointTarget>::update_visualizer_;

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      typedef typename Registration<PointSource, PointTarget>::KdTree InputKdTree;
      typedef typename Registration<PointSource, PointTarget>::KdTreePtr InputKdTreePtr;

      typedef boost::shared_ptr< GeneralizedIterativeClosestPoint<PointSource, PointTarget> > Ptr;
      typedef boost::shared_ptr< const GeneralizedIterativeClosestPoint<PointSource, PointTarget> > ConstPtr;


      typedef Eigen::Matrix<double, 6, 1> Vector6d;

      /** \brief Empty constructor. */
      GeneralizedIterativeClosestPoint () 
        : k_correspondences_(20)
        , gicp_epsilon_(0.001)
        , rotation_epsilon_(2e-3)
        , input_covariances_(0)
        , target_covariances_(0)
        , mahalanobis_(0)
        , max_inner_iterations_(20)
      {
        min_number_correspondences_ = 4;
        reg_name_ = "GeneralizedIterativeClosestPoint";
        max_iterations_ = 200;
        transformation_epsilon_ = 5e-4;
        corr_dist_threshold_ = 5.;
        rigid_transformation_estimation_ = 
          boost::bind (&GeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationBFGS, 
                       this, _1, _2, _3, _4, _5); 
      }
      
      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      PCL_DEPRECATED (void setInputCloud (const PointCloudSourceConstPtr &cloud), "[pcl::registration::GeneralizedIterativeClosestPoint::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.");

      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      setInputSource (const PointCloudSourceConstPtr &cloud)
      {

        if (cloud->points.empty ())
        {
          PCL_ERROR ("[pcl::%s::setInputSource] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
          return;
        }
        PointCloudSource input = *cloud;
        // Set all the point.data[3] values to 1 to aid the rigid transformation
        for (size_t i = 0; i < input.size (); ++i)
          input[i].data[3] = 1.0;
        
        pcl::IterativeClosestPoint<PointSource, PointTarget>::setInputSource (cloud);
        input_covariances_.clear ();
        input_covariances_.reserve (input_->size ());
      }

      /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)
        * \param[in] target the input point cloud target
        */
      inline void 
      setInputTarget (const PointCloudTargetConstPtr &target)
      {
        pcl::IterativeClosestPoint<PointSource, PointTarget>::setInputTarget(target);
        target_covariances_.clear ();
        target_covariances_.reserve (target_->size ());
      }

      /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using an iterative
        * non-linear Levenberg-Marquardt approach.
        * \param[in] cloud_src the source point cloud dataset
        * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
        * \param[in] cloud_tgt the target point cloud dataset
        * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
        * \param[out] transformation_matrix the resultant transformation matrix
        */
      void
      estimateRigidTransformationBFGS (const PointCloudSource &cloud_src,
                                       const std::vector<int> &indices_src,
                                       const PointCloudTarget &cloud_tgt,
                                       const std::vector<int> &indices_tgt,
                                       Eigen::Matrix4f &transformation_matrix);
      
      /** \brief \return Mahalanobis distance matrix for the given point index */
      inline const Eigen::Matrix3d& mahalanobis(size_t index) const
      {
        assert(index < mahalanobis_.size());
        return mahalanobis_[index];
      }

      /** \brief Computes rotation matrix derivative.
        * rotation matrix is obtainded from rotation angles x[3], x[4] and x[5]
        * \return d/d_rx, d/d_ry and d/d_rz respectively in g[3], g[4] and g[5]
        * param x array representing 3D transformation
        * param R rotation matrix
        * param g gradient vector
        */
      void
      computeRDerivative(const Vector6d &x, const Eigen::Matrix3d &R, Vector6d &g) const;

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
      int
      getCorrespondenceRandomness () { return (k_correspondences_); }

      /** set maximum number of iterations at the optimization step
        * \param[in] max maximum number of iterations for the optimizer
        */
      void
      setMaximumOptimizerIterations (int max) { max_inner_iterations_ = max; }

      ///\return maximum number of iterations at the optimization step
      int
      getMaximumOptimizerIterations () { return (max_inner_iterations_); }

    protected:

      /** \brief The number of neighbors used for covariances computation. 
        * default: 20
        */
      int k_correspondences_;

      /** \brief The epsilon constant for gicp paper; this is NOT the convergence 
        * tolerence 
        * default: 0.001
        */
      double gicp_epsilon_;

      /** The epsilon constant for rotation error. (In GICP the transformation epsilon 
        * is split in rotation part and translation part).
        * default: 2e-3
        */
      double rotation_epsilon_;

      /** \brief base transformation */
      Eigen::Matrix4f base_transformation_;

      /** \brief Temporary pointer to the source dataset. */
      const PointCloudSource *tmp_src_;

      /** \brief Temporary pointer to the target dataset. */
      const PointCloudTarget  *tmp_tgt_;

      /** \brief Temporary pointer to the source dataset indices. */
      const std::vector<int> *tmp_idx_src_;

      /** \brief Temporary pointer to the target dataset indices. */
      const std::vector<int> *tmp_idx_tgt_;

      
      /** \brief Input cloud points covariances. */
      std::vector<Eigen::Matrix3d> input_covariances_;

      /** \brief Target cloud points covariances. */
      std::vector<Eigen::Matrix3d> target_covariances_;

      /** \brief Mahalanobis matrices holder. */
      std::vector<Eigen::Matrix3d> mahalanobis_;
      
      /** \brief maximum number of optimizations */
      int max_inner_iterations_;

      /** \brief compute points covariances matrices according to the K nearest 
        * neighbors. K is set via setCorrespondenceRandomness() methode.
        * \param cloud pointer to point cloud
        * \param tree KD tree performer for nearest neighbors search
        * \return cloud_covariance covariances matrices for each point in the cloud
        */
      template<typename PointT>
      void computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud, 
                              const typename pcl::search::KdTree<PointT>::Ptr tree,
                              std::vector<Eigen::Matrix3d>& cloud_covariances);

      /** \return trace of mat1^t . mat2 
        * \param mat1 matrix of dimension nxm
        * \param mat2 matrix of dimension nxp
        */
      inline double 
      matricesInnerProd(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2) const
      {
        double r = 0.;
        size_t n = mat1.rows();
        // tr(mat1^t.mat2)
        for(size_t i = 0; i < n; i++)
          for(size_t j = 0; j < n; j++)
            r += mat1 (j, i) * mat2 (i,j);
        return r;
      }

      /** \brief Rigid transformation computation method  with initial guess.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess the initial guess of the transformation to compute
        */
      void 
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess);

      /** \brief Search for the closest nearest neighbor of a given point.
        * \param query the point to search a nearest neighbour for
        * \param index vector of size 1 to store the index of the nearest neighbour found
        * \param distance vector of size 1 to store the distance to nearest neighbour found
        */
      inline bool 
      searchForNeighbors (const PointSource &query, std::vector<int>& index, std::vector<float>& distance)
      {
        int k = tree_->nearestKSearch (query, 1, index, distance);
        if (k == 0)
          return (false);
        return (true);
      }

      /// \brief compute transformation matrix from transformation matrix
      void applyState(Eigen::Matrix4f &t, const Vector6d& x) const;
      
      /// \brief optimization functor structure
      struct OptimizationFunctorWithIndices : public BFGSDummyFunctor<double,6>
      {
        OptimizationFunctorWithIndices (const GeneralizedIterativeClosestPoint* gicp)
          : BFGSDummyFunctor<double,6> (), gicp_(gicp) {}
        double operator() (const Vector6d& x);
        void  df(const Vector6d &x, Vector6d &df);
        void fdf(const Vector6d &x, double &f, Vector6d &df);

        const GeneralizedIterativeClosestPoint *gicp_;
      };
      
      boost::function<void(const pcl::PointCloud<PointSource> &cloud_src,
                           const std::vector<int> &src_indices,
                           const pcl::PointCloud<PointTarget> &cloud_tgt,
                           const std::vector<int> &tgt_indices,
                           Eigen::Matrix4f &transformation_matrix)> rigid_transformation_estimation_;
  };
}

#include <pcl/registration/impl/gicp.hpp>

#endif  //#ifndef PCL_GICP_H_
