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
 */


#ifndef PCL_PCA_HPP
#define PCL_PCA_HPP

#include <pcl/point_cloud.h>

namespace pcl 
{
  /** Principal Component analysis (PCA) class.\n
   *  Principal components are extracted by singular values decomposition on the 
   * covariance matrix of the centered input cloud. Available data after pca computation 
   * are the mean of the input data, the eigenvalues (in descending order) and 
   * corresponding eigenvectors.\n
   * Other methods allow projection in the eigenspace, reconstruction from eigenspace and 
   *  update of the eigenspace with a new datum (according Matej Artec, Matjaz Jogan and 
   * Ales Leonardis: "Incremental PCA for On-line Visual Learning and Recognition").
   *
   *  \ingroup common
   */
  template <typename PointT>
  class PCA 
  {
    public:
    
      /** Updating method flag */
      enum FLAG 
      {
        /** keep the new basis vectors if possible */
        increase, 
        /** preserve subspace dimension */
        preserve
      };
    
    
      /** \brief Default Constructor
        * \param basis_only flag to compute only the PCA basis
        */
      PCA (bool basis_only = false) : compute_done_ (false), basis_only_ (basis_only) 
      {}
      
      /** Constructor with direct computation
        * \param X input m*n matrix (ie n vectors of R(m))
        * \param basis_only flag to compute only the PCA basis
       */
      PCA(const pcl::PointCloud<PointT>& X, bool basis_only = false) : 
        compute_done_ (false), basis_only_ (basis_only)
      {
        compute (X);
      }

      /** Copy Constructor
        * \param pca_ PCA object
        */
      PCA (PCA const & pca_) 
      {
        mean_         = pca_.mean;
        eigenvalues_  = pca_.eigenvalues;
        eigenvectors_ = pca_.eigenvectors;
        coefficients_ = pca_.coefficients;
      }

      /** Assignment operator
        * \param pca PCA object
        */
      inline PCA& operator= (PCA const & pca) 
      {
        mean_         = pca.mean;
        eigenvalues_  = pca.eigenvalues;
        eigenvectors_ = pca.eigenvectors;
        coefficients_ = pca.coefficients;
        return (*this);
      }

      /// Mean accessor
      inline Eigen::Vector4f& 
      getMean () 
      {
        if (!compute_done_)
          PCL_ERROR ("[pcl::PCA::getMean] no results available\n");
        return (mean_);
      }

      /// Eigen Vectors accessor
      inline Eigen::MatrixXf& 
      getEigenVectors () 
      {
        if (!compute_done_)
          PCL_ERROR ("[pcl::PCA::getEigenVectors] no results available\n");
        return (eigenvectors_);
      }
      
      /// Eigen Values accessor
      inline Eigen::VectorXf& 
      getEigenValues ()
      {
        if (!compute_done_)
          PCL_ERROR ("[pcl::PCA::getEigenValues] no results available\n");
        return (eigenvalues_);
      }
      
      /// Coefficients accessor
      inline Eigen::MatrixXf& 
      getCoefficients () 
      {
        if (!compute_done_)
          PCL_ERROR ("[pcl::PCA::getEigenValues] no results available\n");
        return (coefficients_);
      }

      /** Compute PCA using the batch algorithm
        * \param cloud input point cloud
        */
      inline void 
      compute (const pcl::PointCloud<PointT>& cloud);
      
      
      /** update PCA with a new point
        * \param input input point 
        * \param flag update flag
        */
      inline void 
      update (const PointT& input, FLAG flag = preserve);
      
      /** Project point on the eigenspace.
        * \param input point from original dataset
        * \param projection the point in eigen vectors space
        */
      inline void 
      project (const PointT& input, PointT& projection) const;
      
      /** Reconstruct point from its projection
        * \param projection point from eigenvector space
        * \param input reconstructed point
        */
      inline void 
      reconstruct (const PointT& projection, PointT& input) const;
      
    private:
      bool compute_done_;
      bool basis_only_;
      Eigen::MatrixXf eigenvectors_, coefficients_;
      Eigen::Vector4f mean_;
      Eigen::VectorXf eigenvalues_;
  }; // class PCA
} // namespace pcl

#include "pcl/common/impl/pca.hpp"

#endif // PCL_PCA_HPP

