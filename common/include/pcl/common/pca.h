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
 */

#ifndef PCL_PCA_H
#define PCL_PCA_H

#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

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
    * \author Nizar Sallem
    * \ingroup common
    */
  template <typename PointT>
  class PCA : public pcl::PCLBase <PointT>
  {
    public:
      typedef pcl::PCLBase <PointT> Base;
      typedef typename Base::PointCloud PointCloud;
      typedef typename Base::PointCloudPtr PointCloudPtr;
      typedef typename Base::PointCloudConstPtr PointCloudConstPtr;
      typedef typename Base::PointIndicesPtr PointIndicesPtr;
      typedef typename Base::PointIndicesConstPtr PointIndicesConstPtr;

      using Base::input_;
      using Base::indices_;
      using Base::initCompute;
      using Base::setInputCloud;

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
      PCA (bool basis_only = false)
        : Base ()
        , compute_done_ (false)
        , basis_only_ (basis_only) 
        , eigenvectors_ ()
        , coefficients_ ()
        , mean_ ()
        , eigenvalues_  ()
      {}
      
      /** \brief Constructor with direct computation
        * X input m*n matrix (ie n vectors of R(m))
        * basis_only flag to compute only the PCA basis
        */
      PCL_DEPRECATED ("Use PCA (bool basis_only); setInputCloud (X.makeShared ()); instead")
      PCA (const pcl::PointCloud<PointT>& X, bool basis_only = false);

      /** Copy Constructor
        * \param[in] pca PCA object
        */
      PCA (PCA const & pca) 
        : Base (pca)
        , compute_done_ (pca.compute_done_)
        , basis_only_ (pca.basis_only_) 
        , eigenvectors_ (pca.eigenvectors_)
        , coefficients_ (pca.coefficients_)
        , mean_ (pca.mean_)
        , eigenvalues_  (pca.eigenvalues_)
      {}

      /** Assignment operator
        * \param[in] pca PCA object
        */
      inline PCA& 
      operator= (PCA const & pca) 
      {
        eigenvectors_ = pca.eigenvectors;
        coefficients_ = pca.coefficients;
        eigenvalues_  = pca.eigenvalues;
        mean_         = pca.mean;
        return (*this);
      }
      
      /** \brief Provide a pointer to the input dataset
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      inline void 
      setInputCloud (const PointCloudConstPtr &cloud) 
      { 
        Base::setInputCloud (cloud);
        compute_done_ = false;
      }

      /** \brief Mean accessor
        * \throw InitFailedException
        */
      inline Eigen::Vector4f& 
      getMean () 
      {
        if (!compute_done_)
          initCompute ();
        if (!compute_done_)
          PCL_THROW_EXCEPTION (InitFailedException, 
                               "[pcl::PCA::getMean] PCA initCompute failed");
        return (mean_);
      }

      /** Eigen Vectors accessor
        * \throw InitFailedException
        */
      inline Eigen::Matrix3f& 
      getEigenVectors () 
      {
        if (!compute_done_)
          initCompute ();
        if (!compute_done_)
          PCL_THROW_EXCEPTION (InitFailedException, 
                               "[pcl::PCA::getEigenVectors] PCA initCompute failed");
        return (eigenvectors_);
      }
      
      /** Eigen Values accessor
        * \throw InitFailedException
        */
      inline Eigen::Vector3f& 
      getEigenValues ()
      {
        if (!compute_done_)
          initCompute ();
        if (!compute_done_)
          PCL_THROW_EXCEPTION (InitFailedException, 
                               "[pcl::PCA::getEigenVectors] PCA getEigenValues failed");
        return (eigenvalues_);
      }
      
      /** Coefficients accessor
        * \throw InitFailedException
        */
      inline Eigen::MatrixXf& 
      getCoefficients () 
      {
        if (!compute_done_)
          initCompute ();
        if (!compute_done_)
          PCL_THROW_EXCEPTION (InitFailedException, 
                               "[pcl::PCA::getEigenVectors] PCA getCoefficients failed");
        return (coefficients_);
      }
            
      /** update PCA with a new point
        * \param[in] input input point 
        * \param[in] flag update flag
        * \throw InitFailedException
        */
      inline void 
      update (const PointT& input, FLAG flag = preserve);
      
      /** Project point on the eigenspace.
        * \param[in] input point from original dataset
        * \param[out] projection the point in eigen vectors space
        * \throw InitFailedException
        */
      inline void 
      project (const PointT& input, PointT& projection);

      /** Project cloud on the eigenspace.
        * \param[in] input cloud from original dataset
        * \param[out] projection the cloud in eigen vectors space
        * \throw InitFailedException
        */
      inline void
      project (const PointCloud& input, PointCloud& projection);
      
      /** Reconstruct point from its projection
        * \param[in] projection point from eigenvector space
        * \param[out] input reconstructed point
        * \throw InitFailedException
        */
      inline void 
      reconstruct (const PointT& projection, PointT& input);

      /** Reconstruct cloud from its projection
        * \param[in] projection cloud from eigenvector space
        * \param[out] input reconstructed cloud
        * \throw InitFailedException
        */
      inline void
      reconstruct (const PointCloud& projection, PointCloud& input);

    private:
      inline bool
      initCompute ();

      bool compute_done_;
      bool basis_only_;
      Eigen::Matrix3f eigenvectors_;
      Eigen::MatrixXf coefficients_;
      Eigen::Vector4f mean_;
      Eigen::Vector3f eigenvalues_;
  }; // class PCA
} // namespace pcl

#include <pcl/common/impl/pca.hpp>

#endif // PCL_PCA_H

