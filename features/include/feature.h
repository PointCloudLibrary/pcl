/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: feature.h 35810 2011-02-08 00:03:46Z rusu $
 *
 */

#ifndef PCL_FEATURE_H_
#define PCL_FEATURE_H_

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/mpl/size.hpp>

// PCL includes
#include "pcl/pcl_base.h"
#include "pcl/common/eigen.h"

#include "pcl/kdtree/tree_types.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/organized_data.h"

#include "pcl/io/io.h"

namespace pcl
{
  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points and return it as a 3D vector.
    * \param cloud the input point cloud
    * \param centroid the output centroid
    */
  template <typename PointT> inline void 
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::Vector4f &centroid);

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and return it as a 3D vector.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    */
  template <typename PointT> inline void 
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const std::vector<int> &indices, Eigen::Vector4f &centroid);

  /** \brief Compute the 3D (X-Y-Z) centroid of a set of points using their indices and return it as a 3D vector.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    */
  template <typename PointT> inline void 
  compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const pcl::PointIndices &indices, Eigen::Vector4f &centroid);

  /** \brief Helper functor structure for n-D centroid estimation. */
  template<typename PointT>
  struct NdCentroidFunctor
  {
    typedef typename traits::POD<PointT>::type Pod;
    
    NdCentroidFunctor (const PointT &p, Eigen::VectorXf &centroid)
      : f_idx_ (0),
        centroid_ (centroid),
        p_ (reinterpret_cast<const Pod&>(p)) { }

    template<typename Key> inline void operator() ()
    {
      typedef typename pcl::traits::datatype<PointT, Key>::type T;
      const uint8_t* raw_ptr = reinterpret_cast<const uint8_t*>(&p_) + pcl::traits::offset<PointT, Key>::value;
      const T* data_ptr = reinterpret_cast<const T*>(raw_ptr);

      // Check if the value is invalid
      if (!pcl_isfinite (*data_ptr))
      {
        f_idx_++;
        return;
      }

      centroid_[f_idx_++] += *data_ptr;
    }

    private:
      int f_idx_;
      Eigen::VectorXf &centroid_;
      const Pod &p_;
  };

  /** \brief General, all purpose nD centroid estimation for a set of points using their indices.
    * \param cloud the input point cloud
    * \param centroid the output centroid
    */
  template <typename PointT> inline void 
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, Eigen::VectorXf &centroid);

  /** \brief General, all purpose nD centroid estimation for a set of points using their indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    */
  template <typename PointT> inline void 
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const std::vector<int> &indices, Eigen::VectorXf &centroid);

  /** \brief General, all purpose nD centroid estimation for a set of points using their indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the output centroid
    */
  template <typename PointT> inline void 
  computeNDCentroid (const pcl::PointCloud<PointT> &cloud, 
                     const pcl::PointIndices &indices, Eigen::VectorXf &centroid);

  /** \brief Compute the 3x3 covariance matrix of a given set of points.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeNormalizedCovarianceMatrix.
    * \param cloud the input point cloud
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
  template <typename PointT> inline void 
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                           const Eigen::Vector4f &centroid, 
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute normalized the 3x3 covariance matrix of a given set of points.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of points in the point cloud.
    * \param cloud the input point cloud
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
  template <typename PointT> inline void 
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                     const Eigen::Vector4f &centroid, 
                                     Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeNormalizedCovarianceMatrix.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
  template <typename PointT> inline void 
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                           const std::vector<int> &indices, 
                           const Eigen::Vector4f &centroid, 
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Note: the covariance matrix is not normalized with the number of
    * points. For a normalized covariance, please use
    * computeNormalizedCovarianceMatrix.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
  template <typename PointT> inline void 
  computeCovarianceMatrix (const pcl::PointCloud<PointT> &cloud, 
                           const pcl::PointIndices &indices, 
                           const Eigen::Vector4f &centroid, 
                           Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
  template <typename PointT> inline void 
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                     const std::vector<int> &indices, 
                                     const Eigen::Vector4f &centroid, 
                                     Eigen::Matrix3f &covariance_matrix);

  /** \brief Compute the normalized 3x3 covariance matrix of a given set of points using their indices.
    * The result is returned as a Eigen::Matrix3f.
    * Normalized means that every entry has been divided by the number of entries in indices.
    * \param cloud the input point cloud
    * \param indices the point cloud indices that need to be used
    * \param centroid the centroid of the set of points in the cloud
    * \param covariance_matrix the resultant 3x3 covariance matrix
    */
  template <typename PointT> inline void 
  computeCovarianceMatrixNormalized (const pcl::PointCloud<PointT> &cloud, 
                                     const pcl::PointIndices &indices, 
                                     const Eigen::Vector4f &centroid, 
                                     Eigen::Matrix3f &covariance_matrix);

  /** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
    * plane normal and surface curvature.
    * \param covariance_matrix the 3x3 covariance matrix
    * \param point a point lying on the least-squares plane (SSE aligned)
    * \param plane_parameters the resultant plane parameters as: a, b, c, d (ax + by + cz + d = 0)
    * \param curvature the estimated surface curvature as a measure of
    * \f[
    * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
    * \f]
    */
  inline void 
  solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix, 
                        const Eigen::Vector4f &point, 
                        Eigen::Vector4f &plane_parameters, float &curvature);

  /** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
    * plane normal and surface curvature.
    * \param covariance_matrix the 3x3 covariance matrix
    * \param nx the resultant X component of the plane normal
    * \param ny the resultant Y component of the plane normal
    * \param nz the resultant Z component of the plane normal
    * \param curvature the estimated surface curvature as a measure of
    * \f[
    * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
    * \f]
    */
  inline void 
  solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix, 
                        float &nx, float &ny, float &nz, float &curvature);

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b Feature represents the base feature class. Some generic 3D operations that 
    * are applicable to all features are defined here as static methods.
    * \author Radu Bogdan Rusu
    */
  template <typename PointInT, typename PointOutT>
  class Feature : public PCLBase<PointInT>
  {
    using PCLBase<PointInT>::initCompute;
    using PCLBase<PointInT>::deinitCompute;

    public:
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::input_;

      typedef PCLBase<PointInT> BaseClass;
      
      typedef typename pcl::KdTree<PointInT> KdTree;
      typedef typename pcl::KdTree<PointInT>::Ptr KdTreePtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      typedef boost::function<int (int, double, std::vector<int> &, std::vector<float> &)> SearchMethod;
      typedef boost::function<int (const PointCloudIn &cloud, int index, double, std::vector<int> &, std::vector<float> &)> SearchMethodSurface;
    
    public:
      /** \brief Empty constructor. */
      Feature () : surface_(), tree_(), search_parameter_(0), search_radius_(0), k_(0), fake_surface_(false)
      {};

      /** \brief Provide a pointer to the input dataset that we need to estimate features at every point for.
        * \param cloud the const boost shared pointer to a PointCloud message
        */
      inline void
      setSearchSurface (const PointCloudInConstPtr &cloud)
      {
        surface_ = cloud;
        fake_surface_ = false;
        //use_surface_  = true;
      }

      /** \brief Get a pointer to the surface point cloud dataset. */
      inline PointCloudInConstPtr getSearchSurface () { return (surface_); }

      /** \brief Provide a pointer to the search object.
        * \param tree a pointer to the spatial search object.
        */
      inline void setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr getSearchMethod () { return (tree_); }

      /** \brief Get the internal search parameter. */
      inline double getSearchParameter () { return (search_parameter_); }

      /** \brief Set the number of k nearest neighbors to use for the feature estimation.
        * \param k the number of k-nearest neighbors
        */
      inline void setKSearch (int k) { k_ = k; }

      /** \brief get the number of k nearest neighbors used for the feature estimation. */
      inline int getKSearch () { return (k_); }

      /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
        * estimation.
        * \param radius the sphere radius used as the maximum distance to consider a point a neighbor
        */
      inline void setRadiusSearch (double radius) { search_radius_ = radius; }

      /** \brief Get the sphere radius used for determining the neighbors. */
      inline double getRadiusSearch () { return (search_radius_); }

      /** \brief Base method for feature estimation for all points given in 
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface () 
        * and the spatial locator in setSearchMethod ()
        * \param output the resultant point cloud model dataset containing the estimated features
        */
      void compute (PointCloudOut &output);

      /** \brief Search for k-nearest neighbors using the spatial locator from 
        * \a setSearchmethod, and the given surface from \a setSearchSurface.
        * \param index the index of the query point
        * \param parameter the search parameter (either k or radius)
        * \param indices the resultant vector of indices representing the k-nearest neighbors
        * \param distances the resultant vector of distances representing the distances from the query point to the
        * k-nearest neighbors
        */
      inline int
      searchForNeighbors (int index, double parameter, 
                          std::vector<int> &indices, std::vector<float> &distances)
      {
        if (surface_ == input_)       // if the two surfaces are the same
          return (search_method_ (index, parameter, indices, distances));
        else
          return (search_method_surface_ (*input_, index, parameter, indices, distances));
      }

    protected:
      /** \brief The feature name. */
      std::string feature_name_;

      /** \brief The search method template for indices. */
      SearchMethod search_method_;

      /** \brief The search method template for points. */
      SearchMethodSurface search_method_surface_;

      /** \brief An input point cloud describing the surface that is to be used
        * for nearest neighbors estimation. 
        */
      PointCloudInConstPtr surface_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The actual search parameter (from either \a search_radius_ or \a k_). */
      double search_parameter_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief The number of K nearest neighbors to use for each point. */
      int k_;

      /** \brief Get a string representation of the name of this class. */
      inline const std::string& getClassName () const { return (feature_name_); }

    private:
      /** \brief If no surface is given, we use the input PointCloud as the surface. */
      bool fake_surface_;

      /** \brief Abstract feature estimation method. */
      virtual void computeFeature (PointCloudOut &output) = 0;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointInT, typename PointNT, typename PointOutT>
  class FeatureFromNormals : public Feature<PointInT, PointOutT>
  {
    typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef typename pcl::PointCloud<PointNT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      // Members derived from the base class
      using Feature<PointInT, PointOutT>::input_;

      /** \brief Empty constructor. */
      FeatureFromNormals () {};

      /** \brief Provide a pointer to the input dataset that contains the point normals of 
        * the XYZ dataset.
        * \param normals the const boost shared pointer to a PointCloud message
        */
      inline void setInputNormals (const PointCloudNConstPtr &normals) { normals_ = normals; }

      /** \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
      inline PointCloudNConstPtr getInputNormals () { return (normals_); }

    protected:     
      /** \brief A pointer to the input dataset that contains the point normals of the XYZ 
        * dataset. 
        */
      PointCloudNConstPtr normals_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include "pcl/features/feature.hpp"

#endif  //#ifndef PCL_FEATURE_H_
