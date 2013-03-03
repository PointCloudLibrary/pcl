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

#ifndef PCL_FEATURE_H_
#define PCL_FEATURE_H_

#if defined __GNUC__
#  pragma GCC system_header 
#endif

#include <boost/function.hpp>
#include <boost/bind.hpp>
// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>

namespace pcl
{
  /** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
    * plane normal and surface curvature.
    * \param covariance_matrix the 3x3 covariance matrix
    * \param point a point lying on the least-squares plane (SSE aligned)
    * \param plane_parameters the resultant plane parameters as: a, b, c, d (ax + by + cz + d = 0)
    * \param curvature the estimated surface curvature as a measure of
    * \f[
    * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
    * \f]
    * \ingroup features
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
    * \ingroup features
    */
  inline void
  solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                        float &nx, float &ny, float &nz, float &curvature);

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Feature represents the base feature class. Some generic 3D operations that
    * are applicable to all features are defined here as static methods.
    *
    * \attention
    * The convention for a feature descriptor is:
    *   - if the nearest neighbors for the query point at which the descriptor is to be computed cannot be
    *     determined, the descriptor values will be set to NaN (not a number)
    *   - it is impossible to estimate a feature descriptor for a point that doesn't have finite 3D coordinates.
    *     Therefore, any point that has NaN data on x, y, or z, will most likely have its descriptor set to NaN.
    *
    * \author Radu B. Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointOutT>
  class Feature : public PCLBase<PointInT>
  {
    public:
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::input_;

      typedef PCLBase<PointInT> BaseClass;

      typedef boost::shared_ptr< Feature<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr< const Feature<PointInT, PointOutT> > ConstPtr;

      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;

      typedef boost::function<int (size_t, double, std::vector<int> &, std::vector<float> &)> SearchMethod;
      typedef boost::function<int (const PointCloudIn &cloud, size_t index, double, std::vector<int> &, std::vector<float> &)> SearchMethodSurface;

    public:
      /** \brief Empty constructor. */
      Feature () :
        feature_name_ (), search_method_surface_ (),
        surface_(), tree_(),
        search_parameter_(0), search_radius_(0), k_(0),
        fake_surface_(false)
      {}
            
      /** \brief Empty destructor */
      virtual ~Feature () {}

      /** \brief Provide a pointer to a dataset to add additional information
        * to estimate the features for every point in the input dataset.  This
        * is optional, if this is not set, it will only use the data in the
        * input cloud to estimate the features.  This is useful when you only
        * need to compute the features for a downsampled cloud.
        * \param[in] cloud a pointer to a PointCloud message
        */
      inline void
      setSearchSurface (const PointCloudInConstPtr &cloud)
      {
        surface_ = cloud;
        fake_surface_ = false;
        //use_surface_  = true;
      }

      /** \brief Get a pointer to the surface point cloud dataset. */
      inline PointCloudInConstPtr
      getSearchSurface () const
      {
        return (surface_);
      }

      /** \brief Provide a pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr
      getSearchMethod () const
      {
        return (tree_);
      }

      /** \brief Get the internal search parameter. */
      inline double
      getSearchParameter () const
      {
        return (search_parameter_);
      }

      /** \brief Set the number of k nearest neighbors to use for the feature estimation.
        * \param[in] k the number of k-nearest neighbors
        */
      inline void
      setKSearch (int k) { k_ = k; }

      /** \brief get the number of k nearest neighbors used for the feature estimation. */
      inline int
      getKSearch () const
      {
        return (k_);
      }

      /** \brief Set the sphere radius that is to be used for determining the nearest neighbors used for the feature
        * estimation.
        * \param[in] radius the sphere radius used as the maximum distance to consider a point a neighbor
        */
      inline void
      setRadiusSearch (double radius)
      {
        search_radius_ = radius;
      }

      /** \brief Get the sphere radius used for determining the neighbors. */
      inline double
      getRadiusSearch () const
      {
        return (search_radius_);
      }

      /** \brief Base method for feature estimation for all points given in
        * <setInputCloud (), setIndices ()> using the surface in setSearchSurface ()
        * and the spatial locator in setSearchMethod ()
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      compute (PointCloudOut &output);

    protected:
      /** \brief The feature name. */
      std::string feature_name_;

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
      inline const std::string&
      getClassName () const { return (feature_name_); }

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

      /** \brief This method should get called after ending the actual computation. */
      virtual bool
      deinitCompute ();

      /** \brief If no surface is given, we use the input PointCloud as the surface. */
      bool fake_surface_;

      /** \brief Search for k-nearest neighbors using the spatial locator from
        * \a setSearchmethod, and the given surface from \a setSearchSurface.
        * \param[in] index the index of the query point
        * \param[in] parameter the search parameter (either k or radius)
        * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
        * \param[out] distances the resultant vector of distances representing the distances from the query point to the
        * k-nearest neighbors
        *
        * \return the number of neighbors found. If no neighbors are found or an error occurred, return 0.
        */
      inline int
      searchForNeighbors (size_t index, double parameter,
                          std::vector<int> &indices, std::vector<float> &distances) const
      {
        return (search_method_surface_ (*input_, index, parameter, indices, distances));
      }

      /** \brief Search for k-nearest neighbors using the spatial locator from
        * \a setSearchmethod, and the given surface from \a setSearchSurface.
        * \param[in] cloud the query point cloud
        * \param[in] index the index of the query point in \a cloud
        * \param[in] parameter the search parameter (either k or radius)
        * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
        * \param[out] distances the resultant vector of distances representing the distances from the query point to the
        * k-nearest neighbors
        *
        * \return the number of neighbors found. If no neighbors are found or an error occurred, return 0.
        */
      inline int
      searchForNeighbors (const PointCloudIn &cloud, size_t index, double parameter,
                          std::vector<int> &indices, std::vector<float> &distances) const
      {
        return (search_method_surface_ (cloud, index, parameter, indices, distances));
      }

    private:
      /** \brief Abstract feature estimation method.
        * \param[out] output the resultant features
        */
      virtual void
      computeFeature (PointCloudOut &output) = 0;

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
    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      typedef typename pcl::PointCloud<PointNT> PointCloudN;
      typedef typename PointCloudN::Ptr PointCloudNPtr;
      typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

      typedef boost::shared_ptr< FeatureFromNormals<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr< const FeatureFromNormals<PointInT, PointNT, PointOutT> > ConstPtr;

      // Members derived from the base class
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::getClassName;

      /** \brief Empty constructor. */
      FeatureFromNormals () : normals_ () {}
      
      /** \brief Empty destructor */
      virtual ~FeatureFromNormals () {}

      /** \brief Provide a pointer to the input dataset that contains the point normals of
        * the XYZ dataset.
        * In case of search surface is set to be different from the input cloud,
        * normals should correspond to the search surface, not the input cloud!
        * \param[in] normals the const boost shared pointer to a PointCloud of normals.
        * By convention, L2 norm of each normal should be 1.
        */
      inline void
      setInputNormals (const PointCloudNConstPtr &normals) { normals_ = normals; }

      /** \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
      inline PointCloudNConstPtr
      getInputNormals () const { return (normals_); }

    protected:
      /** \brief A pointer to the input dataset that contains the point normals of the XYZ
        * dataset.
        */
      PointCloudNConstPtr normals_;

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  template <typename PointInT, typename PointLT, typename PointOutT>
  class FeatureFromLabels : public Feature<PointInT, PointOutT>
  {
    typedef typename Feature<PointInT, PointOutT>::PointCloudIn PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef typename pcl::PointCloud<PointLT> PointCloudL;
    typedef typename PointCloudL::Ptr PointCloudNPtr;
    typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

    typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      typedef boost::shared_ptr< FeatureFromLabels<PointInT, PointLT, PointOutT> > Ptr;
      typedef boost::shared_ptr< const FeatureFromLabels<PointInT, PointLT, PointOutT> > ConstPtr;

      // Members derived from the base class
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::k_;

      /** \brief Empty constructor. */
      FeatureFromLabels () : labels_ ()
      {
        k_ = 1; // Search tree is not always used here.
      }
      
      /** \brief Empty destructor */
      virtual ~FeatureFromLabels () {}

      /** \brief Provide a pointer to the input dataset that contains the point labels of
        * the XYZ dataset.
        * In case of search surface is set to be different from the input cloud,
        * labels should correspond to the search surface, not the input cloud!
        * \param[in] labels the const boost shared pointer to a PointCloud of labels.
        */
      inline void
      setInputLabels (const PointCloudLConstPtr &labels)
      {
        labels_ = labels;
      }

      /** \brief Get a pointer to the labels of the input XYZ point cloud dataset. */
      inline PointCloudLConstPtr
      getInputLabels () const
      {
        return (labels_);
      }

    protected:
      /** \brief A pointer to the input dataset that contains the point labels of the XYZ
        * dataset.
        */
      PointCloudLConstPtr labels_;

      /** \brief This method should get called before starting the actual computation. */
      virtual bool
      initCompute ();

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief FeatureWithLocalReferenceFrames provides a public interface for descriptor
    * extractor classes which need a local reference frame at each input keypoint.
    *
    * \attention
    * This interface is for backward compatibility with existing code and in the future it could be
    * merged with pcl::Feature. Subclasses should call the protected method initLocalReferenceFrames ()
    * to correctly initialize the frames_ member.
    *
    * \author Nicola Fioraio
    * \ingroup features
    */
  template <typename PointInT, typename PointRFT>
  class FeatureWithLocalReferenceFrames
  {
    public:
      typedef pcl::PointCloud<PointRFT> PointCloudLRF;
      typedef typename PointCloudLRF::Ptr PointCloudLRFPtr;
      typedef typename PointCloudLRF::ConstPtr PointCloudLRFConstPtr;

      /** \brief Empty constructor. */
      FeatureWithLocalReferenceFrames () : frames_ (), frames_never_defined_ (true) {}

       /** \brief Empty destructor. */
      virtual ~FeatureWithLocalReferenceFrames () {}

      /** \brief Provide a pointer to the input dataset that contains the local
        * reference frames of the XYZ dataset.
        * In case of search surface is set to be different from the input cloud,
        * local reference frames should correspond to the input cloud, not the search surface!
        * \param[in] frames the const boost shared pointer to a PointCloud of reference frames.
        */
      inline void
      setInputReferenceFrames (const PointCloudLRFConstPtr &frames)
      {
        frames_ = frames;
        frames_never_defined_ = false;
      }

      /** \brief Get a pointer to the local reference frames. */
      inline PointCloudLRFConstPtr
      getInputReferenceFrames () const
      {
        return (frames_);
      }

    protected:
      /** \brief A boost shared pointer to the local reference frames. */
      PointCloudLRFConstPtr frames_;
      /** \brief The user has never set the frames. */
      bool frames_never_defined_;

      /** \brief Check if frames_ has been correctly initialized and compute it if needed.
        * \param input the subclass' input cloud dataset.
        * \param lrf_estimation a pointer to a local reference frame estimation class to be used as default.
        * \return true if frames_ has been correctly initialized.
        */
      typedef typename Feature<PointInT, PointRFT>::Ptr LRFEstimationPtr;
      virtual bool
      initLocalReferenceFrames (const size_t& indices_size,
                                const LRFEstimationPtr& lrf_estimation = LRFEstimationPtr());
  };
}

#include <pcl/features/impl/feature.hpp>

#endif  //#ifndef PCL_FEATURE_H_
