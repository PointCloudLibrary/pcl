/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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

#ifndef PCL_MLS_H_
#define PCL_MLS_H_

// PCL includes
#include <pcl/pcl_base.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/random.hpp>
#include "pcl/search/pcl_search.h"



#include <Eigen/SVD>

namespace pcl
{
  /** \brief MovingLeastSquares represent an implementation of the MLS (Moving Least Squares) algorithm for data
    * smoothing and improved normal estimation.
    * \author Zoltan Csaba Marton, Radu B. Rusu, Alexandru-Eugen Ichim, Suat Gedikli
    * \ingroup surface
    */
  template <typename PointInT, typename NormalOutT>
  class MovingLeastSquares: public PCLBase<PointInT>
  {
    public:
      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::fake_indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;

      typedef pcl::PointCloud<NormalOutT> NormalCloudOut;
      typedef typename NormalCloudOut::Ptr NormalCloudOutPtr;
      typedef typename NormalCloudOut::ConstPtr NormalCloudOutConstPtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef boost::function<int (int, double, std::vector<int> &, std::vector<float> &)> SearchMethod;

      /** \brief Empty constructor. */
      MovingLeastSquares () : PCLBase<PointInT> (), tree_ (), order_ (2), polynomial_fit_ (true), search_radius_ (0), sqr_gauss_param_ (0) {};

      /** \brief Provide a pointer to a point cloud where normal information should be saved
        * \note This is optional, it can be the same as the parameter to the reconstruction method, but no normals are estimated if it is not set.
        * \param[in] cloud the const boost shared pointer to a point cloud with normal
        */
      inline void 
      setOutputNormals (NormalCloudOutPtr cloud) { normals_ = cloud; }

      /** \brief Returns a pointer to the point cloud where normal information was saved during reconstruction */
      inline NormalCloudOutPtr 
      getOutputNormals () { return normals_; }

      /** \brief Provide a pointer to the search object.
        * \param[in] tree a pointer to the spatial search object.
        */
      inline void
      setSearchMethod (const KdTreePtr &tree)
      {
        tree_ = tree;
        // Declare the search locator definition
        int (KdTree::*radiusSearch)(int index, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances, unsigned int max_nn) const = &KdTree::radiusSearch;
        search_method_ = boost::bind (radiusSearch, boost::ref (tree_), _1, _2, _3, _4, 0);
      }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr 
      getSearchMethod () { return (tree_); }

      /** \brief Set the order of the polynomial to be fit.
        * \param[in] order the order of the polynomial
        */
      inline void 
      setPolynomialOrder (int order) { order_ = order; }

      /** \brief Get the order of the polynomial to be fit. */
      inline int 
      getPolynomialOrder () { return (order_); }

      /** \brief Sets whether the surface and normal are approximated using a polynomial, or only via tangent estimation.
        * \param[in] polynomial_fit set to true for polynomial fit
        */
      inline void 
      setPolynomialFit (bool polynomial_fit) { polynomial_fit_ = polynomial_fit; }

      /** \brief Get the polynomial_fit value (true if the surface and normal are approximated using a polynomial). */
      inline bool 
      getPolynomialFit () { return (polynomial_fit_); }

      /** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting.
        * \param[in] radius the sphere radius that is to contain all k-nearest neighbors
        * \note Calling this method resets the squared Gaussian parameter to radius * radius !
        */
      inline void 
      setSearchRadius (double radius) { search_radius_ = radius; sqr_gauss_param_ = search_radius_ * search_radius_; }

      /** \brief Get the sphere radius used for determining the k-nearest neighbors. */
      inline double 
      getSearchRadius () { return (search_radius_); }

      /** \brief Set the parameter used for distance based weighting of neighbors (the square of the search radius works
        * best in general).
        * \param[in] sqr_gauss_param the squared Gaussian parameter
        */
      inline void 
      setSqrGaussParam (double sqr_gauss_param) { sqr_gauss_param_ = sqr_gauss_param; }

      /** \brief Get the parameter for distance based weighting of neighbors. */
      inline double 
      getSqrGaussParam () const { return (sqr_gauss_param_); }

      /** \brief Base method for surface reconstruction for all points given in <setInputCloud (), setIndices ()>
        * \param[out] output the resultant reconstructed surface model
        */
      void 
      reconstruct (PointCloudIn &output);

    protected:
      /** \brief The point cloud that will hold the estimated normals, if set. */
      NormalCloudOutPtr normals_;

      /** \brief The search method template for indices. */
      SearchMethod search_method_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief The order of the polynomial to be fit. */
      int order_;

      /** True if the surface and normal be approximated using a polynomial, false if tangent estimation is sufficient. */
      bool polynomial_fit_;

      /** \brief The nearest neighbors search radius for each point. */
      double search_radius_;

      /** \brief Parameter for distance based weighting of neighbors (search_radius_ * search_radius_ works fine) */
      double sqr_gauss_param_;

      /** \brief Number of coefficients, to be computed from the requested order.*/
      int nr_coeff_;

      /** \brief Search for the closest nearest neighbors of a given point using a radius search
        * \param[in] index the index of the query point
        * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
        * \param[out] sqr_distances the resultant squared distances from the query point to the k-nearest neighbors
        */
      inline int
      searchForNeighbors (int index, std::vector<int> &indices, std::vector<float> &sqr_distances)
      {
        return (search_method_ (index, search_radius_, indices, sqr_distances));
      }

      /** \brief Smooth a given point and its neighborghood using Moving Least Squares.
        * \param pt the point to be smoothed (in place operation, needs to contain data when passed in)
        * \param[in] input the input point cloud that \ref nn_indices refer to
        * \param[in] nn_indices the set of nearest neighbors indices for \ref pt
        * \param nn_sqr_dists the set of nearest neighbors squared distances for \ref pt
        * \param[out] normal the output smoothed normal and curvature as a 4D vector (nx, ny, nz, curvature)
        */
      void
      computeMLSPointNormal (PointInT &pt, const PointCloudIn &input, 
                             const std::vector<int> &nn_indices, std::vector<float> &nn_sqr_dists,
                             Eigen::Vector4f &normal);

    private:
      /** \brief Abstract surface reconstruction method. 
        * \param[out] output the result of the reconstruction 
        */
      virtual void performReconstruction (PointCloudIn &output);

      /** \brief Abstract class get name method. */
      std::string getClassName () const { return ("MovingLeastSquares"); }
  };
}

#endif  //#ifndef PCL_MLS_H_
