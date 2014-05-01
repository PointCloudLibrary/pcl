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
#include <pcl/search/pcl_search.h>
#include <pcl/common/common.h>

#include <pcl/surface/boost.h>
#include <pcl/surface/eigen.h>
#include <pcl/surface/processing.h>
#include <map>

namespace pcl
{
  /** \brief MovingLeastSquares represent an implementation of the MLS (Moving Least Squares) algorithm 
    * for data smoothing and improved normal estimation. It also contains methods for upsampling the 
    * resulting cloud based on the parametric fit.
    * Reference paper: "Computing and Rendering Point Set Surfaces" by Marc Alexa, Johannes Behr, 
    * Daniel Cohen-Or, Shachar Fleishman, David Levin and Claudio T. Silva
    * www.sci.utah.edu/~shachar/Publications/crpss.pdf
    * \author Zoltan Csaba Marton, Radu B. Rusu, Alexandru E. Ichim, Suat Gedikli
    * \ingroup surface
    */
  template <typename PointInT, typename PointOutT>
  class MovingLeastSquares: public CloudSurfaceProcessing<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<MovingLeastSquares<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const MovingLeastSquares<PointInT, PointOutT> > ConstPtr;

      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::fake_indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

      typedef typename pcl::search::Search<PointInT> KdTree;
      typedef typename pcl::search::Search<PointInT>::Ptr KdTreePtr;
      typedef pcl::PointCloud<pcl::Normal> NormalCloud;
      typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;
      typedef typename PointCloudOut::Ptr PointCloudOutPtr;
      typedef typename PointCloudOut::ConstPtr PointCloudOutConstPtr;

      typedef pcl::PointCloud<PointInT> PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef boost::function<int (int, double, std::vector<int> &, std::vector<float> &)> SearchMethod;

      enum UpsamplingMethod {NONE, DISTINCT_CLOUD, SAMPLE_LOCAL_PLANE, RANDOM_UNIFORM_DENSITY, VOXEL_GRID_DILATION};

      /** \brief Empty constructor. */
      MovingLeastSquares () : CloudSurfaceProcessing<PointInT, PointOutT> (),
                              normals_ (),
                              distinct_cloud_ (),
                              search_method_ (),
                              tree_ (),
                              order_ (2),
                              polynomial_fit_ (true),
                              search_radius_ (0.0),
                              sqr_gauss_param_ (0.0),
                              compute_normals_ (false),
                              upsample_method_ (NONE),
                              upsampling_radius_ (0.0),
                              upsampling_step_ (0.0),
                              desired_num_points_in_radius_ (0),
                              mls_results_ (),
                              voxel_size_ (1.0),
                              dilation_iteration_num_ (0),
                              nr_coeff_ (),
                              corresponding_input_indices_ (),
                              rng_alg_ (),
                              rng_uniform_distribution_ ()
                              {};
      
      /** \brief Empty destructor */
      virtual ~MovingLeastSquares () {}


      /** \brief Set whether the algorithm should also store the normals computed
        * \note This is optional, but need a proper output cloud type
        */
      inline void
      setComputeNormals (bool compute_normals) { compute_normals_ = compute_normals; }

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

      /** \brief Set the upsampling method to be used
        * \param method
        * \note Options are: * NONE - no upsampling will be done, only the input points will be projected to their own
        *                             MLS surfaces
        *                    * DISTINCT_CLOUD - will project the points of the distinct cloud to the closest point on
        *                                       the MLS surface
        *                    * SAMPLE_LOCAL_PLANE - the local plane of each input point will be sampled in a circular
        *                                           fashion using the \ref upsampling_radius_ and the \ref upsampling_step_
        *                                           parameters
        *                    * RANDOM_UNIFORM_DENSITY - the local plane of each input point will be sampled using an
        *                                               uniform random distribution such that the density of points is
        *                                               constant throughout the cloud - given by the \ref desired_num_points_in_radius_
        *                                               parameter
        *                    * VOXEL_GRID_DILATION - the input cloud will be inserted into a voxel grid with voxels of
        *                                            size \ref voxel_size_; this voxel grid will be dilated \ref dilation_iteration_num_
        *                                            times and the resulting points will be projected to the MLS surface
        *                                            of the closest point in the input cloud; the result is a point cloud
        *                                            with filled holes and a constant point density
        */
      inline void
      setUpsamplingMethod (UpsamplingMethod method) { upsample_method_ = method; }

      /** \brief Set the distinct cloud used for the DISTINCT_CLOUD upsampling method. */
      inline void
      setDistinctCloud (PointCloudInConstPtr distinct_cloud) { distinct_cloud_ = distinct_cloud; }

      /** \brief Get the distinct cloud used for the DISTINCT_CLOUD upsampling method. */
      inline PointCloudInConstPtr
      getDistinctCloud () { return distinct_cloud_; }


      /** \brief Set the radius of the circle in the local point plane that will be sampled
        * \note Used only in the case of SAMPLE_LOCAL_PLANE upsampling
        * \param[in] radius the radius of the circle
        */
      inline void
      setUpsamplingRadius (double radius) { upsampling_radius_ = radius; }

      /** \brief Get the radius of the circle in the local point plane that will be sampled
        * \note Used only in the case of SAMPLE_LOCAL_PLANE upsampling
        */
      inline double
      getUpsamplingRadius () { return upsampling_radius_; }

      /** \brief Set the step size for the local plane sampling
        * \note Used only in the case of SAMPLE_LOCAL_PLANE upsampling
        * \param[in] step_size the step size
        */
      inline void
      setUpsamplingStepSize (double step_size) { upsampling_step_ = step_size; }


      /** \brief Get the step size for the local plane sampling
        * \note Used only in the case of SAMPLE_LOCAL_PLANE upsampling
        */
      inline double
      getUpsamplingStepSize () { return upsampling_step_; }

      /** \brief Set the parameter that specifies the desired number of points within the search radius
        * \note Used only in the case of RANDOM_UNIFORM_DENSITY upsampling
        * \param[in] desired_num_points_in_radius the desired number of points in the output cloud in a sphere of
        * radius \ref search_radius_ around each point
        */
      inline void
      setPointDensity (int desired_num_points_in_radius) { desired_num_points_in_radius_ = desired_num_points_in_radius; }


      /** \brief Get the parameter that specifies the desired number of points within the search radius
        * \note Used only in the case of RANDOM_UNIFORM_DENSITY upsampling
        */
      inline int
      getPointDensity () { return desired_num_points_in_radius_; }

      /** \brief Set the voxel size for the voxel grid
        * \note Used only in the VOXEL_GRID_DILATION upsampling method
        * \param[in] voxel_size the edge length of a cubic voxel in the voxel grid
        */
      inline void
      setDilationVoxelSize (float voxel_size) { voxel_size_ = voxel_size; }


      /** \brief Get the voxel size for the voxel grid
        * \note Used only in the VOXEL_GRID_DILATION upsampling method
        */
      inline float
      getDilationVoxelSize () { return voxel_size_; }

      /** \brief Set the number of dilation steps of the voxel grid
        * \note Used only in the VOXEL_GRID_DILATION upsampling method
        * \param[in] iterations the number of dilation iterations
        */
      inline void
      setDilationIterations (int iterations) { dilation_iteration_num_ = iterations; }

      /** \brief Get the number of dilation steps of the voxel grid
        * \note Used only in the VOXEL_GRID_DILATION upsampling method
        */
      inline int
      getDilationIterations () { return dilation_iteration_num_; }

      /** \brief Base method for surface reconstruction for all points given in <setInputCloud (), setIndices ()>
        * \param[out] output the resultant reconstructed surface model
        */
      void 
      process (PointCloudOut &output);


      /** \brief Get the set of indices with each point in output having the 
        * corresponding point in input */
      inline PointIndicesPtr
      getCorrespondingIndices () { return (corresponding_input_indices_); }

    protected:
      /** \brief The point cloud that will hold the estimated normals, if set. */
      NormalCloudPtr normals_;

      /** \brief The distinct point cloud that will be projected to the MLS surface. */
      PointCloudInConstPtr distinct_cloud_;

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

      /** \brief Parameter that specifies whether the normals should be computed for the input cloud or not */
      bool compute_normals_;

      /** \brief Parameter that specifies the upsampling method to be used */
      UpsamplingMethod upsample_method_;

      /** \brief Radius of the circle in the local point plane that will be sampled
        * \note Used only in the case of SAMPLE_LOCAL_PLANE upsampling
        */
      double upsampling_radius_;

      /** \brief Step size for the local plane sampling
        * \note Used only in the case of SAMPLE_LOCAL_PLANE upsampling
        */
      double upsampling_step_;

      /** \brief Parameter that specifies the desired number of points within the search radius
        * \note Used only in the case of RANDOM_UNIFORM_DENSITY upsampling
        */
      int desired_num_points_in_radius_;

      
      /** \brief Data structure used to store the results of the MLS fitting
        * \note Used only in the case of VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling
        */
      struct MLSResult
      {
        MLSResult () : mean (), plane_normal (), u_axis (), v_axis (), c_vec (), num_neighbors (), curvature (), valid (false) {}

        MLSResult (const Eigen::Vector3d &a_mean,
                   const Eigen::Vector3d &a_plane_normal,
                   const Eigen::Vector3d &a_u,
                   const Eigen::Vector3d &a_v,
                   const Eigen::VectorXd a_c_vec,
                   const int a_num_neighbors,
                   const float &a_curvature);

        Eigen::Vector3d mean, plane_normal, u_axis, v_axis;
        Eigen::VectorXd c_vec;
        int num_neighbors;
        float curvature;
        bool valid;
      };

      /** \brief Stores the MLS result for each point in the input cloud
        * \note Used only in the case of VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling
        */
      std::vector<MLSResult> mls_results_;

      
      /** \brief A minimalistic implementation of a voxel grid, necessary for the point cloud upsampling
        * \note Used only in the case of VOXEL_GRID_DILATION upsampling
        */
      class MLSVoxelGrid
      {
        public:
          struct Leaf { Leaf () : valid (true) {} bool valid; };

          MLSVoxelGrid (PointCloudInConstPtr& cloud,
                        IndicesPtr &indices,
                        float voxel_size);

          void
          dilate ();

          inline void
          getIndexIn1D (const Eigen::Vector3i &index, uint64_t &index_1d) const
          {
            index_1d = index[0] * data_size_ * data_size_ +
                       index[1] * data_size_ + index[2];
          }

          inline void
          getIndexIn3D (uint64_t index_1d, Eigen::Vector3i& index_3d) const
          {
            index_3d[0] = static_cast<Eigen::Vector3i::Scalar> (index_1d / (data_size_ * data_size_));
            index_1d -= index_3d[0] * data_size_ * data_size_;
            index_3d[1] = static_cast<Eigen::Vector3i::Scalar> (index_1d / data_size_);
            index_1d -= index_3d[1] * data_size_;
            index_3d[2] = static_cast<Eigen::Vector3i::Scalar> (index_1d);
          }

          inline void
          getCellIndex (const Eigen::Vector3f &p, Eigen::Vector3i& index) const
          {
            for (int i = 0; i < 3; ++i)
              index[i] = static_cast<Eigen::Vector3i::Scalar> ((p[i] - bounding_min_(i)) / voxel_size_);
          }

          inline void
          getPosition (const uint64_t &index_1d, Eigen::Vector3f &point) const
          {
            Eigen::Vector3i index_3d;
            getIndexIn3D (index_1d, index_3d);
            for (int i = 0; i < 3; ++i)
              point[i] = static_cast<Eigen::Vector3f::Scalar> (index_3d[i]) * voxel_size_ + bounding_min_[i];
          }

          typedef std::map<uint64_t, Leaf> HashMap;
          HashMap voxel_grid_;
          Eigen::Vector4f bounding_min_, bounding_max_;
          uint64_t data_size_;
          float voxel_size_;
      };


      /** \brief Voxel size for the VOXEL_GRID_DILATION upsampling method */
      float voxel_size_;

      /** \brief Number of dilation steps for the VOXEL_GRID_DILATION upsampling method */
      int dilation_iteration_num_; 

      /** \brief Number of coefficients, to be computed from the requested order.*/
      int nr_coeff_;

      /** \brief Collects for each point in output the corrseponding point in the input. */
      PointIndicesPtr corresponding_input_indices_;

      /** \brief Search for the closest nearest neighbors of a given point using a radius search
        * \param[in] index the index of the query point
        * \param[out] indices the resultant vector of indices representing the k-nearest neighbors
        * \param[out] sqr_distances the resultant squared distances from the query point to the k-nearest neighbors
        */
      inline int
      searchForNeighbors (int index, std::vector<int> &indices, std::vector<float> &sqr_distances) const
      {
        return (search_method_ (index, search_radius_, indices, sqr_distances));
      }

      /** \brief Smooth a given point and its neighborghood using Moving Least Squares.
        * \param[in] index the inex of the query point in the input cloud
        * \param[in] nn_indices the set of nearest neighbors indices for pt
        * \param[in] nn_sqr_dists the set of nearest neighbors squared distances for pt
        * \param[out] projected_points the set of points projected points around the query point
        * (in the case of upsampling method NONE, only the query point projected to its own fitted surface will be returned,
        * in the case of the other upsampling methods, multiple points will be returned)
        * \param[out] projected_points_normals the normals corresponding to the projected points
        * \param[out] corresponding_input_indices the set of indices with each point in output having the corresponding point in input
        * \param[out] mls_result stores the MLS result for each point in the input cloud
        * (used only in the case of VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling)
        */
      void
      computeMLSPointNormal (int index,
                             const std::vector<int> &nn_indices,
                             std::vector<float> &nn_sqr_dists,
                             PointCloudOut &projected_points,
                             NormalCloud &projected_points_normals,
                             PointIndices &corresponding_input_indices,
                             MLSResult &mls_result) const;

      /** \brief Fits a point (sample point) given in the local plane coordinates of an input point (query point) to
        * the MLS surface of the input point
        * \param[in] u_disp the u coordinate of the sample point in the local plane of the query point
        * \param[in] v_disp the v coordinate of the sample point in the local plane of the query point
        * \param[in] u_axis the axis corresponding to the u-coordinates of the local plane of the query point
        * \param[in] v_axis the axis corresponding to the v-coordinates of the local plane of the query point
        * \param[in] n_axis
        * \param mean
        * \param[in] curvature the curvature of the surface at the query point
        * \param[in] c_vec the coefficients of the polynomial fit on the MLS surface of the query point
        * \param[in] num_neighbors the number of neighbors of the query point in the input cloud
        * \param[out] result_point the absolute 3D position of the resulting projected point
        * \param[out] result_normal the normal of the resulting projected point
        */
      void
      projectPointToMLSSurface (float &u_disp, float &v_disp,
                                Eigen::Vector3d &u_axis, Eigen::Vector3d &v_axis,
                                Eigen::Vector3d &n_axis,
                                Eigen::Vector3d &mean,
                                float &curvature,
                                Eigen::VectorXd &c_vec,
                                int num_neighbors,
                                PointOutT &result_point,
                                pcl::Normal &result_normal) const;

      void
      copyMissingFields (const PointInT &point_in,
                         PointOutT &point_out) const;

      /** \brief Abstract surface reconstruction method. 
        * \param[out] output the result of the reconstruction 
        */
      virtual void performProcessing (PointCloudOut &output);

      /** \brief Perform upsampling for the distinct-cloud and voxel-grid methods
        * \param[out] output the result of the reconstruction
       */
      void performUpsampling (PointCloudOut &output);

    private:
      /** \brief Boost-based random number generator algorithm. */
      boost::mt19937 rng_alg_;

      /** \brief Random number generator using an uniform distribution of floats
        * \note Used only in the case of RANDOM_UNIFORM_DENSITY upsampling
        */
      boost::shared_ptr<boost::variate_generator<boost::mt19937&, 
                                                 boost::uniform_real<float> > 
                       > rng_uniform_distribution_;

      /** \brief Abstract class get name method. */
      std::string getClassName () const { return ("MovingLeastSquares"); }
      
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

#ifdef _OPENMP
  /** \brief MovingLeastSquaresOMP is a parallelized version of MovingLeastSquares, using the OpenMP standard.
   * \note Compared to MovingLeastSquares, an overhead is incurred in terms of runtime and memory usage.
   * \note The upsampling methods DISTINCT_CLOUD and VOXEL_GRID_DILATION are not parallelized completely, i.e. parts of the algorithm run on a single thread only.
   * \author Robert Huitl
   * \ingroup surface
   */
  template <typename PointInT, typename PointOutT>
  class MovingLeastSquaresOMP: public MovingLeastSquares<PointInT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<MovingLeastSquares<PointInT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const MovingLeastSquares<PointInT, PointOutT> > ConstPtr;

      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using MovingLeastSquares<PointInT, PointOutT>::normals_;
      using MovingLeastSquares<PointInT, PointOutT>::corresponding_input_indices_;
      using MovingLeastSquares<PointInT, PointOutT>::nr_coeff_;
      using MovingLeastSquares<PointInT, PointOutT>::order_;
      using MovingLeastSquares<PointInT, PointOutT>::compute_normals_;

      typedef pcl::PointCloud<pcl::Normal> NormalCloud;
      typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;

      typedef pcl::PointCloud<PointOutT> PointCloudOut;
      typedef typename PointCloudOut::Ptr PointCloudOutPtr;
      typedef typename PointCloudOut::ConstPtr PointCloudOutConstPtr;

      /** \brief Constructor for parallelized Moving Least Squares
        * \param threads the maximum number of hardware threads to use (0 sets the value to 1)
        */
      MovingLeastSquaresOMP (unsigned int threads = 0) : threads_ (threads)
      {

      }

      /** \brief Set the maximum number of threads to use
        * \param threads the maximum number of hardware threads to use (0 sets the value to 1)
        */
      inline void
      setNumberOfThreads (unsigned int threads = 0)
      {
        threads_ = threads;
      }

    protected:
      /** \brief Abstract surface reconstruction method.
        * \param[out] output the result of the reconstruction
        */
      virtual void performProcessing (PointCloudOut &output);

      /** \brief The maximum number of threads the scheduler should use. */
      unsigned int threads_;
  };
#endif
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/mls.hpp>
#endif

#endif  /* #ifndef PCL_MLS_H_ */
