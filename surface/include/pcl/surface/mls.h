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

#pragma once

#include <functional>
#include <map>
#include <random>
#include <Eigen/Core> // for Vector3i, Vector3d, ...

// PCL includes
#include <pcl/memory.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
#include <pcl/search/search.h> // for Search

#include <pcl/surface/processing.h>

namespace pcl
{

  /** \brief Data structure used to store the results of the MLS fitting */
  struct MLSResult
  {
    enum ProjectionMethod
    {
      NONE,      /**< \brief Project to the mls plane. */
      SIMPLE,    /**< \brief Project along the mls plane normal to the polynomial surface. */
      ORTHOGONAL /**< \brief Project to the closest point on the polynonomial surface. */
    };

    /** \brief Data structure used to store the MLS polynomial partial derivatives */
    struct PolynomialPartialDerivative
    {
      double z;    /**< \brief The z component of the polynomial evaluated at z(u, v). */
      double z_u;  /**< \brief The partial derivative dz/du. */
      double z_v;  /**< \brief The partial derivative dz/dv. */
      double z_uu; /**< \brief The partial derivative d^2z/du^2. */
      double z_vv; /**< \brief The partial derivative d^2z/dv^2. */
      double z_uv; /**< \brief The partial derivative d^2z/dudv. */
    };

    /** \brief Data structure used to store the MLS projection results */
    struct MLSProjectionResults
    {
      MLSProjectionResults () : u (0), v (0) {}

      double u;               /**< \brief The u-coordinate of the projected point in local MLS frame. */
      double v;               /**< \brief The v-coordinate of the projected point in local MLS frame. */
      Eigen::Vector3d point;  /**< \brief The projected point. */
      Eigen::Vector3d normal; /**< \brief The projected point's normal. */
      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline
    MLSResult () : num_neighbors (0), curvature (0.0f), order (0), valid (false) {}

    inline
    MLSResult (const Eigen::Vector3d &a_query_point,
               const Eigen::Vector3d &a_mean,
               const Eigen::Vector3d &a_plane_normal,
               const Eigen::Vector3d &a_u,
               const Eigen::Vector3d &a_v,
               const Eigen::VectorXd &a_c_vec,
               const int a_num_neighbors,
               const float a_curvature,
               const int a_order);

    /** \brief Given a point calculate its 3D location in the MLS frame.
      * \param[in] pt The point
      * \param[out] u The u-coordinate of the point in local MLS frame.
      * \param[out] v The v-coordinate of the point in local MLS frame.
      * \param[out] w The w-coordinate of the point in local MLS frame.
      */
    inline void
    getMLSCoordinates (const Eigen::Vector3d &pt, double &u, double &v, double &w) const;

    /** \brief Given a point calculate its 2D location in the MLS frame.
      * \param[in] pt The point
      * \param[out] u The u-coordinate of the point in local MLS frame.
      * \param[out] v The v-coordinate of the point in local MLS frame.
      */
    inline void
    getMLSCoordinates (const Eigen::Vector3d &pt, double &u, double &v) const;

    /** \brief Calculate the polynomial
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \return The polynomial value at the provided uv coordinates.
      */
    inline double
    getPolynomialValue (const double u, const double v) const;

    /** \brief Calculate the polynomial's first and second partial derivatives.
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \return The polynomial partial derivatives at the provided uv coordinates.
      */
    inline PolynomialPartialDerivative
    getPolynomialPartialDerivative (const double u, const double v) const;

    /** \brief Calculate the principal curvatures using the polynomial surface.
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \return The principal curvature [k1, k2] at the provided uv coordinates.
      * \note If an error occurs then 1e-5 is returned.
      */
    Eigen::Vector2f
    calculatePrincipalCurvatures (const double u, const double v) const;

    /** \brief Calculate the principal curvatures using the polynomial surface.
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \return The principal curvature [k1, k2] at the provided uv coordinates.
      * \note If an error occurs then 1e-5 is returned.
      */
    PCL_DEPRECATED(1, 15, "use calculatePrincipalCurvatures() instead")
    inline Eigen::Vector2f
    calculatePrincipleCurvatures (const double u, const double v) const { return calculatePrincipalCurvatures(u, v); };

    /** \brief Project a point orthogonal to the polynomial surface.
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \param[in] w The w-coordinate of the point in local MLS frame.
      * \return The MLSProjectionResults for the input data.
      * \note If the MLSResults does not contain polynomial data it projects the point onto the mls plane.
      * \note If the optimization diverges it performs a simple projection on to the polynomial surface.
      * \note This was implemented based on this https://math.stackexchange.com/questions/1497093/shortest-distance-between-point-and-surface
      */
    inline MLSProjectionResults
    projectPointOrthogonalToPolynomialSurface (const double u, const double v, const double w) const;

    /** \brief Project a point onto the MLS plane.
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \return The MLSProjectionResults for the input data.
      */
    inline MLSProjectionResults
    projectPointToMLSPlane (const double u, const double v) const;

    /** \brief Project a point along the MLS plane normal to the polynomial surface.
      * \param[in] u The u-coordinate of the point in local MLS frame.
      * \param[in] v The v-coordinate of the point in local MLS frame.
      * \return The MLSProjectionResults for the input data.
      * \note If the MLSResults does not contain polynomial data it projects the point onto the mls plane.
      */
    inline MLSProjectionResults
    projectPointSimpleToPolynomialSurface (const double u, const double v) const;

    /**
     * \brief Project a point using the specified method.
     * \param[in] pt The point to be project.
     * \param[in] method The projection method to be used.
     * \param[in] required_neighbors The minimum number of neighbors required.
     * \note If required_neighbors is 0 then any number of neighbors is allowed.
     * \note If required_neighbors is not satisfied it projects to the mls plane.
     * \return The MLSProjectionResults for the input data.
     */
    inline MLSProjectionResults
    projectPoint (const Eigen::Vector3d &pt, ProjectionMethod method, int required_neighbors = 0) const;

    /**
     * \brief Project the query point used to generate the mls surface about using the specified method.
     * \param[in] method The projection method to be used.
     * \param[in] required_neighbors The minimum number of neighbors required.
     * \note If required_neighbors is 0 then any number of neighbors is allowed.
     * \note If required_neighbors is not satisfied it projects to the mls plane.
     * \return The MLSProjectionResults for the input data.
     */
    inline MLSProjectionResults
    projectQueryPoint (ProjectionMethod method, int required_neighbors = 0) const;

    /** \brief Smooth a given point and its neighborhood using Moving Least Squares.
      * \param[in] cloud the input cloud, used together with index and nn_indices
      * \param[in] index the index of the query point in the input cloud
      * \param[in] nn_indices the set of nearest neighbors indices for pt
      * \param[in] search_radius the search radius used to find nearest neighbors for pt
      * \param[in] polynomial_order the order of the polynomial to fit to the nearest neighbors
      * \param[in] weight_func defines the weight function for the polynomial fit
      */
    template <typename PointT> void
    computeMLSSurface (const pcl::PointCloud<PointT> &cloud,
                       pcl::index_t index,
                       const pcl::Indices &nn_indices,
                       double search_radius,
                       int polynomial_order = 2,
                       std::function<double(const double)> weight_func = {});

    Eigen::Vector3d query_point;  /**< \brief The query point about which the mls surface was generated */
    Eigen::Vector3d mean;         /**< \brief The mean point of all the neighbors. */
    Eigen::Vector3d plane_normal; /**< \brief The normal of the local plane of the query point. */
    Eigen::Vector3d u_axis;       /**< \brief The axis corresponding to the u-coordinates of the local plane of the query point. */
    Eigen::Vector3d v_axis;       /**< \brief The axis corresponding to the v-coordinates of the local plane of the query point. */
    Eigen::VectorXd c_vec;        /**< \brief The polynomial coefficients Example: z = c_vec[0] + c_vec[1]*v + c_vec[2]*v^2 + c_vec[3]*u + c_vec[4]*u*v + c_vec[5]*u^2 */
    int num_neighbors;            /**< \brief The number of neighbors used to create the mls surface. */
    float curvature;              /**< \brief The curvature at the query point. */
    int order;                    /**< \brief The order of the polynomial. If order > 1 then use polynomial fit */
    bool valid;                   /**< \brief If True, the mls results data is valid, otherwise False. */
    PCL_MAKE_ALIGNED_OPERATOR_NEW
    private:
      /**
        * \brief The default weight function used when fitting a polynomial surface
        * \param sq_dist the squared distance from a point to origin of the mls frame
        * \param sq_mls_radius the squraed mls search radius used
        * \return The weight for a point at squared distance from the origin of the mls frame
        */
      inline
      double computeMLSWeight (const double sq_dist, const double sq_mls_radius) { return (std::exp (-sq_dist / sq_mls_radius)); }

  };

  /** \brief MovingLeastSquares represent an implementation of the MLS (Moving Least Squares) algorithm
    * for data smoothing and improved normal estimation. It also contains methods for upsampling the
    * resulting cloud based on the parametric fit.
    * Reference paper: "Computing and Rendering Point Set Surfaces" by Marc Alexa, Johannes Behr,
    * Daniel Cohen-Or, Shachar Fleishman, David Levin and Claudio T. Silva
    * www.sci.utah.edu/~shachar/Publications/crpss.pdf
    * \note There is a parallelized version of the processing step, using the OpenMP standard.
    * Compared to the standard version, an overhead is incurred in terms of runtime and memory usage.
    * The upsampling methods DISTINCT_CLOUD and VOXEL_GRID_DILATION are not parallelized completely,
    * i.e. parts of the algorithm run on a single thread only.
    * \author Zoltan Csaba Marton, Radu B. Rusu, Alexandru E. Ichim, Suat Gedikli, Robert Huitl
    * \ingroup surface
    */
  template <typename PointInT, typename PointOutT>
  class MovingLeastSquares : public CloudSurfaceProcessing<PointInT, PointOutT>
  {
    public:
      typedef shared_ptr<MovingLeastSquares<PointInT, PointOutT> > Ptr;
      typedef shared_ptr<const MovingLeastSquares<PointInT, PointOutT> > ConstPtr;

      using PCLBase<PointInT>::input_;
      using PCLBase<PointInT>::indices_;
      using PCLBase<PointInT>::fake_indices_;
      using PCLBase<PointInT>::initCompute;
      using PCLBase<PointInT>::deinitCompute;

      using KdTree = pcl::search::Search<PointInT>;
      using KdTreePtr = typename KdTree::Ptr;
      using NormalCloud = pcl::PointCloud<pcl::Normal>;
      using NormalCloudPtr = NormalCloud::Ptr;

      using PointCloudOut = pcl::PointCloud<PointOutT>;
      using PointCloudOutPtr = typename PointCloudOut::Ptr;
      using PointCloudOutConstPtr = typename PointCloudOut::ConstPtr;

      using PointCloudIn = pcl::PointCloud<PointInT>;
      using PointCloudInPtr = typename PointCloudIn::Ptr;
      using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;

      using SearchMethod = std::function<int (pcl::index_t, double, pcl::Indices &, std::vector<float> &)>;

      enum UpsamplingMethod
      {
        NONE,                   /**< \brief No upsampling will be done, only the input points will be projected
                                            to their own MLS surfaces. */
        DISTINCT_CLOUD,         /**< \brief Project the points of the distinct cloud to the MLS surface. */
        SAMPLE_LOCAL_PLANE,     /**< \brief The local plane of each input point will be sampled in a circular fashion
                                            using the \ref upsampling_radius_ and the \ref upsampling_step_ parameters. */
        RANDOM_UNIFORM_DENSITY, /**< \brief The local plane of each input point will be sampled using an uniform random
                                            distribution such that the density of points is constant throughout the
                                            cloud - given by the \ref desired_num_points_in_radius_ parameter. */
        VOXEL_GRID_DILATION     /**< \brief The input cloud will be inserted into a voxel grid with voxels of
                                            size \ref voxel_size_; this voxel grid will be dilated \ref dilation_iteration_num_
                                            times and the resulting points will be projected to the MLS surface
                                            of the closest point in the input cloud; the result is a point cloud
                                            with filled holes and a constant point density. */
      };

      /** \brief Empty constructor. */
      MovingLeastSquares () : CloudSurfaceProcessing<PointInT, PointOutT> (),
                              distinct_cloud_ (),
                              tree_ (),
                              order_ (2),
                              search_radius_ (0.0),
                              sqr_gauss_param_ (0.0),
                              compute_normals_ (false),
                              upsample_method_ (NONE),
                              upsampling_radius_ (0.0),
                              upsampling_step_ (0.0),
                              desired_num_points_in_radius_ (0),
                              cache_mls_results_ (true),
                              projection_method_ (MLSResult::SIMPLE),
                              threads_ (1),
                              voxel_size_ (1.0),
                              dilation_iteration_num_ (0),
                              nr_coeff_ (),
                              rng_uniform_distribution_ ()
                              {};

      /** \brief Empty destructor */
      ~MovingLeastSquares () {}


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
        search_method_ = [this] (pcl::index_t index, double radius, pcl::Indices& k_indices, std::vector<float>& k_sqr_distances)
        {
          return tree_->radiusSearch (index, radius, k_indices, k_sqr_distances, 0);
        };
      }

      /** \brief Get a pointer to the search method used. */
      inline KdTreePtr
      getSearchMethod () const { return (tree_); }

      /** \brief Set the order of the polynomial to be fit.
        * \param[in] order the order of the polynomial
        * \note Setting order > 1 indicates using a polynomial fit.
        */
      inline void
      setPolynomialOrder (int order) { order_ = order; }

      /** \brief Get the order of the polynomial to be fit. */
      inline int
      getPolynomialOrder () const { return (order_); }

      /** \brief Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting.
        * \param[in] radius the sphere radius that is to contain all k-nearest neighbors
        * \note Calling this method resets the squared Gaussian parameter to radius * radius !
        */
      inline void
      setSearchRadius (double radius) { search_radius_ = radius; sqr_gauss_param_ = search_radius_ * search_radius_; }

      /** \brief Get the sphere radius used for determining the k-nearest neighbors. */
      inline double
      getSearchRadius () const { return (search_radius_); }

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
        */
      inline void
      setUpsamplingMethod (UpsamplingMethod method) { upsample_method_ = method; }

      /** \brief Set the distinct cloud used for the DISTINCT_CLOUD upsampling method. */
      inline void
      setDistinctCloud (PointCloudInConstPtr distinct_cloud) { distinct_cloud_ = distinct_cloud; }

      /** \brief Get the distinct cloud used for the DISTINCT_CLOUD upsampling method. */
      inline PointCloudInConstPtr
      getDistinctCloud () const { return (distinct_cloud_); }


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
      getUpsamplingRadius () const { return (upsampling_radius_); }

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
      getUpsamplingStepSize () const { return (upsampling_step_); }

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
      getPointDensity () const { return (desired_num_points_in_radius_); }

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
      getDilationVoxelSize () const { return (voxel_size_); }

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
      getDilationIterations () const { return (dilation_iteration_num_); }

      /** \brief Set whether the mls results should be stored for each point in the input cloud
        * \param[in] cache_mls_results True if the mls results should be stored, otherwise false.
        * \note The cache_mls_results_ is forced to be true when using upsampling method VOXEL_GRID_DILATION or DISTINCT_CLOUD.
        * \note If memory consumption is a concern, then set it to false when not using upsampling method VOXEL_GRID_DILATION or DISTINCT_CLOUD.
        */
      inline void
      setCacheMLSResults (bool cache_mls_results) { cache_mls_results_ = cache_mls_results; }

      /** \brief Get the cache_mls_results_ value (True if the mls results should be stored, otherwise false). */
      inline bool
      getCacheMLSResults () const { return (cache_mls_results_); }

      /** \brief Set the method to be used when projection the point on to the MLS surface.
        * \param method
        * \note This is only used when polynomial fit is enabled.
        */
      inline void
      setProjectionMethod (MLSResult::ProjectionMethod method) { projection_method_ = method; }


      /** \brief Get the current projection method being used. */
      inline MLSResult::ProjectionMethod
      getProjectionMethod () const { return (projection_method_); }

      /** \brief Get the MLSResults for input cloud
        * \note The results are only stored if setCacheMLSResults(true) was called or when using the upsampling method DISTINCT_CLOUD or VOXEL_GRID_DILATION.
        * \note This vector is aligned with the input cloud indices, so use getCorrespondingIndices to get the correct results when using output cloud indices.
        */
      inline const std::vector<MLSResult>&
      getMLSResults () const { return (mls_results_); }

      /** \brief Set the maximum number of threads to use
      * \param threads the maximum number of hardware threads to use (0 sets the value to 1)
      */
      inline void
      setNumberOfThreads (unsigned int threads = 1)
      {
        threads_ = threads;
      }

      /** \brief Base method for surface reconstruction for all points given in <setInputCloud (), setIndices ()>
        * \param[out] output the resultant reconstructed surface model
        */
      void
      process (PointCloudOut &output) override;


      /** \brief Get the set of indices with each point in output having the
        * corresponding point in input */
      inline PointIndicesPtr
      getCorrespondingIndices () const { return (corresponding_input_indices_); }

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

      /** \brief True if the mls results for the input cloud should be stored
        * \note This is forced to be true when using upsampling methods VOXEL_GRID_DILATION or DISTINCT_CLOUD.
        */
      bool cache_mls_results_;

      /** \brief Stores the MLS result for each point in the input cloud
        * \note Used only in the case of VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling
        */
      std::vector<MLSResult> mls_results_;

      /** \brief Parameter that specifies the projection method to be used. */
      MLSResult::ProjectionMethod projection_method_;

      /** \brief The maximum number of threads the scheduler should use. */
      unsigned int threads_;


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
          getIndexIn1D (const Eigen::Vector3i &index, std::uint64_t &index_1d) const
          {
            index_1d = index[0] * data_size_ * data_size_ +
                       index[1] * data_size_ + index[2];
          }

          inline void
          getIndexIn3D (std::uint64_t index_1d, Eigen::Vector3i& index_3d) const
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
              index[i] = static_cast<Eigen::Vector3i::Scalar> ((p[i] - bounding_min_ (i)) / voxel_size_);
          }

          inline void
          getPosition (const std::uint64_t &index_1d, Eigen::Vector3f &point) const
          {
            Eigen::Vector3i index_3d;
            getIndexIn3D (index_1d, index_3d);
            for (int i = 0; i < 3; ++i)
              point[i] = static_cast<Eigen::Vector3f::Scalar> (index_3d[i]) * voxel_size_ + bounding_min_[i];
          }

          typedef std::map<std::uint64_t, Leaf> HashMap;
          HashMap voxel_grid_;
          Eigen::Vector4f bounding_min_, bounding_max_;
          std::uint64_t data_size_;
          float voxel_size_;
          PCL_MAKE_ALIGNED_OPERATOR_NEW
      };


      /** \brief Voxel size for the VOXEL_GRID_DILATION upsampling method */
      float voxel_size_;

      /** \brief Number of dilation steps for the VOXEL_GRID_DILATION upsampling method */
      int dilation_iteration_num_;

      /** \brief Number of coefficients, to be computed from the requested order.*/
      int nr_coeff_;

      /** \brief Collects for each point in output the corrseponding point in the input. */
      PointIndicesPtr corresponding_input_indices_;

      /** \brief Search for the nearest neighbors of a given point using a radius search
        * \param[in] index the index of the query point
        * \param[out] indices the resultant vector of indices representing the neighbors within search_radius_
        * \param[out] sqr_distances the resultant squared distances from the query point to the neighbors within search_radius_
        */
      inline int
      searchForNeighbors (pcl::index_t index, pcl::Indices &indices, std::vector<float> &sqr_distances) const
      {
        return (search_method_ (index, search_radius_, indices, sqr_distances));
      }

      /** \brief Smooth a given point and its neighborghood using Moving Least Squares.
        * \param[in] index the index of the query point in the input cloud
        * \param[in] nn_indices the set of nearest neighbors indices for pt
        * \param[out] projected_points the set of projected points around the query point
        * (in the case of upsampling method NONE, only the query point projected to its own fitted surface will be returned,
        * in the case of the other upsampling methods, multiple points will be returned)
        * \param[out] projected_points_normals the normals corresponding to the projected points
        * \param[out] corresponding_input_indices the set of indices with each point in output having the corresponding point in input
        * \param[out] mls_result stores the MLS result for each point in the input cloud
        * (used only in the case of VOXEL_GRID_DILATION or DISTINCT_CLOUD upsampling)
        */
      void
      computeMLSPointNormal (pcl::index_t index,
                             const pcl::Indices &nn_indices,
                             PointCloudOut &projected_points,
                             NormalCloud &projected_points_normals,
                             PointIndices &corresponding_input_indices,
                             MLSResult &mls_result) const;


      /** \brief This is a helper function for adding projected points
        * \param[in] index the index of the query point in the input cloud
        * \param[in] point the projected point to be added
        * \param[in] normal the projected point's normal to be added
        * \param[in] curvature the projected point's curvature
        * \param[out] projected_points the set of projected points around the query point
        * \param[out] projected_points_normals the normals corresponding to the projected points
        * \param[out] corresponding_input_indices the set of indices with each point in output having the corresponding point in input
        */
      void
      addProjectedPointNormal (pcl::index_t index,
                               const Eigen::Vector3d &point,
                               const Eigen::Vector3d &normal,
                               double curvature,
                               PointCloudOut &projected_points,
                               NormalCloud &projected_points_normals,
                               PointIndices &corresponding_input_indices) const;


      void
      copyMissingFields (const PointInT &point_in,
                         PointOutT &point_out) const;

      /** \brief Abstract surface reconstruction method.
        * \param[out] output the result of the reconstruction
        */
      void
      performProcessing (PointCloudOut &output) override;

      /** \brief Perform upsampling for the distinct-cloud and voxel-grid methods
        * \param[out] output the result of the reconstruction
       */
      void
      performUpsampling (PointCloudOut &output);

    private:
      /** \brief Random number generator algorithm. */
      mutable std::mt19937 rng_;

      /** \brief Random number generator using an uniform distribution of floats
        * \note Used only in the case of RANDOM_UNIFORM_DENSITY upsampling
        */
      std::unique_ptr<std::uniform_real_distribution<>> rng_uniform_distribution_;

      /** \brief Abstract class get name method. */
      std::string
      getClassName () const { return ("MovingLeastSquares"); }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/surface/impl/mls.hpp>
#endif
