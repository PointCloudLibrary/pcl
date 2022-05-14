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
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 */

#pragma once

#include <pcl/filters/filter.h>
#include <ctime>

namespace pcl
{
  /** \brief @b SamplingSurfaceNormal divides the input space into grids until each grid contains a maximum of N points, 
    * and samples points randomly within each grid. Normal is computed using the N points of each grid. All points
    * sampled within a grid are assigned the same normal.
    *
    * \author Aravindhan K Krishnan. This code is ported from libpointmatcher (https://github.com/ethz-asl/libpointmatcher)
    * \ingroup filters
    */
  template<typename PointT>
  class SamplingSurfaceNormal: public Filter<PointT>
  {
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;
    using Filter<PointT>::indices_;
    using Filter<PointT>::input_;

    using PointCloud = typename Filter<PointT>::PointCloud;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using Vector = Eigen::Matrix<float, Eigen::Dynamic, 1>;

    public:

      using Ptr = shared_ptr<SamplingSurfaceNormal<PointT> >;
      using ConstPtr = shared_ptr<const SamplingSurfaceNormal<PointT> >;

      /** \brief Empty constructor. */
      SamplingSurfaceNormal () : 
        sample_ (10), seed_ (static_cast<unsigned int> (time (nullptr))), ratio_ ()
      {
        filter_name_ = "SamplingSurfaceNormal";
        srand (seed_);
      }

      /** \brief Set maximum number of samples in each grid
        * \param[in] sample maximum number of samples in each grid
        */
      inline void
      setSample (unsigned int sample)
      {
        sample_ = sample;
      }

      /** \brief Get the value of the internal \a sample parameter. */
      inline unsigned int
      getSample () const
      {
        return (sample_);
      }

      /** \brief Set seed of random function.
        * \param[in] seed the input seed
        */
      inline void
      setSeed (unsigned int seed)
      {
        seed_ = seed;
        srand (seed_);
      }

      /** \brief Get the value of the internal \a seed parameter. */
      inline unsigned int
      getSeed () const
      {
        return (seed_);
      }

      /** \brief Set ratio of points to be sampled in each grid
        * \param[in] ratio sample the ratio of points to be sampled in each grid
        */
      inline void
      setRatio (float ratio)
      {
        ratio_ = ratio;
      }

      /** \brief Get the value of the internal \a ratio parameter. */
      inline float
      getRatio () const
      {
        return ratio_;
      }

    protected:

      /** \brief Maximum number of samples in each grid. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;
      /** \brief Ratio of points to be sampled in each grid */
      float ratio_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param[out] output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output) override;

    private:

      /** \brief @b CompareDim is a comparator object for sorting across a specific dimension (i,.e X, Y or Z)
       */
      struct CompareDim
      {
        /** \brief The dimension to sort */
        const int dim;
        /** \brief The input point cloud to sort */
        const pcl::PointCloud <PointT>& cloud;

        /** \brief Constructor. */
        CompareDim (const int dim, const pcl::PointCloud <PointT>& cloud) : dim (dim), cloud (cloud)
        {
        }

        /** \brief The operator function for sorting. */
        bool 
        operator () (const int& p0, const int& p1)
        {
          if (dim == 0)
            return (cloud[p0].x < cloud[p1].x);
          if (dim == 1)
            return (cloud[p0].y < cloud[p1].y);
          if (dim == 2)
            return (cloud[p0].z < cloud[p1].z);
          return (false);
        }
      };

      /** \brief Finds the max and min values in each dimension
        * \param[in] cloud the input cloud 
        * \param[out] max_vec the max value vector
        * \param[out] min_vec the min value vector
        */
      void 
      findXYZMaxMin (const PointCloud& cloud, Vector& max_vec, Vector& min_vec);

      /** \brief Recursively partition the point cloud, stopping when each grid contains less than sample_ points
        *  Points are randomly sampled when a grid is found
        * \param cloud
        * \param first
        * \param last
        * \param min_values
        * \param max_values
        * \param indices
        * \param[out] outcloud output the resultant point cloud
        */
      void 
      partition (const PointCloud& cloud, const int first, const int last, 
                 const Vector min_values, const Vector max_values, 
                 Indices& indices, PointCloud& outcloud);

      /** \brief Randomly sample the points in each grid.
        * \param[in] data 
        * \param[in] first
        * \param[in] last
        * \param[out] indices 
        * \param[out] output the resultant point cloud
        */
      void 
      samplePartition (const PointCloud& data, const int first, const int last, 
                       Indices& indices, PointCloud& outcloud);

      /** \brief Returns the threshold for splitting in a given dimension.
        * \param[in] cloud the input cloud
        * \param[in] cut_dim the input dimension (0=x, 1=y, 2=z)
        * \param[in] cut_index the input index in the cloud
        */
      float 
      findCutVal (const PointCloud& cloud, const int cut_dim, const int cut_index);

      /** \brief Computes the normal for points in a grid. This is a port from features to avoid features dependency for
        * filters
        * \param[in] cloud The input cloud
        * \param[out] normal the computed normal
        * \param[out] curvature the computed curvature
        */
      void 
      computeNormal (const PointCloud& cloud, Eigen::Vector4f &normal, float& curvature);

      /** \brief Computes the covariance matrix for points in the cloud. This is a port from features to avoid features dependency for
        * filters
        * \param[in] cloud The input cloud
        * \param[out] covariance_matrix the covariance matrix 
        * \param[out] centroid the centroid
        */
      unsigned int 
      computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                      Eigen::Matrix3f &covariance_matrix,
                                      Eigen::Vector4f &centroid);

      /** \brief Solve the eigenvalues and eigenvectors of a given 3x3 covariance matrix, and estimate the least-squares
        * plane normal and surface curvature.
        * \param[in] covariance_matrix the 3x3 covariance matrix
        * \param[out] (nx ny nz) plane_parameters the resultant plane parameters as: a, b, c, d (ax + by + cz + d = 0)
        * \param[out] curvature the estimated surface curvature as a measure of
        */
      void 
      solvePlaneParameters (const Eigen::Matrix3f &covariance_matrix,
                            float &nx, float &ny, float &nz, float &curvature);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/sampling_surface_normal.hpp>
#endif
