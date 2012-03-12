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
 * $Id: normal_3d.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#ifndef PCL_CUDA_NORMAL_3D_H_
#define PCL_CUDA_NORMAL_3D_H_

#include <pcl/pcl_exports.h>

#include <pcl/cuda/common/eigen.h>

namespace pcl
{
  namespace cuda
  {

    template <template <typename> class Storage>
    struct NormalEstimationKernel
    {
      typedef boost::shared_ptr <const PointCloudAOS <Storage> > CloudConstPtr;
      NormalEstimationKernel (const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float focallength, float sqr_radius, float sqrt_desired_nr_neighbors)
        : points_ (thrust::raw_pointer_cast(&input->points[0]))
        , focallength_ (focallength)
        , search_ (input, focallength, sqr_radius)
        , sqr_radius_(sqr_radius)
        , sqrt_desired_nr_neighbors_ (sqrt_desired_nr_neighbors)
      {}
  
      inline __host__ __device__
      float4 operator () (float3 query_pt)
      {
        CovarianceMatrix cov;
        int nnn = 0;
        if (!isnan (query_pt.x))
          nnn = 
          search_.computeCovarianceOnline (query_pt, cov, sqrt_desired_nr_neighbors_);
        else
          return make_float4(query_pt.x);
  
        CovarianceMatrix evecs;
        float3 evals;
        // compute eigenvalues and -vectors
        if (nnn <= 1)
          return make_float4(0);
  
        eigen33 (cov, evecs, evals);
        //float curvature = evals.x / (evals.x + evals.y + evals.z);
        float curvature = evals.x / (query_pt.z * (0.2f / 4.0f) * query_pt.z * (0.2f / 4.0f));
  
        float3 mc = normalize (evecs.data[0]);
        // TODO: this should be an optional step, as it slows down eveything
        // btw, this flips the normals to face the origin (assumed to be the view point)
        if ( dot (query_pt, mc) > 0 )
          mc = -mc;
        return make_float4 (mc.x, mc.y, mc.z, curvature);
      }
  
      const PointXYZRGB *points_;
      float focallength_;
      OrganizedRadiusSearch<CloudConstPtr> search_;
      float sqr_radius_;
      float sqrt_desired_nr_neighbors_;
    };

    template <template <typename> class Storage>
    struct FastNormalEstimationKernel
    {
      FastNormalEstimationKernel (const boost::shared_ptr <const PointCloudAOS <Storage> > &input, int width, int height)
        : points_ (thrust::raw_pointer_cast(&input->points[0])), width_(width), height_(height)
      {}
  
      inline __host__ __device__
      float4 operator () (int idx)
      {
        float3 query_pt = points_[idx];
        if (isnan(query_pt.z))
          return make_float4 (0.0f,0.0f,0.0f,0.0f);

        int xIdx = idx % width_;
        int yIdx = idx / width_;

        // are we at a border? are our neighbor valid points?
        bool west_valid  = (xIdx > 1)         && !isnan (points_[idx-1].z) &&      fabs (points_[idx-1].z - query_pt.z) < 200;
        bool east_valid  = (xIdx < width_-1)  && !isnan (points_[idx+1].z) &&      fabs (points_[idx+1].z - query_pt.z) < 200;
        bool north_valid = (yIdx > 1)         && !isnan (points_[idx-width_].z) && fabs (points_[idx-width_].z - query_pt.z) < 200;
        bool south_valid = (yIdx < height_-1) && !isnan (points_[idx+width_].z) && fabs (points_[idx+width_].z - query_pt.z) < 200;

        float3 horiz, vert;
        if (west_valid & east_valid)
          horiz = points_[idx+1] - points_[idx-1];
        if (west_valid & !east_valid)
          horiz = points_[idx] - points_[idx-1];
        if (!west_valid & east_valid)
          horiz = points_[idx+1] - points_[idx];
        if (!west_valid & !east_valid)
          return make_float4 (0.0f,0.0f,0.0f,1.0f);

        if (south_valid & north_valid)
          vert = points_[idx-width_] - points_[idx+width_];
        if (south_valid & !north_valid)
          vert = points_[idx] - points_[idx+width_];
        if (!south_valid & north_valid)
          vert = points_[idx-width_] - points_[idx];
        if (!south_valid & !north_valid)
          return make_float4 (0.0f,0.0f,0.0f,1.0f);

        float3 normal = cross (horiz, vert);

        float curvature = length (normal);
        curvature = fabs(horiz.z) > 0.04 | fabs(vert.z) > 0.04 | !west_valid | !east_valid | !north_valid | !south_valid;

        float3 mc = normalize (normal);
        if ( dot (query_pt, mc) > 0 )
          mc = -mc;
        return make_float4 (mc.x, mc.y, mc.z, curvature);
      }
  
      const PointXYZRGB *points_;
      int width_;
      int height_;
    };
  
    template <template <typename> class Storage>
    struct NormalDeviationKernel
    {
      typedef boost::shared_ptr <const PointCloudAOS <Storage> > CloudConstPtr;
      NormalDeviationKernel (const boost::shared_ptr <const PointCloudAOS <Storage> > &input, float focallength, float sqr_radius, float sqrt_desired_nr_neighbors)
        : points_ (thrust::raw_pointer_cast(&input->points[0]))
        , focallength_ (focallength)
        , search_ (input, focallength, sqr_radius)
        , sqr_radius_(sqr_radius)
        , sqrt_desired_nr_neighbors_ (sqrt_desired_nr_neighbors)
      {}
  
      template <typename Tuple>
      inline __host__ __device__
      float4 operator () (Tuple &t)
      {
        float3 query_pt = thrust::get<0>(t);
        float4 normal = thrust::get<1>(t);
        CovarianceMatrix cov;
        float3 centroid;
        if (!isnan (query_pt.x))
          centroid = search_.computeCentroid (query_pt, cov, sqrt_desired_nr_neighbors_);
        else
          return make_float4(query_pt.x);

        float proj = normal.x * (query_pt.x - centroid.x) / sqrt(sqr_radius_) + 
                     normal.y * (query_pt.y - centroid.y) / sqrt(sqr_radius_) + 
                     normal.z * (query_pt.z - centroid.z) / sqrt(sqr_radius_) ; 


        //return make_float4 (normal.x*proj, normal.y*proj, normal.z*proj, clamp (fabs (proj), 0.0f, 1.0f));
        return make_float4 (
           (centroid.x - query_pt.x) / sqrt(sqr_radius_) ,
           (centroid.y - query_pt.y) / sqrt(sqr_radius_) ,
           (centroid.z - query_pt.z) / sqrt(sqr_radius_) ,
           0);
      }
  
      const PointXYZRGB *points_;
      float focallength_;
      OrganizedRadiusSearch<CloudConstPtr> search_;
      float sqr_radius_;
      float sqrt_desired_nr_neighbors_;
    };

  } // namespace
} // namespace

#endif  


