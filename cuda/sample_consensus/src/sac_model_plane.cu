/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include "pcl/cuda/sample_consensus/sac_model_plane.h"
#include "pcl/cuda/cutil_math.h"

#include <vector_types.h>
#include <thrust/copy.h>
#include <thrust/count.h>

#include <stdio.h>
#include <limits>


namespace pcl
{
  namespace cuda
  {
    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> 
    SampleConsensusModelPlane<Storage>::SampleConsensusModelPlane (
        const PointCloudConstPtr &cloud) : 
      SampleConsensusModel<Storage> (cloud)
    {
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> void
    SampleConsensusModelPlane<Storage>::getSamples (int &iterations, Indices &samples)
    {
      samples.resize (3);
      float trand = indices_->size () / (RAND_MAX + 1.0f);
      for (int i = 0; i < 3; ++i)
      {
        int idx = (int)(rngl_ () * trand);
        samples[i] = (*indices_)[idx];
      }
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> bool
    SampleConsensusModelPlane<Storage>::computeModelCoefficients (
        const Indices &samples, Coefficients &model_coefficients)
    {
      if (samples.size () != 3)
      {
        return (false);
      }

      // Compute the segment values (in 3d) between p1 and p0
      float3 p1p0 = ((PointXYZRGB)input_->points[samples[1]]).xyz - ((PointXYZRGB)input_->points[samples[0]]).xyz;
      // Compute the segment values (in 3d) between p2 and p0
      float3 p2p0 = ((PointXYZRGB)input_->points[samples[2]]).xyz - ((PointXYZRGB)input_->points[samples[0]]).xyz;

      // Avoid some crashes by checking for collinearity here
      float3 dy1dy2 = p1p0 / p2p0;
      if ( (dy1dy2.x == dy1dy2.y) && (dy1dy2.z == dy1dy2.y) )          // Check for collinearity
        return (false);

      // Compute the plane coefficients from the 3 given points in a straightforward manner
      // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
      float3 mc = normalize (cross (p1p0, p2p0));

      if (model_coefficients.size () != 4)
        model_coefficients.resize (4);
      model_coefficients[0] = mc.x;
      model_coefficients[1] = mc.y;
      model_coefficients[2] = mc.z;
      // ... + d = 0
      model_coefficients[3] = -1 * dot (mc, ((PointXYZRGB)input_->points[samples[0]]).xyz);

      return (true);
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> 
    //template <typename Tuple> 
    float4
    CreatePlaneHypothesis<Storage>::operator () (int t)
    {
      float4 coeff;
      coeff.x = coeff.y = coeff.z = coeff.w = bad_value;

      int3 samples;
      float trand = nr_indices / (RAND_MAX + 1.0f);
      thrust::default_random_engine rng (t);
    //  rng.discard (10);
      samples.x = indices[(int)(rng () * trand)];
    //  rng.discard (20);
      samples.y = indices[(int)(rng () * trand)];
    //  rng.discard (30);
      samples.z = indices[(int)(rng () * trand)];
    /*  samples.x = indices[(int)(thrust::get<0>(t) * trand)];
      samples.y = indices[(int)(thrust::get<1>(t) * trand)];
      samples.z = indices[(int)(thrust::get<2>(t) * trand)];*/

      if (isnan (input[samples.x].x) ||
          isnan (input[samples.y].x) ||
          isnan (input[samples.z].x))
        return (coeff);

      // Compute the segment values (in 3d) between p1 and p0
      float3 p1p0 = input[samples.y].xyz - input[samples.x].xyz;
      // Compute the segment values (in 3d) between p2 and p0
      float3 p2p0 = input[samples.z].xyz - input[samples.x].xyz;

      // Avoid some crashes by checking for collinearity here
      float3 dy1dy2 = p1p0 / p2p0;
      if ( (dy1dy2.x == dy1dy2.y) && (dy1dy2.z == dy1dy2.y) )          // Check for collinearity
        return (coeff);

      // Compute the plane coefficients from the 3 given points in a straightforward manner
      // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
      float3 mc = normalize (cross (p1p0, p2p0));

      coeff.x = mc.x;
      coeff.y = mc.y;
      coeff.z = mc.z;
      // ... + d = 0
      coeff.w = -1 * dot (mc, input[samples.x].xyz);

      return (coeff);
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> bool
    SampleConsensusModelPlane<Storage>::generateModelHypotheses (
        Hypotheses &h, int max_iterations)
    {
      using namespace thrust;

      // Create a vector of how many samples/coefficients do we want to get
      h.resize (max_iterations);

      typename Storage<int>::type randoms (max_iterations);
      // a sequence counting up from 0 
      thrust::counting_iterator<int> index_sequence_begin (0); 
      // transform the range [0,1,2,...N] 
      // to a range of random numbers 
      thrust::transform (index_sequence_begin, 
                         index_sequence_begin + max_iterations, 
                         randoms.begin (), 
                         parallel_random_generator ((int) time (0)));
      
      thrust::counting_iterator<int> first (0);
      // Input: Point Cloud, Indices
      // Output: Hypotheses
      transform (//first, first + max_iterations,
                 randoms.begin (), randoms.begin () + max_iterations,
                 h.begin (), 
                 CreatePlaneHypothesis<Storage> (thrust::raw_pointer_cast (&input_->points[0]), 
                                                 thrust::raw_pointer_cast (&(*indices_)[0]),
                                                 (int) indices_->size (), std::numeric_limits<float>::quiet_NaN ()));
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////
    template <typename Tuple> bool
    CountPlanarInlier::operator () (const Tuple &t)
    {
      if (!isfinite (thrust::raw_reference_cast(thrust::get<0>(t)).x))
        return (false);

      return (std::abs (thrust::raw_reference_cast(thrust::get<0>(t)).x * coefficients.x +
                    thrust::raw_reference_cast(thrust::get<0>(t)).y * coefficients.y +
                    thrust::raw_reference_cast(thrust::get<0>(t)).z * coefficients.z + coefficients.w) < threshold);
    }

    //////////////////////////////////////////////////////////////////////////
    template <typename Tuple> int
    CheckPlanarInlier::operator () (const Tuple &t)
    {
      if (isnan (thrust::get<0>(t).x))
        return (-1);
      // Fill in XYZ (and copy NaNs with it)
      float4 pt;
      pt.x = thrust::get<0>(t).x;
      pt.y = thrust::get<0>(t).y;
      pt.z = thrust::get<0>(t).z;
      pt.w = 1;

      if (std::abs (dot (pt, coefficients)) < threshold)
        // If inlier, return its position in the vector
        return (thrust::get<1>(t));
      else
        // If outlier, return -1
        return (-1);
    }


    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModelPlane<Storage>::countWithinDistance (
        const Coefficients &model_coefficients, float threshold)
    {
      using namespace thrust;

      // Needs a valid set of model coefficients
      if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[pcl::cuda::SampleConsensusModelPlane::countWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
        return 0;
      }

      float4 coefficients;
      coefficients.x = model_coefficients[0];
      coefficients.y = model_coefficients[1];
      coefficients.z = model_coefficients[2];
      coefficients.w = model_coefficients[3];

      return (int) count_if (
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
                             indices_->size (),
          CountPlanarInlier (coefficients, threshold));
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModelPlane<Storage>::countWithinDistance (
        const Hypotheses &h, int idx, float threshold)
    {
      if (isnan (((float4)h[idx]).x))
        return (0);

      return (int)
        (thrust::count_if (
          thrust::make_zip_iterator (thrust::make_tuple (input_->points.begin (), indices_->begin ())),
          thrust::make_zip_iterator (thrust::make_tuple (input_->points.begin (), indices_->begin ())) + 
                             indices_->size (),
          CountPlanarInlier (h[idx], threshold)));
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModelPlane<Storage>::selectWithinDistance (
        const Coefficients &model_coefficients, float threshold, IndicesPtr &inliers, IndicesPtr &inliers_stencil)
    {
      using namespace thrust;

      // Needs a valid set of model coefficients
      if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[pcl::cuda::SampleConsensusModelPlane::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
        return 0;
      }

      int nr_points = (int) indices_->size ();
      if (!inliers_stencil)
        inliers_stencil.reset (new Indices());

      inliers_stencil->resize (nr_points);

      float4 coefficients;
      coefficients.x = model_coefficients[0];
      coefficients.y = model_coefficients[1];
      coefficients.z = model_coefficients[2];
      coefficients.w = model_coefficients[3];

      // Send the data to the device
      transform (
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
                             nr_points,
          inliers_stencil->begin (), 
          CheckPlanarInlier (coefficients, threshold));

      if (!inliers)
        inliers.reset (new Indices());
      inliers->resize (nr_points);

      typename Indices::iterator it = copy_if (inliers_stencil->begin (), inliers_stencil->end (), inliers->begin (), isInlier ());
      // Copy data
      //it = remove_copy (inliers_stencil->begin (), inliers_stencil->end (), inliers->begin (), -1);
      
      inliers->resize (it - inliers->begin ());
      return (int) inliers->size();
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModelPlane<Storage>::selectWithinDistance (
        const Hypotheses &h, int idx, float threshold, IndicesPtr &inliers, IndicesPtr &inliers_stencil)
    {
      using namespace thrust;

      // Needs a valid set of model coefficients
    /*  if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[pcl::cuda::SampleConsensusModelPlane::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
        return;
      }*/

      int nr_points = (int) indices_->size ();

      if (!inliers_stencil)
        inliers_stencil.reset (new Indices());

      inliers_stencil->resize (nr_points);

      float4 coefficients;
      coefficients.x = ((float4)h[idx]).x;
      coefficients.y = ((float4)h[idx]).y;
      coefficients.z = ((float4)h[idx]).z;
      coefficients.w = ((float4)h[idx]).w;

      // Send the data to the device
      transform (
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
                             nr_points,
          inliers_stencil->begin (), 
          CheckPlanarInlier (coefficients, threshold));

      if (!inliers)
        inliers.reset (new Indices());
      inliers->resize (nr_points);

      // Copy data
      typename Indices::iterator it = copy_if (inliers_stencil->begin (), inliers_stencil->end (), inliers->begin (), isInlier ());
      inliers->resize (it - inliers->begin ());
      return (int) inliers->size ();
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModelPlane<Storage>::selectWithinDistance (
        Hypotheses &h, int idx, float threshold, IndicesPtr &inliers_stencil, float3 & centroid)
    {
      using namespace thrust;

      // Needs a valid set of model coefficients
    /*  if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[pcl::cuda::SampleConsensusModelPlane::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
        return;
      }*/

      int nr_points = (int) indices_->size ();

      if (!inliers_stencil)
        inliers_stencil.reset (new Indices());
      inliers_stencil->resize (nr_points);

      float4 coefficients;
      coefficients.x = ((float4)h[idx]).x;
      coefficients.y = ((float4)h[idx]).y;
      coefficients.z = ((float4)h[idx]).z;
      coefficients.w = ((float4)h[idx]).w;

      transform (
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
                             nr_points,
          inliers_stencil->begin (), 
          CheckPlanarInlier (coefficients, threshold));
      return nr_points - (int) thrust::count (inliers_stencil->begin (), inliers_stencil->end (), -1);
    }

    template class SampleConsensusModelPlane<Device>;
    template class SampleConsensusModelPlane<Host>;

  } // namespace
} // namespace

