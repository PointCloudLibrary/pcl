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

#include <pcl/pcl_exports.h>

#include <pcl/cuda/sample_consensus/sac_model_1point_plane.h>
#include <pcl/cuda/common/eigen.h>
#include <pcl/cuda/cutil_math.h>

#include <thrust/copy.h>
#include <thrust/count.h>
#include <thrust/random.h>

#include <vector_types.h>
#include <stdio.h>
#include <limits>

// specify inlier computation method
//#define KINECT_NORMALS
#define KINECT

namespace pcl
{
  namespace cuda
  {

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> 
    SampleConsensusModel1PointPlane<Storage>::SampleConsensusModel1PointPlane (
        const PointCloudConstPtr &cloud) : 
      SampleConsensusModel<Storage> (cloud)
    {
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> void
    SampleConsensusModel1PointPlane<Storage>::getSamples (int &iterations, Indices &samples)
    {
      samples.resize (1);
      float trand = indices_->size () / (RAND_MAX + 1.0f);
      int idx = (int)(rngl_ () * trand);
      samples[0] = (*indices_)[idx];
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> bool
    SampleConsensusModel1PointPlane<Storage>::computeModelCoefficients (
        const Indices &samples, Coefficients &model_coefficients)
    {
      if (samples.size () != 1)
        return (false);

    /*  if (isnan ((PointXYZRGB)input_->points[samples[0]]).x ||
          isnan ((PointXYZRGB)input_->points[samples[1]]).x ||
          isnan ((PointXYZRGB)input_->points[samples[2]]).x)
        return (false);*/

      float3 normal;
      normal.x = 0;
      normal.y = 0;
      normal.z = -1;

      // Compute the plane coefficients
      // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
      float3 mc = normalize (normal);

      if (model_coefficients.size () != 4)
        model_coefficients.resize (4);
      model_coefficients[0] = mc.x;
      model_coefficients[1] = mc.y;
      model_coefficients[2] = mc.z;
      // ... + d = 0
      model_coefficients[3] = -1 * dot (mc, ((PointXYZRGB)input_->points[samples[0]]).xyz);

      return (true);
    }

    __host__ __device__
    unsigned int hash(unsigned int a)
    {
        a = (a+0x7ed55d16) + (a<<12);
        a = (a^0xc761c23c) ^ (a>>19);
        a = (a+0x165667b1) + (a<<5);
        a = (a+0xd3a2646c) ^ (a<<9);
        a = (a+0xfd7046c5) + (a<<3);
        a = (a^0xb55a4f09) ^ (a>>16);
        return a;
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> 
    //template <typename Tuple> 
    thrust::tuple <int, float4>
    Create1PointPlaneSampleHypothesis<Storage>::operator () (int t)
    {
      float4 coeff;
      coeff.x = coeff.y = coeff.z = coeff.w = 5;

      float trand = (float) nr_indices / (thrust::default_random_engine::max + 1.0f);
      //rng.seed (hash (t));

      //int sample_point = indices[(int)(rng () * trand)];
      int sample_point = indices[(int)(t * trand)];

      if (isnan (input[sample_point].x))
        return (thrust::make_tuple (sample_point, coeff));

#if 0
      //TODO:: kind of important:  get normal! :D
      int xIdx = sample_point % width_;
      int yIdx = sample_point / width_;

      //int counter = 1;

      int window_size = 3;
      int left_index = 0;
      int top_index = 0;
      // West
      if (xIdx >= window_size)
      {
        left_index = sample_point - window_size;
      }
      else
      {
        left_index = sample_point + window_size;
      }

      // North
      if (yIdx >= window_size)
      {
        top_index = sample_point - window_size * width_;
      }
      else
      {
        top_index = sample_point + window_size * width_;
      }

      float3 left_point;

      left_point.x = input[left_index].x - input[sample_point].x;
      left_point.y = input[left_index].y - input[sample_point].y;
      left_point.z = input[left_index].z - input[sample_point].z;

      float3 top_point;

      top_point.x = input[top_index].x - input[sample_point].x;
      top_point.y = input[top_index].y - input[sample_point].y;
      top_point.z = input[top_index].z - input[sample_point].z;

      float3 normal = cross (top_point, left_point);

      // Compute the plane coefficients from the 3 given points in a straightforward manner
      // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
      float3 mc = normalize (normal);

      if (0 == (normal.x) && 0 == (normal.y) && 0 == (normal.z))
      {
        //mc.x = mc.y = 0;
        if (top_point.x == 0 && top_point.y == 0 && top_point.z == 0)
        {
          mc.x = 999999;
          mc.y = input[top_index].x;
          mc.z = input[sample_point].x;
          //mc.z = top_index - sample_point;
          //mc.z = 999999;
        }
        else
        {
          if (left_point.x == 0 && left_point.y == 0 && left_point.z == 0)
          {
            mc.x = mc.y = 888888;
            mc.z = left_index - sample_point;
            //mc.z = 888888;
          }
        }
      }
#else
      float3 mc = make_float3 (normals_[sample_point].x, normals_[sample_point].y, normals_[sample_point].z);
#endif

      coeff.x = mc.x;
      coeff.y = mc.y;
      coeff.z = mc.z;
      // ... + d = 0
      coeff.w = -1 * dot (mc, input[sample_point].xyz);

      return (thrust::make_tuple (sample_point, coeff));
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> 
    //template <typename Tuple> 
    float4
    Create1PointPlaneHypothesis<Storage>::operator () (int t)
    {
      float4 coeff;
      coeff.x = coeff.y = coeff.z = coeff.w = bad_value;

      float trand = nr_indices / (RAND_MAX + 1.0f);
      thrust::default_random_engine rng (t);

      int sample_point = indices[(int)(rng () * trand)];

      if (isnan (input[sample_point].x))
        return (coeff);

      //TODO:: kind of important:  get normal! :D

      //int xIdx = sample_point % width;
      //int yIdx = sample_point / width;

      //float3 b = input[sample_point];
      //int counter = 1;

      //// West
      //if (xIdx < width-window_size)
      //{
      //  b += input[sample_point + window_size];
      //  counter += 1
      //}

      //// North
      //if (yIdx >= window_size)
      //{
      //  b += input[sample_point - window_size * width];
      //}

      //// South
      //if (yIdx < height-window_size)
      //{
      //  b += input[sample_point + window_size * width];
      //}

      //// East
      //if (xIdx >= window_size)
      //{
      //  b += input[sample_point + window_size];
      //}

      //// Estimate the XYZ centroid
      //compute3DCentroid (cloud, xyz_centroid);

      //// Compute the 3x3 covariance matrix
      //computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);

      //// Get the plane normal and surface curvature
      //solvePlaneParameters (covariance_matrix, xyz_centroid, plane_parameters, curvature);

      //int[5] idxs;
      //idxs[0] = sample_point;
      //  west  = sample_point - window_size;
      //else
      //  west = -1;


      float3 normal;
      normal.x = 0;
      normal.y = 0;
      normal.z = -1;

      // Compute the plane coefficients from the 3 given points in a straightforward manner
      // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
      float3 mc = normalize (normal);

      coeff.x = mc.x;
      coeff.y = mc.y;
      coeff.z = mc.z;
      // ... + d = 0
      coeff.w = -1 * dot (mc, input[sample_point].xyz);

      return (coeff);
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> bool
    SampleConsensusModel1PointPlane<Storage>::generateModelHypotheses (
        Hypotheses &h, int max_iterations)
    {
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
                         parallel_random_generator (0));
      
      //thrust::counting_iterator<int> first (0);
      // Input: Point Cloud, Indices
      // Output: Hypotheses
      transform (//first, first + max_iterations,
                 //index_sequence_begin, 
                 //index_sequence_begin + max_iterations, 
                 randoms.begin (), randoms.begin () + max_iterations,
                 h.begin (), 
                 Create1PointPlaneHypothesis<Storage> (thrust::raw_pointer_cast (&input_->points[0]), 
                                                       thrust::raw_pointer_cast (&(*indices_)[0]),
                                                       indices_->size (), std::numeric_limits<float>::quiet_NaN ()));
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> bool
    SampleConsensusModel1PointPlane<Storage>::generateModelHypotheses (
        Hypotheses &h, Samples &samples, int max_iterations)
    {
      // Create a vector of how many samples/coefficients do we want to get
      h.resize (max_iterations);
      samples.resize (max_iterations);

      typename Storage<int>::type randoms (max_iterations);
      // a sequence counting up from 0 
      thrust::counting_iterator<int> index_sequence_begin (0);
      // transform the range [0,1,2,...N] 
      // to a range of random numbers 
      thrust::transform (index_sequence_begin, 
                         index_sequence_begin + max_iterations, 
                         randoms.begin (), 
                         parallel_random_generator (0));
     

      //thrust::counting_iterator<int> first (0);
      // Input: Point Cloud, Indices
      // Output: Hypotheses
      transform (//first, first + max_iterations,
                 //index_sequence_begin, 
                 //index_sequence_begin + max_iterations, 
                 randoms.begin (), randoms.begin () + max_iterations,
                 //index_sequence_begin, index_sequence_begin + max_iterations,
                 thrust::make_zip_iterator (thrust::make_tuple (samples.begin (), h.begin())),
    //             h.begin (), 
                 Create1PointPlaneSampleHypothesis<Storage> (thrust::raw_pointer_cast (&input_->points[0]), 
                                                             thrust::raw_pointer_cast (&(*normals_)[0]),
                                                             thrust::raw_pointer_cast (&(*indices_)[0]),
                                                             input_->width, input_->height,
                                                             indices_->size (), std::numeric_limits<float>::quiet_NaN ()));
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////
    template <typename Tuple> bool
    CountPlanarInlier::operator () (const Tuple &t)
    {
      if (!isfinite (thrust::raw_reference_cast(thrust::get<0>(t)).x))
        return (false);

      //TODO: make threshold adaptive, depending on z

      return (std::abs (thrust::raw_reference_cast(thrust::get<0>(t)).x * coefficients.x +
                    thrust::raw_reference_cast(thrust::get<0>(t)).y * coefficients.y +
                    thrust::raw_reference_cast(thrust::get<0>(t)).z * coefficients.z + coefficients.w) < threshold);
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    NewCheckPlanarInlier<Storage>::operator () (const int &idx)
    {
      if (idx == -1)
        return -1;

      PointXYZRGB p = input_[idx];
      
      if (isnan (p.x))
        return -1;

      if (std::abs (p.x * coefficients.x +
                p.y * coefficients.y +
                p.z * coefficients.z + coefficients.w) < threshold)
        // If inlier, return its position in the vector
        return idx;
      else
        // If outlier, return -1
        return -1;
    }

    //////////////////////////////////////////////////////////////////////////
    template <typename Tuple> int
    CheckPlanarInlier::operator () (const Tuple &t)
    {
      if (thrust::get<1>(t) == -1)
        return (-1);
      if (isnan (thrust::get<0>(t).x))
        return (-1);
      // Fill in XYZ (and copy NaNs with it)
      float4 pt;
      pt.x = thrust::get<0>(t).x;
      pt.y = thrust::get<0>(t).y;
      pt.z = thrust::get<0>(t).z;
      pt.w = 1;

      //TODO: make threshold adaptive, depending on z

      if (std::abs (dot (pt, coefficients)) < threshold)
        // If inlier, return its position in the vector
        return (thrust::get<1>(t));
      else
        // If outlier, return -1
        return (-1);
    }

    int CheckPlanarInlierKinectIndices::operator () (const PointXYZRGB &pt, const int  &idx)
    {
      //if (isnan (pt.x) | isnan (pt.y) | isnan (pt.z) | (idx == -1))
      //  return (-1);

      const float b = 0.075f;
      const float f = 580.0f/2.0f;
      float length_pt = sqrtf (dot (pt, pt));
      float dot_n_p = pt.x * coefficients.x +
                      pt.y * coefficients.y +
                      pt.z * coefficients.z;
      float D = - coefficients.w * length_pt / dot_n_p - length_pt;
      
      float orig_disparity = b * f / pt.z;
      float actual_disparity = orig_disparity * length_pt / (length_pt + D);

      if ((std::abs (actual_disparity - orig_disparity) <= 1.0/6.0) & idx != -1)
        return (idx);
      else
        return -1;
    }

    template <typename Tuple>
    int CheckPlanarInlierKinectNormalIndices::operator () (const Tuple &t, const int  &idx)
    {
      //if (isnan (pt.x) | isnan (pt.y) | isnan (pt.z) | (idx == -1))
      //  return (-1);

      const PointXYZRGB &pt = thrust::get<0>(t);
      float4 &normal = thrust::get<1>(t);

      const float b = 0.075f;
      const float f = 580.0f/2.0f;
      float length_pt = sqrtf (dot (pt, pt));
      float dot_n_p = pt.x * coefficients.x +
                      pt.y * coefficients.y +
                      pt.z * coefficients.z;
      float D = - coefficients.w * length_pt / dot_n_p - length_pt;
      
      float orig_disparity = b * f / pt.z;
      float actual_disparity = orig_disparity * length_pt / (length_pt + D);

      if ((std::abs (actual_disparity - orig_disparity) <= 1.0/2.0) & (idx != -1)
          &
            (
              std::abs (std::acos (normal.x*coefficients.x + normal.y*coefficients.y + normal.z*coefficients.z)) < angle_threshold
              |
              std::abs (std::acos (-(normal.x*coefficients.x + normal.y*coefficients.y + normal.z*coefficients.z))) < angle_threshold
            )
         )
        return (idx);
      else
        return -1;
    }

    template <typename Tuple>
    int CheckPlanarInlierNormalIndices::operator () (const Tuple &t, const int  &idx)
    {
      const PointXYZRGB &pt = thrust::get<0>(t);
      if (isnan (pt.x) | isnan (pt.y) | isnan (pt.z) | (idx == -1))
        return (-1);

      float4 &normal = thrust::get<1>(t);
      //TODO: make threshold adaptive, depending on z

      if (std::abs (pt.x * coefficients.x +
                pt.y * coefficients.y +
                pt.z * coefficients.z + coefficients.w) < threshold
          &
            (
              std::abs (std::acos (normal.x*coefficients.x + normal.y*coefficients.y + normal.z*coefficients.z)) < angle_threshold
              |
              std::abs (std::acos (-(normal.x*coefficients.x + normal.y*coefficients.y + normal.z*coefficients.z))) < angle_threshold
            )
          )
        // If inlier, return its position in the vector
        return (idx);
      else
        // If outlier, return -1
        return (-1);
    }

    int CheckPlanarInlierIndices::operator () (const PointXYZRGB &pt, const int  &idx)
    {
      if (idx == -1)
        return (-1);
      if (isnan (pt.x) | isnan (pt.y) | isnan (pt.z))
        return (-1);

      //TODO: make threshold adaptive, depending on z

      if (std::abs (pt.x * coefficients.x +
                pt.y * coefficients.y +
                pt.z * coefficients.z + coefficients.w) < threshold)
        // If inlier, return its position in the vector
        return (idx);
      else
        // If outlier, return -1
        return (-1);
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModel1PointPlane<Storage>::countWithinDistance (
        const Coefficients &model_coefficients, float threshold)
    {
      // Needs a valid set of model coefficients
      if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[pcl::cuda::SampleConsensusModel1PointPlane::countWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
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
    SampleConsensusModel1PointPlane<Storage>::countWithinDistance (
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
    SampleConsensusModel1PointPlane<Storage>::selectWithinDistance (
        const Coefficients &model_coefficients, float threshold, IndicesPtr &inliers, IndicesPtr &inliers_stencil)
    {
      // Needs a valid set of model coefficients
      if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[SampleConsensusModel1PointPlane::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
        return 0;
      }

      int nr_points = (int) indices_->size ();
      {
    //  pcl::ScopeTime t ("Resize inl");
      if (!inliers_stencil)
        inliers_stencil.reset (new Indices());

      inliers_stencil->resize (nr_points);
      }

      float4 coefficients;
      coefficients.x = model_coefficients[0];
      coefficients.y = model_coefficients[1];
      coefficients.z = model_coefficients[2];
      coefficients.w = model_coefficients[3];

      {
    //  pcl::ScopeTime t ("transform");
      // Send the data to the device
      transform (
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
                             nr_points,
          inliers_stencil->begin (), 
          CheckPlanarInlier (coefficients, threshold));
      }

      {
    //  pcl::ScopeTime t ("Resize all");
      if (!inliers)
        inliers.reset (new Indices());
      inliers->resize (nr_points);
      }
      typename Indices::iterator it;
      {
    //  pcl::ScopeTime t ("copy-if");
      // Copy data
      it = copy_if (inliers_stencil->begin (), inliers_stencil->end (), inliers->begin (), isInlier ());
      //it = remove_copy (inliers_stencil->begin (), inliers_stencil->end (), inliers->begin (), -1);
      }
      {
    //  pcl::ScopeTime t ("Resize");
      inliers->resize (it - inliers->begin ());
      }
      return (int) inliers->size();
    }

    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModel1PointPlane<Storage>::selectWithinDistance (
        const Hypotheses &h, int idx, float threshold, IndicesPtr &inliers, IndicesPtr &inliers_stencil)
    {
      // Needs a valid set of model coefficients
    /*  if (model_coefficients.size () != 4)
      {
        fprintf (stderr, "[SampleConsensusModel1PointPlane::selectWithinDistance] Invalid number of model coefficients given (%lu)!\n", (unsigned long) model_coefficients.size ());
        return;
      }*/

      int nr_points = (int) indices_->size ();
      {
    //  pcl::ScopeTime t ("Resize inl");

      if (!inliers_stencil)
        inliers_stencil.reset (new Indices());

      inliers_stencil->resize (nr_points);
      }

      float4 coefficients;
      coefficients.x = ((float4)h[idx]).x;
      coefficients.y = ((float4)h[idx]).y;
      coefficients.z = ((float4)h[idx]).z;
      coefficients.w = ((float4)h[idx]).w;

      {
    //  pcl::ScopeTime t ("transform");
      // Send the data to the device
      transform (
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
          make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
                             nr_points,
          inliers_stencil->begin (), 
          CheckPlanarInlier (coefficients, threshold));
      }

      {
    //  pcl::ScopeTime t ("Resize all");
      if (!inliers)
        inliers.reset (new Indices());
      inliers->resize (nr_points);
      }
      typename Indices::iterator it;
      {
    //  pcl::ScopeTime t ("copy-if");
      // Copy data
      it = copy_if (inliers_stencil->begin (), inliers_stencil->end (), inliers->begin (), isInlier ());
      }
      {
    //  pcl::ScopeTime t ("Resize");
      inliers->resize (it - inliers->begin ());
      }
      return (int) inliers->size ();
    }



    //////////////////////////////////////////////////////////////////////////
    template <template <typename> class Storage> int
    SampleConsensusModel1PointPlane<Storage>::selectWithinDistance (
        Hypotheses &h, int idx, float threshold, IndicesPtr &inliers_stencil, float3 &c)
    {
      float angle_threshold = 0.26f;

      int nr_points = (int) indices_stencil_->size ();
      float bad_point = std::numeric_limits<float>::quiet_NaN ();

      if (!inliers_stencil)
        inliers_stencil.reset (new Indices());
      inliers_stencil->resize (nr_points);

      // necessary for the transform_if call below (since not all elements get written, we init with -1)..
      //inliers_stencil->resize (nr_points, -1);

      float4 coefficients;
      coefficients.x = ((float4)h[idx]).x;
      coefficients.y = ((float4)h[idx]).y;
      coefficients.z = ((float4)h[idx]).z;
      coefficients.w = ((float4)h[idx]).w;

      if (isnan (coefficients.x) | 
          isnan (coefficients.y) | 
          isnan (coefficients.z) | 
          isnan (coefficients.w) )
      {
        c.x = c.y = c.z = 0;
        return 0;
      }

      float3 best_centroid;
      IndicesPtr best_inliers_stencil;
      
      float3 centroid;

      centroid.x = centroid.y = centroid.z = 0;
      best_centroid = centroid;

      //ORIG
      //  transform (
      //      make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
      //      make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
      //                         nr_points,
      //      inliers_stencil->begin (), 
      //      CheckPlanarInlier (coefficients, threshold));

      // this is just as fast as the ORIG version, but requires initialization to -1 (see above) --> much slower
      //  transform_if (
      //      make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())),
      //      make_zip_iterator (make_tuple (input_->points.begin (), indices_->begin ())) + 
      //                         nr_points,
      //      indices_->begin(),
      //      inliers_stencil->begin (), 
      //      CheckPlanarInlier (coefficients, threshold),
      //      isInlier () 
      //      );

      // i forgot why this was slow. but it was. :)
      //  transform (
      //      indices_stencil_->begin (),
      //      indices_stencil_->end(),
      //      inliers_stencil->begin (), 
      //      NewCheckPlanarInlier<Storage> (coefficients, (float)threshold, input_->points));

      // compute inliers
      // fastest
#ifdef KINECT
      // NOTE: this performs inlier checks with kinect disparity error model, without normal check
      transform (
          input_->points.begin (), input_->points.end (),
          indices_stencil_->begin (),
          inliers_stencil->begin (), 
          CheckPlanarInlierKinectIndices (coefficients, threshold, angle_threshold));
#endif

#ifdef KINECT_NORMALS
      // NOTE: this performs inlier checks with kinect disparity error model, with normal check
      transform (
          make_zip_iterator (make_tuple (input_->points.begin (), normals_->begin())),
          make_zip_iterator (make_tuple (input_->points.begin (), normals_->begin())) + nr_points,
          indices_stencil_->begin (),
          inliers_stencil->begin (), 
          CheckPlanarInlierKinectNormalIndices (coefficients, threshold, angle_threshold));
#endif

      // store inliers here
      Indices inliers;
      inliers.resize (indices_->size ()); // is this necessary?
      
      typename Indices::iterator last = thrust::remove_copy (inliers_stencil->begin (), inliers_stencil->end (), inliers.begin (), -1);
      inliers.erase (last, inliers.end ());

      if (inliers.size () < 1)
        return (int) inliers.size ();

      best_inliers_stencil = inliers_stencil;
      int best_nr_inliers = (int) inliers.size ();

      int nr_inliers_after_refit = (int) inliers.size ();
      int nr_inliers_before_refit;
      int nr_refit_iterations = 0;

      do {
        nr_inliers_before_refit = nr_inliers_after_refit;

        compute3DCentroid (make_permutation_iterator (input_->points.begin (), inliers.begin ()),
                         make_permutation_iterator (input_->points.begin (), inliers.end ()),
                         centroid);

        if (isnan (centroid.x) | isnan (centroid.y) | isnan (centroid.z))
        {
          std::cerr << "Wow, centroid contains nans!" << std::endl;
      
          inliers_stencil = best_inliers_stencil;
          c = make_float3 (bad_point, bad_point, bad_point);
          //best_centroid;
          return best_nr_inliers;
        }
        
        // Note: centroid contains centroid * inliers.size() at this point !
#if 0
        std::cerr << "----------------------------------------------------------------------------" << std::endl;
        std::cerr << "inliers before: " << inliers.size () << std::endl;
        std::cerr << "Centroid: " << 
          centroid.x << ", " << centroid.y << ", " << centroid.z << ", " << std::endl;
#endif

        CovarianceMatrix covariance_matrix;

        computeCovariance (make_permutation_iterator (input_->points.begin (), inliers.begin ()),
                           make_permutation_iterator (input_->points.begin (), inliers.end ()),
                           covariance_matrix, centroid);

        if (isnan (covariance_matrix.data[0].x))
        {
          std::cerr << "Wow, covariance matrix contains nans!" << std::endl;
          inliers_stencil = best_inliers_stencil;
          c = make_float3 (bad_point, bad_point, bad_point);
              //best_centroid;
          return best_nr_inliers;
        }

#if 0
        std::cerr << "Covariance: " << 
          covariance_matrix.data[0].x << ", " << covariance_matrix.data[0].y << ", " << covariance_matrix.data[0].z << std::endl << 
          covariance_matrix.data[1].x << ", " << covariance_matrix.data[1].y << ", " << covariance_matrix.data[1].z << std::endl << 
          covariance_matrix.data[2].x << ", " << covariance_matrix.data[2].y << ", " << covariance_matrix.data[2].z << std::endl;
#endif

        CovarianceMatrix evecs;
        float3 evals;

        // compute eigenvalues and -vectors
        eigen33 (covariance_matrix, evecs, evals);

        float3 mc = normalize (evecs.data[0]);

#if 0
        std::cerr << "Eigenvectors: " << 
          evecs.data[0].x << ", " << evecs.data[0].y << ", " << evecs.data[0].z << std::endl << 
          evecs.data[1].x << ", " << evecs.data[1].y << ", " << evecs.data[1].z << std::endl << 
          evecs.data[2].x << ", " << evecs.data[2].y << ", " << evecs.data[2].z << std::endl;
        std::cerr << "Coefficients before: " << 
          coefficients.x << ", " << coefficients.y << ", " << coefficients.z << ", " << coefficients.w << ", " << std::endl;
#endif
       
        // compute plane coefficients from eigenvector corr. to smallest eigenvalue and centroid
        coefficients.x = mc.x;
        coefficients.y = mc.y;
        coefficients.z = mc.z;
        // ... + d = 0
        coefficients.w = -1 * dot (mc, centroid);

#if 0
        std::cerr << "Coefficients after: " << 
          coefficients.x << ", " << coefficients.y << ", " << coefficients.z << ", " << coefficients.w << ", " << std::endl;
#endif

        // finally, another inlier check:
#ifdef KINECT
        transform (
          input_->points.begin (), input_->points.end (),
          //make_zip_iterator (make_tuple (input_->points.begin (), normals_.begin())),
          //make_zip_iterator (make_tuple (input_->points.begin (), normals_.begin())) + nr_points,
    //        input_->points.begin (),
    //        input_->points.end (),
            indices_stencil_->begin (),
            inliers_stencil->begin (), 
            CheckPlanarInlierKinectIndices (coefficients, threshold, angle_threshold));
#endif

#ifdef KINECT_NORMALS
        transform (
            make_zip_iterator (make_tuple (input_->points.begin (), normals_->begin())),
            make_zip_iterator (make_tuple (input_->points.begin (), normals_->begin())) + nr_points,
            indices_stencil_->begin (),
            inliers_stencil->begin (), 
            CheckPlanarInlierKinectNormalIndices (coefficients, threshold, angle_threshold));
#endif

        // copy inliers from stencil to inlier vector
        inliers.resize (inliers_stencil->size ()); // is this necessary?
        last = thrust::remove_copy (inliers_stencil->begin (), inliers_stencil->end (), inliers.begin (), -1);
        inliers.erase (last, inliers.end ());

        nr_inliers_after_refit = (int) inliers.size ();

        compute3DCentroid (make_permutation_iterator (input_->points.begin (), inliers.begin ()),
                         make_permutation_iterator (input_->points.begin (), inliers.end ()),
                         centroid);

        if (nr_inliers_after_refit > best_nr_inliers)
        {
          best_nr_inliers = nr_inliers_after_refit;
          best_inliers_stencil = inliers_stencil;
          best_centroid = centroid;
          h[idx] = coefficients;
        }

        //fprintf (stderr, "iteration %i: %f, %f, %f, %f  ---> %i\n", nr_refit_iterations, coefficients.x, coefficients.y, coefficients.z, coefficients.w, best_nr_inliers);

      } while (nr_inliers_after_refit > nr_inliers_before_refit & ++nr_refit_iterations < 120);

#if 0
      std::cerr << "inliers after: " << nr_inliers_after_refit << std::endl;
#endif
      //std::cerr << "--> refitting steps: " << nr_refit_iterations << std::endl;

      inliers_stencil = best_inliers_stencil;
      c = best_centroid;
      return best_nr_inliers;
    }


    // explicit template instantiation for device and host
    template class PCL_EXPORTS SampleConsensusModel1PointPlane<Device>;
    template class PCL_EXPORTS SampleConsensusModel1PointPlane<Host>;

  } // namespace
} // namespace
