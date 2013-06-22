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

#ifndef PCL_FILTERS_IMPL_NORMAL_REFINEMENT_H_
#define PCL_FILTERS_IMPL_NORMAL_REFINEMENT_H_

#include <pcl/filters/normal_refinement.h>

///////////////////////////////////////////////////////////////////////////////////////////
template <typename NormalT> void
pcl::NormalRefinement<NormalT>::applyFilter (PointCloud &output)
{
  // Check input
  if (input_->empty ())
  {
    PCL_ERROR ("[pcl::%s::applyFilter] No source was input!\n",
               getClassName ().c_str ());
  }
  
  // Copy to output
  output = *input_;
  
  // Check that correspondences are non-empty
  if (k_indices_.empty () || k_sqr_distances_.empty ())
  {
    PCL_ERROR ("[pcl::%s::applyFilter] No point correspondences given! Returning original input.\n",
               getClassName ().c_str ());
    return;
  }

  // Check that correspondences are OK
  const unsigned int size = k_indices_.size ();
  if (k_sqr_distances_.size () != size || input_->size () != size)
  {
    PCL_ERROR ("[pcl::%s::applyFilter] Inconsistency between size of correspondence indices/distances or input! Returning original input.\n",
               getClassName ().c_str ());
    return;
  }
  
  // Run refinement while monitoring convergence
  for (unsigned int i = 0; i < max_iterations_; ++i)
  {
    // Output of the current iteration
    PointCloud tmp = output;
    
    // Mean change in direction, measured by dot products
    float ddot = 0.0f;
    
    // Loop over all points in current output and write refined normal to tmp
    int num_valids = 0;
    for(unsigned int j = 0; j < size; ++j)
    {
      // Point to write to
      NormalT& tmpj = tmp[j];
      
      // Refine
      if (refineNormal (output, j, k_indices_[j], k_sqr_distances_[j], tmpj))
      {
        // Accumulate using similarity in direction between previous iteration and current
        const NormalT& outputj = output[j];
        ddot += tmpj.normal_x * outputj.normal_x + tmpj.normal_y * outputj.normal_y + tmpj.normal_z * outputj.normal_z;
        ++num_valids;
      }
    }
    
    // Take mean of similarities
    ddot /= static_cast<float> (num_valids);
    
    // Negate to since we want an error measure to approach zero
    ddot = 1.0f - ddot;
    
    // Update output
    output = tmp;
    
    // Break if converged
    if (ddot < convergence_threshold_)
    {
      PCL_DEBUG("[pcl::%s::applyFilter] Converged after %i iterations with mean error of %f.\n",
                getClassName ().c_str (), i+1, ddot);
      break;
    }
  }
}

#endif
