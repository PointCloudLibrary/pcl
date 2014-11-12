/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 */


#ifndef PCL_TRACKING_IMPL_KLD_ADAPTIVE_PARTICLE_OMP_FILTER_H_
#define PCL_TRACKING_IMPL_KLD_ADAPTIVE_PARTICLE_OMP_FILTER_H_

#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>

template <typename PointInT, typename StateT> void
pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointInT, StateT>::weight ()
{
  if (!use_normal_)
  {
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < particle_num_; i++)
      this->computeTransformedPointCloudWithoutNormal (particles_->points[i], *transed_reference_vector_[i]);
    
    PointCloudInPtr coherence_input (new PointCloudIn);
    this->cropInputPointCloud (input_, *coherence_input);
    if (change_counter_ == 0)
    {
      // test change detector
      if (!use_change_detector_ || this->testChangeDetection (coherence_input))
      {
        changed_ = true;
        change_counter_ = change_detector_interval_;
        coherence_->setTargetCloud (coherence_input);
        coherence_->initCompute ();
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
        for (int i = 0; i < particle_num_; i++)
        {
          IndicesPtr indices;
          coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
        }
      }
      else
        changed_ = false;
    }
    else
    {
      --change_counter_;
      coherence_->setTargetCloud (coherence_input);
      coherence_->initCompute ();
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
      for (int i = 0; i < particle_num_; i++)
      {
        IndicesPtr indices;
        coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
      }
    }
  }
  else
  {
    std::vector<IndicesPtr> indices_list (particle_num_);
    for (int i = 0; i < particle_num_; i++)
    {
      indices_list[i] = IndicesPtr (new std::vector<int>);
    }
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < particle_num_; i++)
    {
      this->computeTransformedPointCloudWithNormal (particles_->points[i], *indices_list[i], *transed_reference_vector_[i]);
    }
    
    PointCloudInPtr coherence_input (new PointCloudIn);
    this->cropInputPointCloud (input_, *coherence_input);
    
    coherence_->setTargetCloud (coherence_input);
    coherence_->initCompute ();
#ifdef _OPENMP
#pragma omp parallel for num_threads(threads_)
#endif
    for (int i = 0; i < particle_num_; i++)
    {
      coherence_->compute (transed_reference_vector_[i], indices_list[i], particles_->points[i].weight);
    }
  }
  
  normalizeWeight ();
}

#define PCL_INSTANTIATE_KLDAdaptiveParticleFilterOMPTracker(T,ST) template class PCL_EXPORTS pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<T,ST>;

#endif
