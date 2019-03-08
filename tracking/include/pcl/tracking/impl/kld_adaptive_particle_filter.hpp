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


#ifndef PCL_TRACKING_IMPL_KLD_ADAPTIVE_PARTICLE_FILTER_H_
#define PCL_TRACKING_IMPL_KLD_ADAPTIVE_PARTICLE_FILTER_H_

#include <pcl/tracking/kld_adaptive_particle_filter.h>

template <typename PointInT, typename StateT> bool
pcl::tracking::KLDAdaptiveParticleFilterTracker<PointInT, StateT>::initCompute ()
{
  if (!Tracker<PointInT, StateT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  if (transed_reference_vector_.empty ())
  {
    // only one time allocation
    transed_reference_vector_.resize (maximum_particle_number_);
    for (unsigned int i = 0; i < maximum_particle_number_; i++)
    {
      transed_reference_vector_[i] = PointCloudInPtr (new PointCloudIn ());
    }
  }
  
  coherence_->setTargetCloud (input_);

  if (!change_detector_)
    change_detector_ = boost::shared_ptr<pcl::octree::OctreePointCloudChangeDetector<PointInT> >(new pcl::octree::OctreePointCloudChangeDetector<PointInT> (change_detector_resolution_));
  
  if (!particles_ || particles_->points.empty ())
    initParticles (true);
  return (true);
}

template <typename PointInT, typename StateT> bool
pcl::tracking::KLDAdaptiveParticleFilterTracker<PointInT, StateT>::insertIntoBins
(std::vector<int> &&new_bin, std::vector<std::vector<int> > &bins)
{
  for (auto &existing_bin : bins)
  {
    if (equalBin (new_bin, existing_bin))
      return false;
  }
  bins.push_back (std::move(new_bin));
  return true;
}

template <typename PointInT, typename StateT> void
pcl::tracking::KLDAdaptiveParticleFilterTracker<PointInT, StateT>::resample ()
{
  unsigned int k = 0;
  unsigned int n = 0;
  PointCloudStatePtr S (new PointCloudState);
  std::vector<std::vector<int> > bins;
  
  // initializing for sampling without replacement
  std::vector<int> a (particles_->points.size ());
  std::vector<double> q (particles_->points.size ());
  this->genAliasTable (a, q, particles_);
  
  const std::vector<double> zero_mean (StateT::stateDimension (), 0.0);
  
  // select the particles with KLD sampling
  do
  {
    int j_n = sampleWithReplacement (a, q);
    StateT x_t = particles_->points[j_n];
    x_t.sample (zero_mean, step_noise_covariance_);
    
    // motion
    if (rand () / double (RAND_MAX) < motion_ratio_)
      x_t = x_t + motion_;
    
    S->points.push_back (x_t);
    // calc bin
    std::vector<int> new_bin (StateT::stateDimension ());
    for (int i = 0; i < StateT::stateDimension (); i++)
      new_bin[i] = static_cast<int> (x_t[i] / bin_size_[i]);
    
    // calc bin index... how?
    if (insertIntoBins (std::move(new_bin), bins))
      ++k;
    ++n;
  }
  while (n < maximum_particle_number_ && (k < 2 || n < calcKLBound (k)));
  
  particles_ = S;               // swap
  particle_num_ = static_cast<int> (particles_->points.size ());
}


#define PCL_INSTANTIATE_KLDAdaptiveParticleFilterTracker(T,ST) template class PCL_EXPORTS pcl::tracking::KLDAdaptiveParticleFilterTracker<T,ST>;

#endif
