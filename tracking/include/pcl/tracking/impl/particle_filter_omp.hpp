#ifndef PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_

#include <pcl/tracking/particle_filter_omp.h>

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterOMPTracker<PointInT, StateT>::weight ()
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
          IndicesPtr indices;   // dummy
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
        IndicesPtr indices;     // dummy
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

#define PCL_INSTANTIATE_ParticleFilterOMPTracker(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterOMPTracker<T,ST>;

#endif
