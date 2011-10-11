#ifndef PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterOMPTracker<PointInT, StateT>::weight ()
{
//  if (!use_normal_)
  {
#pragma omp parallel for schedule (dynamic, threads_)
    for (int i = 0; i < particle_num_; i++)
    {
      IndicesPtr indices;
      //computeTransformedPointCloud (particles_->points[i], *indices, *transed_reference_vector_[i]);
      computeTransformedPointCloudWithoutNormal (particles_->points[i], *transed_reference_vector_[i]);
    }
    
    PointCloudInPtr coherence_input (new PointCloudIn);
    cropInputPointCloud (input_, *coherence_input);
    
    coherence_->setTargetCloud (coherence_input);
    coherence_->initCompute ();
    
#pragma omp parallel for schedule (dynamic, threads_)
    for (int i = 0; i < particle_num_; i++)
    {
      IndicesPtr indices;
      coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
    }
  }
//   else
//   {
// #pragma omp parallel for schedule (dynamic, threads_)
//     for (int i = 0; i < particle_num_; i++)
//     {
//       IndicesPtr indices (new std::vector<int>);
//       computeTransformedPointCloud (particles_->points[i], *indices, *transed_reference_vector_[i]);
//     }
    
//     PointCloudInPtr coherence_input (new PointCloudIn);
//     cropInputPointCloud (input_, *coherence_input, transed_reference_vector_);
    
//     coherence_->setTargetCloud (coherence_input);
//     coherence_->initCompute ();
// #pragma omp parallel for schedule (dynamic, threads_)
//     for (int i = 0; i < particle_num_; i++)
//     {
//       IndicesPtr indices (new std::vector<int>);
//       coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
//     }
//   }
  
  normalizeWeight ();
}

#define PCL_INSTANTIATE_ParticleFilterOMPTracker(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterOMPTracker<T,ST>;

#endif
