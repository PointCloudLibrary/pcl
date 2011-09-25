#ifndef PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterOMPTracker<PointInT, StateT>::weight ()
{
  coherence_->initCompute ();
#pragma omp parallel for schedule (dynamic, threads_)
  for ( int i = 0; i < particle_num_; i++ )
  {
    double likelihood = calcLikelihood (particles_->points[i]);
    particles_->points[i].weight = likelihood;
  }
  normalizeWeight ();
}

#define PCL_INSTANTIATE_ParticleFilterOMPTracker(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterOMPTracker<T,ST>;

#endif
