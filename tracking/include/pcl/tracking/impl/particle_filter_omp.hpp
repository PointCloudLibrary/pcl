#ifndef PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_OMP_FILTER_H_

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterOMPTracker<PointInT, StateT>::weight ()
{
  coherence_->initCompute ();
  std::vector<double> w_i (particle_num_, 1.0);
#pragma omp parallel for schedule (dynamic, threads_)
  for (int i = 0; i < particle_num_; i++)
  {
      // TODO: "new"s requires giant lock?
      IndicesPtr indices = IndicesPtr (new std::vector<int> ());
      PointCloudInPtr transed_reference = PointCloudInPtr (new PointCloudIn ());
      computeTransformedPointCloud (particles_->points[i], *indices, *transed_reference);
      coherence_->compute (transed_reference, indices, particles_->points[i].weight);
  }
  normalizeWeight ();
}

#define PCL_INSTANTIATE_ParticleFilterOMPTracker(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterOMPTracker<T,ST>;

#endif
