#ifndef PCL_TRACKING_IMPL_PARTICLE_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_FILTER_H_

#include <boost/random.hpp>

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

template <typename PointInT, typename StateT> bool
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::initCompute ()
{
  if (!Tracker<PointInT, StateT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }
  
  coherence_->setTargetCloud (input_);
  
  if (!particles_ || particles_->points.empty ())
    initParticles (true);
  return (true);
}

/////////////////////////////////////////////////////
// deprecated
template <typename PointInT, typename StateT> double
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::calcLikelihood (const StateT &hypothesis)
{
  IndicesPtr indices = IndicesPtr (new std::vector<int> ());
  PointCloudInPtr transed_reference = PointCloudInPtr (new PointCloudIn ());
  computeTransformedPointCloud (hypothesis, *indices, *transed_reference);
  //return coherence_->compute (transed_reference, indices, hypothesis.weight);
  return 0.0;
}


template <typename PointInT, typename StateT> int
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::sampleWithReplacement
(const std::vector<int>& a, const std::vector<double>& q)
{
  using namespace boost;
  static mt19937 gen (static_cast<unsigned long>(time (0)));
  uniform_real<> dst (0.0, 1.0);
  variate_generator<mt19937&, uniform_real<> > rand (gen, dst);
  double rU = rand () * particle_num_;
  int k = (int)rU;
  rU -= k;    /* rU - [rU] */
  if ( rU < q[k] )
    return k;
  else
    return a[k];
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::genAliasTable (std::vector<int> &a, std::vector<double> &q,
                                                                       const PointCloudStateConstPtr &particles)
{
  /* generate an alias table, a and q */
  std::vector<int> HL (particle_num_);
  std::vector<int>::iterator H = HL.begin ();
  std::vector<int>::iterator L = HL.end () - 1;
  for ( int i = 0; i < particle_num_; i++ )
    q[i] = particles->points[i].weight * particle_num_;
  for ( int i = 0; i < particle_num_; i++ )
    a[i] = i;
  // setup H and L
  for ( int i = 0; i < particle_num_; i++ )
    if ( q[i] >= 1.0 )
      *H++ = i;
    else
      *L-- = i;
            
  while ( H != HL.begin() && L != HL.end() - 1 )
  {
    int j = *(L + 1);
    int k = *(H - 1);
    a[j] = k;
    q[k] += q[j] - 1;
    L++;
    if ( q[k] < 1.0 )
    {
      *L-- = k;
      --H;
    }
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::initParticles (bool reset)
{
  PCL_INFO ("[pcl::%s::initParticles] initializing...\n", getClassName ().c_str ());
  particles_.reset (new PointCloudState ());
  std::vector<double> initial_noise_mean;
  if (reset)
  {
    representative_state_.zero ();
    StateT offset = StateT::toState (trans_);
    representative_state_ = offset;
    representative_state_.weight = 1.0 / particle_num_;
  }

  // sampling...
  for ( int i = 0; i < particle_num_; i++ )
  {
    StateT p;
    p.zero ();
    p.sample (initial_noise_mean_, initial_noise_covariance_);
    p = p + representative_state_;
    p.weight = 1.0 / particle_num_;
    particles_->points.push_back (p); // update
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::normalizeWeight ()
{
    // apply exponential function
    double w_min = std::numeric_limits<double>::max ();
    double w_max = std::numeric_limits<double>::min ();
    
    for ( int i = 0; i < particle_num_; i++ )
    {
        double weight = particles_->points[i].weight;
        if (w_min > weight)
            w_min = weight;
        if (w_max < weight)
            w_max = weight;
    }
    
    double denom = w_max - w_min;
    for ( int i = 0; i < particle_num_; i++ )
        particles_->points[i].weight
            = exp (1.0 - alpha_ * (particles_->points[i].weight - w_min) / denom);
    
    double sum = 0.0;
    for ( int i = 0; i < particle_num_; i++ )
        sum += particles_->points[i].weight;
    std::cout << "sum: " << sum << std::endl;
    if (sum != 0.0)
    {
        for ( int i = 0; i < particle_num_; i++ )
            particles_->points[i].weight =  particles_->points[i].weight / sum;
    }
    else
    {
        for ( int i = 0; i < particle_num_; i++ )
            particles_->points[i].weight = 1.0 / particle_num_;
    }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::weight ()
{
  coherence_->initCompute ();
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

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloud
(const StateT& hypothesis, std::vector<int>& indices, PointCloudIn &cloud)
{
  const Eigen::Affine3f trans = toEigenMatrix (hypothesis);
  // destructively assigns to cloud
  pcl::transformPointCloudWithNormals<PointInT> (*ref_, cloud, trans);
  // search the nearest pairs
  std::vector<int> k_indices(1);
  std::vector<float> k_distances(1);
  for ( size_t i = 0; i < cloud.points.size (); i++ )
  {
    PointInT input_point = cloud.points[i];

    if (!pcl_isfinite(input_point.x) || !pcl_isfinite(input_point.y) || !pcl_isfinite(input_point.z))
      continue;
    // take occlusion into account
    Eigen::Vector4f p = input_point.getVector4fMap ();
    Eigen::Vector4f n = input_point.getNormalVector4fMap ();
    double theta = pcl::getAngle3D (p, n);
    // TODO: check NAN?
    if ( theta > occlusion_angle_thr_ )
      indices.push_back (i);
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::resample ()
{
  resampleWithReplacement ();
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::resampleWithReplacement ()
{
  std::vector<int> a (particle_num_);
  std::vector<double> q (particle_num_);
  genAliasTable (a, q, particles_);
  
  const std::vector<double> zero_mean (StateT::stateDimension (), 0.0);
  // memoize the original list of particles
  PointCloudStatePtr origparticles = particles_;
  particles_->points.clear ();
  // the first particle, it is a just copy of the maximum result
  StateT p = representative_state_;
  particles_->points.push_back (p);
  for ( int i = 1; i < particle_num_; i++ )
  {
    int target_particle_index = sampleWithReplacement (a, q);
    StateT p = origparticles->points[target_particle_index];
    // add noise using gaussian
    p.sample (zero_mean, step_noise_covariance_);
    particles_->points.push_back (p);
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::update ()
{
  representative_state_.zero ();
  representative_state_.weight = 0.0;
  for ( int i = 0; i < particle_num_; i++)
  {
    StateT p = particles_->points[i];
    representative_state_ = representative_state_ + p * p.weight;
  }
  representative_state_.weight = 1.0 / particle_num_;
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::computeTracking ()
{
  
  for (int i = 0; i < iteration_num_; i++)
  {
    resample ();
    weight (); // likelihood is called in it
    update ();
  }
  
  if ( getResult ().weight < resample_likelihood_thr_ )
  {
    PCL_WARN ("too small likelihood, re-initializing...\n");
    initParticles (false);
  }
}

#define PCL_INSTANTIATE_ParticleFilterTracker(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterTracker<T,ST>;

#endif 
