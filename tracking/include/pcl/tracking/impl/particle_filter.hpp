#ifndef PCL_TRACKING_IMPL_PARTICLE_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_FILTER_H_

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/boost.h>
#include <pcl/tracking/particle_filter.h>

template <typename PointInT, typename StateT> bool
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::initCompute ()
{
  if (!Tracker<PointInT, StateT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  if (transed_reference_vector_.empty ())
  {
    // only one time allocation
    transed_reference_vector_.resize (particle_num_);
    for (int i = 0; i < particle_num_; i++)
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

template <typename PointInT, typename StateT> int
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::sampleWithReplacement
(const std::vector<int>& a, const std::vector<double>& q)
{
  using namespace boost;
  static mt19937 gen (static_cast<unsigned int>(time (0)));
  uniform_real<> dst (0.0, 1.0);
  variate_generator<mt19937&, uniform_real<> > rand (gen, dst);
  double rU = rand () * static_cast<double> (particles_->points.size ());
  int k = static_cast<int> (rU);
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
  std::vector<int> HL (particles->points.size ());
  std::vector<int>::iterator H = HL.begin ();
  std::vector<int>::iterator L = HL.end () - 1;
  size_t num = particles->points.size ();
  for ( size_t i = 0; i < num; i++ )
    q[i] = particles->points[i].weight * static_cast<float> (num);
  for ( size_t i = 0; i < num; i++ )
    a[i] = static_cast<int> (i);
  // setup H and L
  for ( size_t i = 0; i < num; i++ )
    if ( q[i] >= 1.0 )
      *H++ = static_cast<int> (i);
    else
      *L-- = static_cast<int> (i);
            
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
  particles_.reset (new PointCloudState ());
  std::vector<double> initial_noise_mean;
  if (reset)
  {
    representative_state_.zero ();
    StateT offset = StateT::toState (trans_);
    representative_state_ = offset;
    representative_state_.weight = 1.0f / static_cast<float> (particle_num_);
  }

  // sampling...
  for ( int i = 0; i < particle_num_; i++ )
  {
    StateT p;
    p.zero ();
    p.sample (initial_noise_mean_, initial_noise_covariance_);
    p = p + representative_state_;
    p.weight = 1.0f / static_cast<float> (particle_num_);
    particles_->points.push_back (p); // update
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::normalizeWeight ()
{
    // apply exponential function
    double w_min = std::numeric_limits<double>::max ();
    double w_max = - std::numeric_limits<double>::max ();
    for ( size_t i = 0; i < particles_->points.size (); i++ )
    {
      double weight = particles_->points[i].weight;
      if (w_min > weight)
        w_min = weight;
      if (weight != 0.0 && w_max < weight)
        w_max = weight;
    }
    
    fit_ratio_ = w_min;
    if (w_max != w_min)
    {
      for ( size_t i = 0; i < particles_->points.size (); i++ )
      {
        if (particles_->points[i].weight != 0.0)
        {
          particles_->points[i].weight = static_cast<float> (normalizeParticleWeight (particles_->points[i].weight, w_min, w_max));
        }
      }
    }
    else
    {
      for ( size_t i = 0; i < particles_->points.size (); i++ )
        particles_->points[i].weight = 1.0f / static_cast<float> (particles_->points.size ());
    }
    
    double sum = 0.0;
    for ( size_t i = 0; i < particles_->points.size (); i++ )
    {
        sum += particles_->points[i].weight;
    }
    
    if (sum != 0.0)
    {
      for ( size_t i = 0; i < particles_->points.size (); i++ )
        particles_->points[i].weight =  particles_->points[i].weight / static_cast<float> (sum);
    }
    else
    {
      for ( size_t i = 0; i < particles_->points.size (); i++ )
        particles_->points[i].weight = 1.0f / static_cast<float> (particles_->points.size ());
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::cropInputPointCloud (
    const PointCloudInConstPtr &, PointCloudIn &output)
{
  double x_min, y_min, z_min, x_max, y_max, z_max;
  calcBoundingBox (x_min, x_max, y_min, y_max, z_min, z_max);
  pass_x_.setFilterLimits (float (x_min), float (x_max));
  pass_y_.setFilterLimits (float (y_min), float (y_max));
  pass_z_.setFilterLimits (float (z_min), float (z_max));
  
  // x
  PointCloudInPtr xcloud (new PointCloudIn);
  pass_x_.setInputCloud (input_);
  pass_x_.filter (*xcloud);
  // y
  PointCloudInPtr ycloud (new PointCloudIn);
  pass_y_.setInputCloud (xcloud);
  pass_y_.filter (*ycloud);
  // z
  pass_z_.setInputCloud (ycloud);
  pass_z_.filter (output);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::calcBoundingBox (
    double &x_min, double &x_max, double &y_min, double &y_max, double &z_min, double &z_max)
{
  x_min = y_min = z_min = std::numeric_limits<double>::max ();
  x_max = y_max = z_max = - std::numeric_limits<double>::max ();
  
  for (size_t i = 0; i < transed_reference_vector_.size (); i++)
  {
    PointCloudInPtr target = transed_reference_vector_[i];
    PointInT Pmin, Pmax;
    pcl::getMinMax3D (*target, Pmin, Pmax);
    if (x_min > Pmin.x)
      x_min = Pmin.x;
    if (x_max < Pmax.x)
      x_max = Pmax.x;
    if (y_min > Pmin.y)
      y_min = Pmin.y;
    if (y_max < Pmax.y)
      y_max = Pmax.y;
    if (z_min > Pmin.z)
      z_min = Pmin.z;
    if (z_max < Pmax.z)
      z_max = Pmax.z;
  }
}

template <typename PointInT, typename StateT> bool
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::testChangeDetection
(const PointCloudInConstPtr &input)
{
  change_detector_->setInputCloud (input);
  change_detector_->addPointsFromInputCloud ();
  std::vector<int> newPointIdxVector;
  change_detector_->getPointIndicesFromNewVoxels (newPointIdxVector, change_detector_filter_);
  change_detector_->switchBuffers ();
  return newPointIdxVector.size () > 0;
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::weight ()
{
  if (!use_normal_)
  {
    for (size_t i = 0; i < particles_->points.size (); i++)
    {
      computeTransformedPointCloudWithoutNormal (particles_->points[i], *transed_reference_vector_[i]);
    }
    
    PointCloudInPtr coherence_input (new PointCloudIn);
    cropInputPointCloud (input_, *coherence_input);
    
    coherence_->setTargetCloud (coherence_input);
    coherence_->initCompute ();
    for (size_t i = 0; i < particles_->points.size (); i++)
    {
      IndicesPtr indices;
      coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
    }
  }
  else
  {
    for (size_t i = 0; i < particles_->points.size (); i++)
    {
      IndicesPtr indices (new std::vector<int>);
      computeTransformedPointCloudWithNormal (particles_->points[i], *indices, *transed_reference_vector_[i]);
    }
    
    PointCloudInPtr coherence_input (new PointCloudIn);
    cropInputPointCloud (input_, *coherence_input);
    
    coherence_->setTargetCloud (coherence_input);
    coherence_->initCompute ();
    for (size_t i = 0; i < particles_->points.size (); i++)
    {
      IndicesPtr indices (new std::vector<int>);
      coherence_->compute (transed_reference_vector_[i], indices, particles_->points[i].weight);
    }
  }
  
  normalizeWeight ();
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloud
(const StateT& hypothesis, std::vector<int>& indices, PointCloudIn &cloud)
{
  if (use_normal_)
    computeTransformedPointCloudWithNormal (hypothesis, indices, cloud);
  else
    computeTransformedPointCloudWithoutNormal (hypothesis, cloud);
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloudWithoutNormal
(const StateT& hypothesis, PointCloudIn &cloud)
{
  const Eigen::Affine3f trans = toEigenMatrix (hypothesis);
  // destructively assigns to cloud
  pcl::transformPointCloud<PointInT> (*ref_, cloud, trans);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloudWithNormal (
#ifdef PCL_TRACKING_NORMAL_SUPPORTED
    const StateT& hypothesis, std::vector<int>& indices, PointCloudIn &cloud)
#else
    const StateT&, std::vector<int>&, PointCloudIn &)
#endif
{
#ifdef PCL_TRACKING_NORMAL_SUPPORTED
  const Eigen::Affine3f trans = toEigenMatrix (hypothesis);
  // destructively assigns to cloud
  pcl::transformPointCloudWithNormals<PointInT> (*ref_, cloud, trans);
  for ( size_t i = 0; i < cloud.points.size (); i++ )
  {
    PointInT input_point = cloud.points[i];

    if (!pcl_isfinite(input_point.x) || !pcl_isfinite(input_point.y) || !pcl_isfinite(input_point.z))
      continue;
    // take occlusion into account
    Eigen::Vector4f p = input_point.getVector4fMap ();
    Eigen::Vector4f n = input_point.getNormalVector4fMap ();
    double theta = pcl::getAngle3D (p, n);
    if ( theta > occlusion_angle_thr_ )
      indices.push_back (i);
  }
#else
  PCL_WARN ("[pcl::%s::computeTransformedPointCloudWithoutNormal] use_normal_ == true is not supported in this Point Type.",
            getClassName ().c_str ());
#endif
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::resample ()
{
  resampleWithReplacement ();
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::resampleWithReplacement ()
{
  std::vector<int> a (particles_->points.size ());
  std::vector<double> q (particles_->points.size ());
  genAliasTable (a, q, particles_);
  
  const std::vector<double> zero_mean (StateT::stateDimension (), 0.0);
  // memoize the original list of particles
  PointCloudStatePtr origparticles = particles_;
  particles_->points.clear ();
  // the first particle, it is a just copy of the maximum result
  StateT p = representative_state_;
  particles_->points.push_back (p);
  
  // with motion
  int motion_num = static_cast<int> (particles_->points.size ()) * static_cast<int> (motion_ratio_);
  for ( int i = 1; i < motion_num; i++ )
  {
    int target_particle_index = sampleWithReplacement (a, q);
    StateT p = origparticles->points[target_particle_index];
    // add noise using gaussian
    p.sample (zero_mean, step_noise_covariance_);
    p = p + motion_;
    particles_->points.push_back (p);
  }
  
  // no motion
  for ( int i = motion_num; i < particle_num_; i++ )
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
  
  StateT orig_representative = representative_state_;
  representative_state_.zero ();
  representative_state_.weight = 0.0;
  for ( size_t i = 0; i < particles_->points.size (); i++)
  {
    StateT p = particles_->points[i];
    representative_state_ = representative_state_ + p * p.weight;
  }
  representative_state_.weight = 1.0f / static_cast<float> (particles_->points.size ());
  motion_ = representative_state_ - orig_representative;
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::computeTracking ()
{
  
  for (int i = 0; i < iteration_num_; i++)
  {
    if (changed_)
    {
      resample ();
    }
    
    weight (); // likelihood is called in it
    
    if (changed_)
    {
      update ();
    }
  }
  
  // if ( getResult ().weight < resample_likelihood_thr_ )
  // {
  //   PCL_WARN ("too small likelihood, re-initializing...\n");
  //   initParticles (false);
  // }
}

#define PCL_INSTANTIATE_ParticleFilterTracker(T,ST) template class PCL_EXPORTS pcl::tracking::ParticleFilterTracker<T,ST>;

#endif 
