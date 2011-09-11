#ifndef PCL_TRACKING_IMPL_PARTICLE_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_FILTER_H_

#include <boost/random.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transform.h>
#include <pcl/registration/transforms.h>

template <typename PointInT, typename StateT> bool
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::initCompute ()
{
  if (!Tracker<PointInT, StateT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }
  
  if (!particles_ || particles_->points.empty ())
    initParticles ();
  return (true);
}

template <typename PointInT, typename StateT> double
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::calcLikelihood (StateT hypothesis)
{
  double val = 1.0;
  // build a pointcloud for hypothesis
  Eigen::Affine3f trans = toEigenMatrix (hypothesis);
  PointCloudIn transed_reference;
  pcl::transformPointCloudWithNormals<PointInT> (*ref_, transed_reference, trans);

  // search the nearest pairs
  std::vector<int> k_indices(1);
  std::vector<float> k_distances(1);
  int num_points = 0;
  for ( size_t i = 0; i < transed_reference.points.size (); i++ )
  {
    PointInT input_point = transed_reference.points[i];

    // take occlusion into account
    Eigen::Vector4f p = input_point.getVector4fMap ();
    Eigen::Vector4f n(input_point.normal[0], input_point.normal[1], input_point.normal[2], 0.0f);
    // TODO: check NAN
    if ( pcl::getAngle3D(p, n) > occlusion_angle_thr_ )
    {
      tree_->nearestKSearch (input_point, 1, k_indices, k_distances); // input_ is set by Tracker::initCompute
      PointInT target_point = input_->points[k_indices[0]];
      for (size_t i = 0; i < coherences_.size (); i++)
      {
        CoherencePtr coherence = coherences_[i];
        val *= coherence->compute (input_point, target_point);
      }
      ++num_points;
    }
  }

  // take min_indices_ into account
  // TODO: normalization of likelihood is required?
  double likelihood;
  if ( num_points >= min_indices_ )
    likelihood = val;
  //likelihood = exp((((ret * ret ) / num) / (- 2.0 * sigma2)));
  else
    likelihood = 0.0;
  return likelihood;
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
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::genAliasTable (std::vector<int> &a, std::vector<double> &q)
{
  /* generate an alias table, a and q */
  std::vector<int> HL (particle_num_);
  std::vector<int>::iterator H = HL.begin ();
  std::vector<int>::iterator L = HL.end () - 1;
  for ( int i = 0; i < particle_num_; i++ )
    q[i] = particles_->points[i].weight * particle_num_;
  //q[i] = particles_[i].getWeight() * particle_num_;
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

template <typename PointInT, typename StateT> double
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::normalizeAngle (const double val)
{
  if ( val > M_PI * 2.0 )
  {
    int offset = val / ( M_PI * 2.0 );
    return val - M_PI * 2.0 * offset;
  }
  else if ( val < 0.0 )
  {
    int offset = abs(val) / ( M_PI * 2.0 );
    return val + M_PI * 2.0 * offset;
  }
  else
  {
    return val;
  }
}

// TODO: should be a method of StateT
template <typename PointInT, typename StateT> Eigen::Affine3f
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::toEigenMatrix (StateT particle)
{
  double xval = particle.x + offset_.x;
  double yval = particle.y + offset_.y;
  double zval = particle.z + offset_.z;
  // i need to normalize euler angles?
  double rollval = normalizeAngle(particle.roll + offset_.roll);
  double pitchval = normalizeAngle(particle.pitch + offset_.pitch);
  double yawval = normalizeAngle(particle.yaw + offset_.yaw);
  
  return trans_ * getTransformation(xval, yval, zval, rollval, pitchval, yawval);
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::initParticles ()
{
  std::cout << "initParticles" << std::endl;
  //particles_ = PointCloudState ();
  particles_.reset( new PointCloudState ());
  
  representative_state_.zero ();
  representative_state_.weight = 0.0;
  
  // sampling...
  for ( int i = 0; i < particle_num_; i++ )
  {
    StateT p;
    p.weight = 1.0 / particle_num_;
    p.zero ();
    p.sample (initial_noise_mean_, initial_noise_covariance_);
    particles_->points.push_back (p); // update
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::normalizeWeight ()
{
  double sum = 0.0;
  for ( int i = 0; i < particle_num_; i++ )
    sum += particles_->points[i].weight;
  for ( int i = 0; i < particle_num_; i++ )
    particles_->points[i].weight =  particles_->points[i].weight / sum;
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::weight ()
{
  for ( int i = 0; i < particle_num_; i++ )
  {
    double likelihood = calcLikelihood (particles_->points[i]);
    particles_->points[i].weight = likelihood;
  }
  normalizeWeight ();
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::resample ()
{
  std::vector<int> a (particle_num_);
  std::vector<double> q (particle_num_);
  genAliasTable (a, q);
  
  const std::vector<double> zero_mean (StateT::stateDimension (), 0.0);
  // memoize the original list of particles
  //std::vector<Particle> origparticles = particles_;
  PointCloudStatePtr  origparticles = particles_;
  particles_->points.clear ();
  // the first particle, it is a just copy of the maximum result
  pcl::tracking::ParticleXYZRPY p = representative_state_;
  //particles_.push_back (p);
  particles_->points.push_back (p);
  for ( int i = 1; i < particle_num_; i++ )
  {
    int target_particle_index = sampleWithReplacement (a, q);
    //Particle p = origparticles.points[target_particle_index];
    pcl::tracking::ParticleXYZRPY p = origparticles->points[target_particle_index];
    // add noise using gaussian
    p.sample (zero_mean, step_noise_covariance_);
    //particles_.push_back (p);
    particles_->points.push_back (p);
  }
}

template <typename PointInT, typename StateT> void
pcl::tracking::ParticleFilterTracker<PointInT, StateT>::update ()
{
  representative_state_.zero ();
  double maxweight = 0.0;
  for ( int i = 0; i < particle_num_; i++)
  {
    StateT p = particles_->points[i];
    if (maxweight < p.weight )
      maxweight = p.weight;
    representative_state_ = representative_state_ + p * p.weight;
  }
  representative_state_.weight = maxweight;
  
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
    initParticles ();
  }
}

#endif 
