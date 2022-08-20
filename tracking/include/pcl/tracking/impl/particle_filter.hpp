#ifndef PCL_TRACKING_IMPL_PARTICLE_FILTER_H_
#define PCL_TRACKING_IMPL_PARTICLE_FILTER_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/tracking/particle_filter.h>

#include <random>

namespace pcl {
namespace tracking {
template <typename PointInT, typename StateT>
bool
ParticleFilterTracker<PointInT, StateT>::initCompute()
{
  if (!Tracker<PointInT, StateT>::initCompute()) {
    PCL_ERROR("[pcl::%s::initCompute] Init failed.\n", getClassName().c_str());
    return (false);
  }

  if (transed_reference_vector_.empty()) {
    // only one time allocation
    transed_reference_vector_.resize(particle_num_);
    for (int i = 0; i < particle_num_; i++) {
      transed_reference_vector_[i] = PointCloudInPtr(new PointCloudIn());
    }
  }

  coherence_->setTargetCloud(input_);

  if (!change_detector_)
    change_detector_.reset(new pcl::octree::OctreePointCloudChangeDetector<PointInT>(
        change_detector_resolution_));

  if (!particles_ || particles_->points.empty())
    initParticles(true);
  return (true);
}

template <typename PointInT, typename StateT>
int
ParticleFilterTracker<PointInT, StateT>::sampleWithReplacement(
    const std::vector<int>& a, const std::vector<double>& q)
{
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<> rd(0.0, 1.0);

  double rU = rd(rng) * static_cast<double>(particles_->size());
  int k = static_cast<int>(rU);
  rU -= k; /* rU - [rU] */
  if (rU < q[k])
    return k;
  return a[k];
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::genAliasTable(
    std::vector<int>& a,
    std::vector<double>& q,
    const PointCloudStateConstPtr& particles)
{
  /* generate an alias table, a and q */
  std::vector<int> HL(particles->size());
  auto H = HL.begin();
  auto L = HL.end() - 1;
  const auto num = particles->size();
  for (std::size_t i = 0; i < num; i++)
    q[i] = (*particles)[i].weight * static_cast<float>(num);
  for (std::size_t i = 0; i < num; i++)
    a[i] = static_cast<int>(i);
  // setup H and L
  for (std::size_t i = 0; i < num; i++)
    if (q[i] >= 1.0)
      *H++ = static_cast<int>(i);
    else
      *L-- = static_cast<int>(i);

  while (H != HL.begin() && L != HL.end() - 1) {
    int j = *(L + 1);
    int k = *(H - 1);
    a[j] = k;
    q[k] += q[j] - 1;
    ++L;
    if (q[k] < 1.0) {
      *L-- = k;
      --H;
    }
  }
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::initParticles(bool reset)
{
  particles_.reset(new PointCloudState());
  if (reset) {
    representative_state_.zero();
    StateT offset = StateT::toState(trans_);
    representative_state_ = offset;
    representative_state_.weight = 1.0f / static_cast<float>(particle_num_);
  }

  // sampling...
  for (int i = 0; i < particle_num_; i++) {
    StateT p;
    p.zero();
    p.sample(initial_noise_mean_, initial_noise_covariance_);
    p = p + representative_state_;
    p.weight = 1.0f / static_cast<float>(particle_num_);
    particles_->points.push_back(p); // update
  }
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::normalizeWeight()
{
  // apply exponential function
  double w_min = std::numeric_limits<double>::max();
  double w_max = -std::numeric_limits<double>::max();
  for (const auto& point : *particles_) {
    double weight = point.weight;
    if (w_min > weight)
      w_min = weight;
    if (weight != 0.0 && w_max < weight)
      w_max = weight;
  }

  fit_ratio_ = w_min;
  if (w_max != w_min) {
    for (auto& point : *particles_) {
      if (point.weight != 0.0) {
        point.weight =
            static_cast<float>(normalizeParticleWeight(point.weight, w_min, w_max));
      }
    }
  }
  else {
    for (auto& point : *particles_)
      point.weight = 1.0f / static_cast<float>(particles_->size());
  }

  double sum = 0.0;
  for (const auto& point : *particles_) {
    sum += point.weight;
  }

  if (sum != 0.0) {
    for (auto& point : *particles_)
      point.weight /= static_cast<float>(sum);
  }
  else {
    for (auto& point : *particles_)
      point.weight = 1.0f / static_cast<float>(particles_->size());
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::cropInputPointCloud(
    const PointCloudInConstPtr&, PointCloudIn& output)
{
  double x_min, y_min, z_min, x_max, y_max, z_max;
  calcBoundingBox(x_min, x_max, y_min, y_max, z_min, z_max);
  pass_x_.setFilterLimits(float(x_min), float(x_max));
  pass_y_.setFilterLimits(float(y_min), float(y_max));
  pass_z_.setFilterLimits(float(z_min), float(z_max));

  // x
  PointCloudInPtr xcloud(new PointCloudIn);
  pass_x_.setInputCloud(input_);
  pass_x_.filter(*xcloud);
  // y
  PointCloudInPtr ycloud(new PointCloudIn);
  pass_y_.setInputCloud(xcloud);
  pass_y_.filter(*ycloud);
  // z
  pass_z_.setInputCloud(ycloud);
  pass_z_.filter(output);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::calcBoundingBox(double& x_min,
                                                         double& x_max,
                                                         double& y_min,
                                                         double& y_max,
                                                         double& z_min,
                                                         double& z_max)
{
  x_min = y_min = z_min = std::numeric_limits<double>::max();
  x_max = y_max = z_max = -std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < transed_reference_vector_.size(); i++) {
    PointCloudInPtr target = transed_reference_vector_[i];
    PointInT Pmin, Pmax;
    pcl::getMinMax3D(*target, Pmin, Pmax);
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

template <typename PointInT, typename StateT>
bool
ParticleFilterTracker<PointInT, StateT>::testChangeDetection(
    const PointCloudInConstPtr& input)
{
  change_detector_->setInputCloud(input);
  change_detector_->addPointsFromInputCloud();
  pcl::Indices newPointIdxVector;
  change_detector_->getPointIndicesFromNewVoxels(newPointIdxVector,
                                                 change_detector_filter_);
  change_detector_->switchBuffers();
  return !newPointIdxVector.empty();
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::weight()
{
  if (!use_normal_) {
    for (std::size_t i = 0; i < particles_->size(); i++) {
      computeTransformedPointCloudWithoutNormal((*particles_)[i],
                                                *transed_reference_vector_[i]);
    }

    PointCloudInPtr coherence_input(new PointCloudIn);
    cropInputPointCloud(input_, *coherence_input);

    coherence_->setTargetCloud(coherence_input);
    coherence_->initCompute();
    for (std::size_t i = 0; i < particles_->size(); i++) {
      IndicesPtr indices;
      coherence_->compute(
          transed_reference_vector_[i], indices, (*particles_)[i].weight);
    }
  }
  else {
    for (std::size_t i = 0; i < particles_->size(); i++) {
      IndicesPtr indices(new pcl::Indices);
      computeTransformedPointCloudWithNormal(
          (*particles_)[i], *indices, *transed_reference_vector_[i]);
    }

    PointCloudInPtr coherence_input(new PointCloudIn);
    cropInputPointCloud(input_, *coherence_input);

    coherence_->setTargetCloud(coherence_input);
    coherence_->initCompute();
    for (std::size_t i = 0; i < particles_->size(); i++) {
      IndicesPtr indices(new pcl::Indices);
      coherence_->compute(
          transed_reference_vector_[i], indices, (*particles_)[i].weight);
    }
  }

  normalizeWeight();
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloud(
    const StateT& hypothesis, pcl::Indices& indices, PointCloudIn& cloud)
{
  if (use_normal_)
    computeTransformedPointCloudWithNormal(hypothesis, indices, cloud);
  else
    computeTransformedPointCloudWithoutNormal(hypothesis, cloud);
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloudWithoutNormal(
    const StateT& hypothesis, PointCloudIn& cloud)
{
  const Eigen::Affine3f trans = toEigenMatrix(hypothesis);
  // destructively assigns to cloud
  pcl::transformPointCloud<PointInT>(*ref_, cloud, trans);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename StateT>
template <typename PointT, pcl::traits::HasNormal<PointT>>
void
ParticleFilterTracker<PointInT, StateT>::computeTransformedPointCloudWithNormal(
    const StateT& hypothesis, pcl::Indices& indices, PointCloudIn& cloud)
{
  const Eigen::Affine3f trans = toEigenMatrix(hypothesis);
  // destructively assigns to cloud
  pcl::transformPointCloudWithNormals<PointInT>(*ref_, cloud, trans);
  for (std::size_t i = 0; i < cloud.size(); i++) {
    PointInT input_point = cloud[i];

    if (!std::isfinite(input_point.x) || !std::isfinite(input_point.y) ||
        !std::isfinite(input_point.z))
      continue;
    // take occlusion into account
    Eigen::Vector4f p = input_point.getVector4fMap();
    Eigen::Vector4f n = input_point.getNormalVector4fMap();
    double theta = pcl::getAngle3D(p, n);
    if (theta > occlusion_angle_thr_)
      indices.push_back(i);
  }
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::resample()
{
  resampleWithReplacement();
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::resampleWithReplacement()
{
  std::vector<int> a(particles_->size());
  std::vector<double> q(particles_->size());
  genAliasTable(a, q, particles_);

  const std::vector<double> zero_mean(StateT::stateDimension(), 0.0);
  // memoize the original list of particles
  PointCloudStatePtr origparticles = particles_;
  particles_->points.clear();
  // the first particle, it is a just copy of the maximum result
  StateT p = representative_state_;
  particles_->points.push_back(p);

  // with motion
  int motion_num =
      static_cast<int>(particles_->size()) * static_cast<int>(motion_ratio_);
  for (int i = 1; i < motion_num; i++) {
    int target_particle_index = sampleWithReplacement(a, q);
    StateT p = (*origparticles)[target_particle_index];
    // add noise using gaussian
    p.sample(zero_mean, step_noise_covariance_);
    p = p + motion_;
    particles_->points.push_back(p);
  }

  // no motion
  for (int i = motion_num; i < particle_num_; i++) {
    int target_particle_index = sampleWithReplacement(a, q);
    StateT p = (*origparticles)[target_particle_index];
    // add noise using gaussian
    p.sample(zero_mean, step_noise_covariance_);
    particles_->points.push_back(p);
  }
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::update()
{

  StateT orig_representative = representative_state_;
  representative_state_.zero();
  representative_state_.weight = 0.0;
  for (const auto& p : *particles_) {
    representative_state_ = representative_state_ + p * p.weight;
  }
  representative_state_.weight = 1.0f / static_cast<float>(particles_->size());
  motion_ = representative_state_ - orig_representative;
}

template <typename PointInT, typename StateT>
void
ParticleFilterTracker<PointInT, StateT>::computeTracking()
{

  for (int i = 0; i < iteration_num_; i++) {
    if (changed_) {
      resample();
    }

    weight(); // likelihood is called in it

    if (changed_) {
      update();
    }
  }

  // if ( getResult ().weight < resample_likelihood_thr_ )
  // {
  //   PCL_WARN ("too small likelihood, re-initializing...\n");
  //   initParticles (false);
  // }
}
} // namespace tracking
} // namespace pcl

#define PCL_INSTANTIATE_ParticleFilterTracker(T, ST)                                   \
  template class PCL_EXPORTS pcl::tracking::ParticleFilterTracker<T, ST>;

#endif
