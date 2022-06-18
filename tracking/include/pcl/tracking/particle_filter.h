#pragma once

#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/tracker.h>
#include <pcl/tracking/tracking.h>
#include <pcl/memory.h>
#include <pcl/point_types.h>

namespace pcl {
namespace tracking {
/** \brief @b ParticleFilterTracker tracks the PointCloud which is given by
 * setReferenceCloud within the measured PointCloud using particle filter method.
 * \author Ryohei Ueda
 * \ingroup tracking
 */
template <typename PointInT, typename StateT>
class ParticleFilterTracker : public Tracker<PointInT, StateT> {
protected:
  using Tracker<PointInT, StateT>::deinitCompute;

public:
  using Tracker<PointInT, StateT>::tracker_name_;
  using Tracker<PointInT, StateT>::search_;
  using Tracker<PointInT, StateT>::input_;
  using Tracker<PointInT, StateT>::indices_;
  using Tracker<PointInT, StateT>::getClassName;

  using Ptr = shared_ptr<ParticleFilterTracker<PointInT, StateT>>;
  using ConstPtr = shared_ptr<const ParticleFilterTracker<PointInT, StateT>>;

  using BaseClass = Tracker<PointInT, StateT>;

  using PointCloudIn = typename Tracker<PointInT, StateT>::PointCloudIn;
  using PointCloudInPtr = typename PointCloudIn::Ptr;
  using PointCloudInConstPtr = typename PointCloudIn::ConstPtr;

  using PointCloudState = typename Tracker<PointInT, StateT>::PointCloudState;
  using PointCloudStatePtr = typename PointCloudState::Ptr;
  using PointCloudStateConstPtr = typename PointCloudState::ConstPtr;

  using Coherence = PointCoherence<PointInT>;
  using CoherencePtr = typename Coherence::Ptr;
  using CoherenceConstPtr = typename Coherence::ConstPtr;

  using CloudCoherence = PointCloudCoherence<PointInT>;
  using CloudCoherencePtr = typename CloudCoherence::Ptr;
  using CloudCoherenceConstPtr = typename CloudCoherence::ConstPtr;

  /** \brief Empty constructor. */
  ParticleFilterTracker()
  : iteration_num_(1)
  , particle_num_()
  , min_indices_(1)
  , ref_()
  , particles_()
  , coherence_()
  , resample_likelihood_thr_(0.0)
  , occlusion_angle_thr_(M_PI / 2.0)
  , alpha_(15.0)
  , representative_state_()
  , use_normal_(false)
  , motion_()
  , motion_ratio_(0.25)
  , pass_x_()
  , pass_y_()
  , pass_z_()
  , transed_reference_vector_()
  , change_detector_()
  , changed_(false)
  , change_counter_(0)
  , change_detector_filter_(10)
  , change_detector_interval_(10)
  , change_detector_resolution_(0.01)
  , use_change_detector_(false)
  {
    tracker_name_ = "ParticleFilterTracker";
    pass_x_.setFilterFieldName("x");
    pass_y_.setFilterFieldName("y");
    pass_z_.setFilterFieldName("z");
    pass_x_.setKeepOrganized(false);
    pass_y_.setKeepOrganized(false);
    pass_z_.setKeepOrganized(false);
  }

  /** \brief Set the number of iteration.
   * \param[in] iteration_num the number of iteration.
   */
  inline void
  setIterationNum(const int iteration_num)
  {
    iteration_num_ = iteration_num;
  }

  /** \brief Get the number of iteration. */
  inline int
  getIterationNum() const
  {
    return iteration_num_;
  }

  /** \brief Set the number of the particles.
   * \param[in] particle_num the number of the particles.
   */
  inline void
  setParticleNum(const int particle_num)
  {
    particle_num_ = particle_num;
  }

  /** \brief Get the number of the particles. */
  inline int
  getParticleNum() const
  {
    return particle_num_;
  }

  /** \brief Set a pointer to a reference dataset to be tracked.
   * \param[in] ref a pointer to a PointCloud message
   */
  inline void
  setReferenceCloud(const PointCloudInConstPtr& ref)
  {
    ref_ = ref;
  }

  /** \brief Get a pointer to a reference dataset to be tracked. */
  inline PointCloudInConstPtr const
  getReferenceCloud()
  {
    return ref_;
  }

  /** \brief Set the PointCloudCoherence as likelihood.
   * \param[in] coherence a pointer to PointCloudCoherence.
   */
  inline void
  setCloudCoherence(const CloudCoherencePtr& coherence)
  {
    coherence_ = coherence;
  }

  /** \brief Get the PointCloudCoherence to compute likelihood. */
  inline CloudCoherencePtr
  getCloudCoherence() const
  {
    return coherence_;
  }

  /** \brief Set the covariance of step noise.
   * \param[in] step_noise_covariance the diagonal elements of covariance matrix
   * of step noise.
   */
  inline void
  setStepNoiseCovariance(const std::vector<double>& step_noise_covariance)
  {
    step_noise_covariance_ = step_noise_covariance;
  }

  /** \brief Set the covariance of the initial noise. It will be used when
   * initializing the particles.
   * \param[in] initial_noise_covariance the diagonal elements of covariance matrix of
   * initial noise.
   */
  inline void
  setInitialNoiseCovariance(const std::vector<double>& initial_noise_covariance)
  {
    initial_noise_covariance_ = initial_noise_covariance;
  }

  /** \brief Set the mean of the initial noise. It will be used when
   * initializing the particles.
   * \param[in] initial_noise_mean the mean values of initial noise.
   */
  inline void
  setInitialNoiseMean(const std::vector<double>& initial_noise_mean)
  {
    initial_noise_mean_ = initial_noise_mean;
  }

  /** \brief Set the threshold to re-initialize the particles.
   * \param[in] resample_likelihood_thr threshold to re-initialize.
   */
  inline void
  setResampleLikelihoodThr(const double resample_likelihood_thr)
  {
    resample_likelihood_thr_ = resample_likelihood_thr;
  }

  /** \brief Set the threshold of angle to be considered occlusion (default:
   * pi/2). ParticleFilterTracker does not take the occluded points into account
   * according to the angle between the normal and the position.
   * \param[in] occlusion_angle_thr threshold of angle to be considered occlusion.
   */
  inline void
  setOcclusionAngleThe(const double occlusion_angle_thr)
  {
    occlusion_angle_thr_ = occlusion_angle_thr;
  }

  /** \brief Set the minimum number of indices (default: 1).
   * ParticleFilterTracker does not take into account the hypothesis
   * whose the number of points is smaller than the minimum indices.
   * \param[in] min_indices the minimum number of indices.
   */
  inline void
  setMinIndices(const int min_indices)
  {
    min_indices_ = min_indices;
  }

  /** \brief Set the transformation from the world coordinates to the frame of
   * the particles.
   * \param[in] trans Affine transformation from the worldcoordinates to the frame of
   * the particles.
   */
  inline void
  setTrans(const Eigen::Affine3f& trans)
  {
    trans_ = trans;
  }

  /** \brief Get the transformation from the world coordinates to the frame of
   * the particles. */
  inline Eigen::Affine3f
  getTrans() const
  {
    return trans_;
  }

  /** \brief Get an instance of the result of tracking.
   * This function returns the particle that represents the transform between
   * the reference point cloud at the beginning and the best guess about its
   * location in the most recent frame.
   */
  inline StateT
  getResult() const override
  {
    return representative_state_;
  }

  /** \brief Convert a state to affine transformation from the world coordinates
   * frame.
   * \param[in] particle an instance of StateT.
   */
  Eigen::Affine3f
  toEigenMatrix(const StateT& particle)
  {
    return particle.toEigenMatrix();
  }

  /** \brief Get a pointer to a pointcloud of the particles. */
  inline PointCloudStatePtr
  getParticles() const
  {
    return particles_;
  }

  /** \brief Normalize the weight of a particle using \f$ std::exp(1- alpha ( w
   * - w_{min}) / (w_max - w_min)) \f$
   * \note This method is described in [P.Azad
   * et. al, ICRA11].
   * \param[in] w the weight to be normalized
   * \param[in] w_min the minimum weight of the particles
   * \param[in] w_max the maximum weight of the particles
   */
  inline double
  normalizeParticleWeight(double w, double w_min, double w_max)
  {
    return std::exp(1.0 - alpha_ * (w - w_min) / (w_max - w_min));
  }

  /** \brief Set the value of alpha.
   *  \param[in] alpha the value of alpha
   */
  inline void
  setAlpha(double alpha)
  {
    alpha_ = alpha;
  }

  /** \brief Get the value of alpha. */
  inline double
  getAlpha()
  {
    return alpha_;
  }

  /** \brief Set the value of use_normal_.
   * \param[in] use_normal the value of use_normal_.
   */
  inline void
  setUseNormal(bool use_normal)
  {
    if (traits::has_normal_v<PointInT> || !use_normal) {
      use_normal_ = use_normal;
      return;
    }
    PCL_WARN("[pcl::%s::setUseNormal] "
             "use_normal_ == true is not supported in this Point Type.\n",
             getClassName().c_str());
    use_normal_ = false;
  }

  /** \brief Get the value of use_normal_. */
  inline bool
  getUseNormal()
  {
    return use_normal_;
  }

  /** \brief Set the value of use_change_detector_.
   * \param[in] use_change_detector the value of use_change_detector_.
   */
  inline void
  setUseChangeDetector(bool use_change_detector)
  {
    use_change_detector_ = use_change_detector;
  }

  /** \brief Get the value of use_change_detector_. */
  inline bool
  getUseChangeDetector()
  {
    return use_change_detector_;
  }

  /** \brief Set the motion ratio
   * \param[in] motion_ratio the ratio of hypothesis to use motion model.
   */
  inline void
  setMotionRatio(double motion_ratio)
  {
    motion_ratio_ = motion_ratio;
  }

  /** \brief Get the motion ratio. */
  inline double
  getMotionRatio()
  {
    return motion_ratio_;
  }

  /** \brief Set the number of interval frames to run change detection.
   * \param[in] change_detector_interval the number of interval frames.
   */
  inline void
  setIntervalOfChangeDetection(unsigned int change_detector_interval)
  {
    change_detector_interval_ = change_detector_interval;
  }

  /** \brief Get the number of interval frames to run change detection. */
  inline unsigned int
  getIntervalOfChangeDetection()
  {
    return change_detector_interval_;
  }

  /** \brief Set the minimum amount of points required within leaf node to
   * become serialized in change detection
   * \param[in] change_detector_filter the minimum amount of points required within leaf
   * node
   */
  inline void
  setMinPointsOfChangeDetection(unsigned int change_detector_filter)
  {
    change_detector_filter_ = change_detector_filter;
  }

  /** \brief Set the resolution of change detection.
   * \param[in] resolution resolution of change detection octree
   */
  inline void
  setResolutionOfChangeDetection(double resolution)
  {
    change_detector_resolution_ = resolution;
  }

  /** \brief Get the resolution of change detection. */
  inline double
  getResolutionOfChangeDetection()
  {
    return change_detector_resolution_;
  }

  /** \brief Get the minimum amount of points required within leaf node to
   * become serialized in change detection. */
  inline unsigned int
  getMinPointsOfChangeDetection()
  {
    return change_detector_filter_;
  }

  /** \brief Get the adjustment ratio. */
  inline double
  getFitRatio() const
  {
    return fit_ratio_;
  }

  /** \brief Reset the particles to restart tracking*/
  virtual inline void
  resetTracking()
  {
    if (particles_)
      particles_->points.clear();
  }

protected:
  /** \brief Compute the parameters for the bounding box of hypothesis
   * pointclouds.
   * \param[out] x_min the minimum value of x axis.
   * \param[out] x_max the maximum value of x axis.
   * \param[out] y_min the minimum value of y axis.
   * \param[out] y_max the maximum value of y axis.
   * \param[out] z_min the minimum value of z axis.
   * \param[out] z_max the maximum value of z axis.
   */
  void
  calcBoundingBox(double& x_min,
                  double& x_max,
                  double& y_min,
                  double& y_max,
                  double& z_min,
                  double& z_max);

  /** \brief Crop the pointcloud by the bounding box calculated from hypothesis
   * and the reference pointcloud.
   * \param[in] cloud a pointer to pointcloud to be cropped.
   * \param[out] output a pointer to be assigned the cropped pointcloud.
   */
  void
  cropInputPointCloud(const PointCloudInConstPtr& cloud, PointCloudIn& output);

  /** \brief Compute a reference pointcloud transformed to the pose that hypothesis
   represents.
   * \param[in] hypothesis a particle which represents a hypothesis.
   * \param[in] indices the indices which should be taken into account.
   * \param[out] cloud the resultant point cloud model dataset which is transformed to
   hypothesis.
   **/
  void
  computeTransformedPointCloud(const StateT& hypothesis,
                               pcl::Indices& indices,
                               PointCloudIn& cloud);

#ifdef DOXYGEN_ONLY
  /** \brief Compute a reference pointcloud transformed to the pose that hypothesis
   * represents and calculate indices taking occlusion into account.
   * \param[in] hypothesis a particle which represents a hypothesis.
   * \param[in] indices the indices which should be taken into account.
   * \param[out] cloud the resultant point cloud model dataset which is transformed to
   hypothesis.
   **/
  void
  computeTransformedPointCloudWithNormal(const StateT& hypothesis,
                                         pcl::Indices& indices,
                                         PointCloudIn& cloud);
#else
  template <typename PointT = PointInT, traits::HasNormal<PointT> = true>
  void
  computeTransformedPointCloudWithNormal(const StateT& hypothesis,
                                         pcl::Indices& indices,
                                         PointCloudIn& cloud);
  template <typename PointT = PointInT, traits::HasNoNormal<PointT> = true>
  void
  computeTransformedPointCloudWithNormal(const StateT&, pcl::Indices&, PointCloudIn&)
  {
    PCL_WARN("[pcl::%s::computeTransformedPointCloudWithNormal] "
             "use_normal_ == true is not supported in this Point Type.\n",
             getClassName().c_str());
  }
#endif

  /** \brief Compute a reference pointcloud transformed to the pose that hypothesis
   * represents and calculate indices without taking occlusion into account.
   * \param[in] hypothesis a particle which represents a hypothesis.
   * \param[out] cloud the resultant point cloud model dataset which is transformed to
   *hypothesis.
   **/
  void
  computeTransformedPointCloudWithoutNormal(const StateT& hypothesis,
                                            PointCloudIn& cloud);

  /** \brief This method should get called before starting the actua computation. */
  bool
  initCompute() override;

  /** \brief Weighting phase of particle filter method. Calculate the likelihood
   * of all of the particles and set the weights. */
  virtual void
  weight();

  /** \brief Resampling phase of particle filter method. Sampling the particles
   * according to the weights calculated in weight method. In particular,
   * "sample with replacement" is archieved by walker's alias method.
   */
  virtual void
  resample();

  /** \brief Calculate the weighted mean of the particles and set it as the result. */
  virtual void
  update();

  /** \brief Normalize the weights of all the particels. */
  virtual void
  normalizeWeight();

  /** \brief Initialize the particles. initial_noise_covariance_ and
   * initial_noise_mean_ are used for Gaussian sampling. */
  void
  initParticles(bool reset);

  /** \brief Track the pointcloud using particle filter method. */
  void
  computeTracking() override;

  /** \brief Implementation of "sample with replacement" using Walker's alias method.
   * about Walker's alias method, you can check the paper below: article{355749}, author
   * = {Walker, Alastair J.}, title = {An Efficient Method for Generating Discrete
   * Random Variables with General Distributions},
   * journal = {ACM Trans. Math. Softw.},
   * volume = {3},
   * number = {3},
   * year = {1977},
   * issn = {0098-3500},
   * pages = {253--256},
   * doi = {http://doi.acm.org/10.1145/355744.355749},
   * publisher = {ACM},
   * address = {New York, NY, USA},
   * }
   * \param a an alias table, which generated by genAliasTable.
   * \param q a table of weight, which generated by genAliasTable.
   */
  int
  sampleWithReplacement(const std::vector<int>& a, const std::vector<double>& q);

  /** \brief Generate the tables for walker's alias method. */
  void
  genAliasTable(std::vector<int>& a,
                std::vector<double>& q,
                const PointCloudStateConstPtr& particles);

  /** \brief Resampling the particle with replacement. */
  void
  resampleWithReplacement();

  /** \brief Resampling the particle in deterministic way. */
  void
  resampleDeterministic();

  /** \brief Run change detection and return true if there is a change.
   * \param[in] input a pointer to the input pointcloud.
   */
  bool
  testChangeDetection(const PointCloudInConstPtr& input);

  /** \brief The number of iteration of particlefilter. */
  int iteration_num_;

  /** \brief The number of the particles. */
  int particle_num_;

  /** \brief The minimum number of points which the hypothesis should have. */
  int min_indices_;

  /** \brief Adjustment of the particle filter. */
  double fit_ratio_;

  /** \brief A pointer to reference point cloud. */
  PointCloudInConstPtr ref_;

  /** \brief A pointer to the particles  */
  PointCloudStatePtr particles_;

  /** \brief A pointer to PointCloudCoherence. */
  CloudCoherencePtr coherence_;

  /** \brief The diagonal elements of covariance matrix of the step noise. the
   * covariance matrix is used at every resample method.
   */
  std::vector<double> step_noise_covariance_;

  /** \brief The diagonal elements of covariance matrix of the initial noise.
   * the covariance matrix is used when initialize the particles.
   */
  std::vector<double> initial_noise_covariance_;

  /** \brief The mean values of initial noise. */
  std::vector<double> initial_noise_mean_;

  /** \brief The threshold for the particles to be re-initialized. */
  double resample_likelihood_thr_;

  /** \brief The threshold for the points to be considered as occluded. */
  double occlusion_angle_thr_;

  /** \brief The weight to be used in normalization of the weights of the
   * particles. */
  double alpha_;

  /** \brief The result of tracking. */
  StateT representative_state_;

  /** \brief An affine transformation from the world coordinates frame to the
   * origin of the particles. */
  Eigen::Affine3f trans_;

  /** \brief A flag to use normal or not. defaults to false. */
  bool use_normal_;

  /** \brief Difference between the result in t and t-1. */
  StateT motion_;

  /** \brief Ratio of hypothesis to use motion model. */
  double motion_ratio_;

  /** \brief Pass through filter to crop the pointclouds within the hypothesis
   * bounding box. */
  pcl::PassThrough<PointInT> pass_x_;
  /** \brief Pass through filter to crop the pointclouds within the hypothesis
   * bounding box. */
  pcl::PassThrough<PointInT> pass_y_;
  /** \brief Pass through filter to crop the pointclouds within the hypothesis
   * bounding box. */
  pcl::PassThrough<PointInT> pass_z_;

  /** \brief A list of the pointers to pointclouds. */
  std::vector<PointCloudInPtr> transed_reference_vector_;

  /** \brief Change detector used as a trigger to track. */
  typename pcl::octree::OctreePointCloudChangeDetector<PointInT>::Ptr change_detector_;

  /** \brief A flag to be true when change of pointclouds is detected. */
  bool changed_;

  /** \brief A counter to skip change detection. */
  unsigned int change_counter_;

  /** \brief Minimum points in a leaf when calling change detector. defaults
   * to 10. */
  unsigned int change_detector_filter_;

  /** \brief The number of interval frame to run change detection. defaults
   * to 10. */
  unsigned int change_detector_interval_;

  /** \brief Resolution of change detector. defaults to 0.01. */
  double change_detector_resolution_;

  /** \brief The flag which will be true if using change detection. */
  bool use_change_detector_;
};
} // namespace tracking
} // namespace pcl

// #include <pcl/tracking/impl/particle_filter.hpp>
#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/particle_filter.hpp>
#endif
