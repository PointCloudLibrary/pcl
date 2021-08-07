#pragma once

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/tracking.h>

namespace pcl {
namespace tracking {

/** \brief @b KLDAdaptiveParticleFilterTracker tracks the PointCloud which is given by
 * setReferenceCloud within the measured PointCloud using particle filter method. The
 * number of the particles changes adaptively based on KLD sampling [D. Fox, NIPS-01],
 * [D.Fox, IJRR03].
 * \author Ryohei Ueda
 * \ingroup tracking
 */
template <typename PointInT, typename StateT>
class KLDAdaptiveParticleFilterTracker
: public ParticleFilterTracker<PointInT, StateT> {
public:
  using Tracker<PointInT, StateT>::tracker_name_;
  using Tracker<PointInT, StateT>::search_;
  using Tracker<PointInT, StateT>::input_;
  using Tracker<PointInT, StateT>::getClassName;
  using ParticleFilterTracker<PointInT, StateT>::transed_reference_vector_;
  using ParticleFilterTracker<PointInT, StateT>::coherence_;
  using ParticleFilterTracker<PointInT, StateT>::initParticles;
  using ParticleFilterTracker<PointInT, StateT>::weight;
  using ParticleFilterTracker<PointInT, StateT>::update;
  using ParticleFilterTracker<PointInT, StateT>::iteration_num_;
  using ParticleFilterTracker<PointInT, StateT>::particle_num_;
  using ParticleFilterTracker<PointInT, StateT>::particles_;
  using ParticleFilterTracker<PointInT, StateT>::use_normal_;
  using ParticleFilterTracker<PointInT, StateT>::use_change_detector_;
  using ParticleFilterTracker<PointInT, StateT>::change_detector_resolution_;
  using ParticleFilterTracker<PointInT, StateT>::change_detector_;
  using ParticleFilterTracker<PointInT, StateT>::motion_;
  using ParticleFilterTracker<PointInT, StateT>::motion_ratio_;
  using ParticleFilterTracker<PointInT, StateT>::step_noise_covariance_;
  using ParticleFilterTracker<PointInT, StateT>::representative_state_;
  using ParticleFilterTracker<PointInT, StateT>::sampleWithReplacement;

  using BaseClass = Tracker<PointInT, StateT>;

  using Ptr = shared_ptr<KLDAdaptiveParticleFilterTracker<PointInT, StateT>>;
  using ConstPtr = shared_ptr<const KLDAdaptiveParticleFilterTracker<PointInT, StateT>>;

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
  KLDAdaptiveParticleFilterTracker()
  : ParticleFilterTracker<PointInT, StateT>()
  , maximum_particle_number_()
  , epsilon_(0)
  , delta_(0.99)
  , bin_size_()
  {
    tracker_name_ = "KLDAdaptiveParticleFilterTracker";
  }

  /** \brief set the bin size.
   * \param bin_size the size of a bin
   */
  inline void
  setBinSize(const StateT& bin_size)
  {
    bin_size_ = bin_size;
  }

  /** \brief get the bin size. */
  inline StateT
  getBinSize() const
  {
    return (bin_size_);
  }

  /** \brief set the maximum number of the particles.
   * \param nr the maximum number of the particles.
   */
  inline void
  setMaximumParticleNum(unsigned int nr)
  {
    maximum_particle_number_ = nr;
  }

  /** \brief get the maximum number of the particles.*/
  inline unsigned int
  getMaximumParticleNum() const
  {
    return (maximum_particle_number_);
  }

  /** \brief set epsilon to be used to calc K-L boundary.
   * \param eps epsilon
   */
  inline void
  setEpsilon(double eps)
  {
    epsilon_ = eps;
  }

  /** \brief get epsilon to be used to calc K-L boundary. */
  inline double
  getEpsilon() const
  {
    return (epsilon_);
  }

  /** \brief set delta to be used in chi-squared distribution.
   * \param delta delta of chi-squared distribution.
   */
  inline void
  setDelta(double delta)
  {
    delta_ = delta;
  }

  /** \brief get delta to be used in chi-squared distribution.*/
  inline double
  getDelta() const
  {
    return (delta_);
  }

protected:
  /** \brief return true if the two bins are equal.
   * \param a index of the bin
   * \param b index of the bin
   */
  virtual bool
  equalBin(const std::vector<int>& a, const std::vector<int>& b)
  {
    int dimension = StateT::stateDimension();
    for (int i = 0; i < dimension; i++)
      if (a[i] != b[i])
        return (false);
    return (true);
  }

  /** \brief return upper quantile of standard normal distribution.
   * \param[in] u ratio of quantile.
   */
  double
  normalQuantile(double u)
  {
    const double a[9] = {1.24818987e-4,
                         -1.075204047e-3,
                         5.198775019e-3,
                         -0.019198292004,
                         0.059054035642,
                         -0.151968751364,
                         0.319152932694,
                         -0.5319230073,
                         0.797884560593};
    const double b[15] = {-4.5255659e-5,
                          1.5252929e-4,
                          -1.9538132e-5,
                          -6.76904986e-4,
                          1.390604284e-3,
                          -7.9462082e-4,
                          -2.034254874e-3,
                          6.549791214e-3,
                          -0.010557625006,
                          0.011630447319,
                          -9.279453341e-3,
                          5.353579108e-3,
                          -2.141268741e-3,
                          5.35310549e-4,
                          0.999936657524};
    double w, y, z;

    if (u == 0.)
      return (0.5);
    y = u / 2.0;
    if (y < -3.)
      return (0.0);
    if (y > 3.)
      return (1.0);
    if (y < 0.0)
      y = -y;
    if (y < 1.0) {
      w = y * y;
      z = a[0];
      for (int i = 1; i < 9; i++)
        z = z * w + a[i];
      z *= (y * 2.0);
    }
    else {
      y -= 2.0;
      z = b[0];
      for (int i = 1; i < 15; i++)
        z = z * y + b[i];
    }

    if (u < 0.0)
      return ((1. - z) / 2.0);
    return ((1. + z) / 2.0);
  }

  /** \brief calculate K-L boundary. K-L boundary follows 1/2e*chi(k-1, 1-d)^2.
   * \param[in] k the number of bins and the first parameter of chi distribution.
   */
  virtual double
  calcKLBound(int k)
  {
    double z = normalQuantile(delta_);
    double chi = 1.0 - 2.0 / (9.0 * (k - 1)) + sqrt(2.0 / (9.0 * (k - 1))) * z;
    return ((k - 1.0) / (2.0 * epsilon_) * chi * chi * chi);
  }

  /** \brief insert a bin into the set of the bins. if that bin is already registered,
   * return false. if not, return true.
   * \param new_bin a bin to be inserted.
   * \param bins a set of the bins
   */
  virtual bool
  insertIntoBins(std::vector<int>&& new_bin, std::vector<std::vector<int>>& bins);

  /** \brief This method should get called before starting the actual
   * computation. */
  bool
  initCompute() override;

  /** \brief resampling phase of particle filter method. sampling the particles
   * according to the weights calculated in weight method. in particular, "sample with
   * replacement" is archieved by walker's alias method.
   */
  void
  resample() override;

  /** \brief the maximum number of the particles. */
  unsigned int maximum_particle_number_;

  /** \brief error between K-L distance and MLE*/
  double epsilon_;

  /** \brief probability of distance between K-L distance and MLE is less than
   * epsilon_*/
  double delta_;

  /** \brief the size of a bin.*/
  StateT bin_size_;
};
} // namespace tracking
} // namespace pcl

#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/kld_adaptive_particle_filter.hpp>
#endif
