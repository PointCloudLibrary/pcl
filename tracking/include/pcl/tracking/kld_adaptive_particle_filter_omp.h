#pragma once

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/kld_adaptive_particle_filter.h>
#include <pcl/tracking/tracking.h>

namespace pcl {
namespace tracking {
/** \brief @b KLDAdaptiveParticleFilterOMPTracker tracks the PointCloud which is given
 * by setReferenceCloud within the measured PointCloud using particle filter method. The
 * number of the particles changes adaptively based on KLD sampling [D. Fox, NIPS-01],
 * [D.Fox, IJRR03]. and the computation of the weights of the particles is parallelized
 * using OpenMP.
 * \author Ryohei Ueda
 * \ingroup tracking
 */
template <typename PointInT, typename StateT>
class KLDAdaptiveParticleFilterOMPTracker
: public KLDAdaptiveParticleFilterTracker<PointInT, StateT> {
public:
  using Tracker<PointInT, StateT>::tracker_name_;
  using Tracker<PointInT, StateT>::search_;
  using Tracker<PointInT, StateT>::input_;
  using Tracker<PointInT, StateT>::indices_;
  using Tracker<PointInT, StateT>::getClassName;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::particles_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::change_detector_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::change_counter_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::change_detector_interval_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::use_change_detector_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::pass_x_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::pass_y_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::pass_z_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::alpha_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::changed_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::coherence_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::use_normal_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::particle_num_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::change_detector_filter_;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::transed_reference_vector_;
  // using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::calcLikelihood;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::normalizeWeight;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::normalizeParticleWeight;
  using KLDAdaptiveParticleFilterTracker<PointInT, StateT>::calcBoundingBox;

  using BaseClass = Tracker<PointInT, StateT>;

  using Ptr = shared_ptr<KLDAdaptiveParticleFilterOMPTracker<PointInT, StateT>>;
  using ConstPtr =
      shared_ptr<const KLDAdaptiveParticleFilterOMPTracker<PointInT, StateT>>;

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

  /** \brief Initialize the scheduler and set the number of threads to use.
   * \param nr_threads the number of hardware threads to use (0 sets the value
   * back to automatic)
   */
  KLDAdaptiveParticleFilterOMPTracker(unsigned int nr_threads = 0)
  : KLDAdaptiveParticleFilterTracker<PointInT, StateT>()
  {
    tracker_name_ = "KLDAdaptiveParticleFilterOMPTracker";

    setNumberOfThreads(nr_threads);
  }

  /** \brief Initialize the scheduler and set the number of threads to use.
   * \param nr_threads the number of hardware threads to use (0 sets the value back to
   * automatic)
   */
  void
  setNumberOfThreads(unsigned int nr_threads = 0);

protected:
  /** \brief The number of threads the scheduler should use. */
  unsigned int threads_;

  /** \brief weighting phase of particle filter method. calculate the likelihood of all
   * of the particles and set the weights.
   */
  void
  weight() override;
};
} // namespace tracking
} // namespace pcl

//#include <pcl/tracking/impl/particle_filter_omp.hpp>
#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/kld_adaptive_particle_filter_omp.hpp>
#endif
