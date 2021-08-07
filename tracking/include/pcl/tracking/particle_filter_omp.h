#pragma once

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/tracking.h>

namespace pcl {
namespace tracking {
/** \brief @b ParticleFilterOMPTracker tracks the PointCloud which is given by
 * setReferenceCloud within the measured PointCloud using particle filter method in
 * parallel, using the OpenMP standard. \author Ryohei Ueda \ingroup tracking
 */
template <typename PointInT, typename StateT>
class ParticleFilterOMPTracker : public ParticleFilterTracker<PointInT, StateT> {
public:
  using Tracker<PointInT, StateT>::tracker_name_;
  using Tracker<PointInT, StateT>::search_;
  using Tracker<PointInT, StateT>::input_;
  using Tracker<PointInT, StateT>::indices_;
  using Tracker<PointInT, StateT>::getClassName;
  using ParticleFilterTracker<PointInT, StateT>::particles_;
  using ParticleFilterTracker<PointInT, StateT>::change_detector_;
  using ParticleFilterTracker<PointInT, StateT>::change_counter_;
  using ParticleFilterTracker<PointInT, StateT>::change_detector_interval_;
  using ParticleFilterTracker<PointInT, StateT>::use_change_detector_;
  using ParticleFilterTracker<PointInT, StateT>::alpha_;
  using ParticleFilterTracker<PointInT, StateT>::changed_;
  using ParticleFilterTracker<PointInT, StateT>::coherence_;
  using ParticleFilterTracker<PointInT, StateT>::use_normal_;
  using ParticleFilterTracker<PointInT, StateT>::particle_num_;
  using ParticleFilterTracker<PointInT, StateT>::change_detector_filter_;
  using ParticleFilterTracker<PointInT, StateT>::transed_reference_vector_;
  // using ParticleFilterTracker<PointInT, StateT>::calcLikelihood;
  using ParticleFilterTracker<PointInT, StateT>::normalizeWeight;
  using ParticleFilterTracker<PointInT, StateT>::normalizeParticleWeight;
  using ParticleFilterTracker<PointInT, StateT>::calcBoundingBox;

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

  /** \brief Initialize the scheduler and set the number of threads to use.
   * \param nr_threads the number of hardware threads to use (0 sets the value
   * back to automatic)
   */
  ParticleFilterOMPTracker(unsigned int nr_threads = 0)
  : ParticleFilterTracker<PointInT, StateT>()
  {
    tracker_name_ = "ParticleFilterOMPTracker";

    setNumberOfThreads(nr_threads);
  }

  /** \brief Initialize the scheduler and set the number of threads to use.
   * \param nr_threads the number of hardware threads to use (0 sets the value
   * back to automatic)
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
#include <pcl/tracking/impl/particle_filter_omp.hpp>
#endif
