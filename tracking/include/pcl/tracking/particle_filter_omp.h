#ifndef PCL_TRACKING_PARTICLE_FILTER_OMP_H_
#define PCL_TRACKING_PARTICLE_FILTER_OMP_H_

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/coherence.h>

namespace pcl
{
  namespace tracking
  {
  /** \brief @b ParticleFilterOMPTracker tracks the PointCloud which is given by
      setReferenceCloud within the measured PointCloud using particle filter method 
      in parallel, using the OpenMP standard.
    * \author Ryohei Ueda
    * \ingroup tracking
    */
    template <typename PointInT, typename StateT>
    class ParticleFilterOMPTracker: public ParticleFilterTracker<PointInT, StateT>
    {
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
      //using ParticleFilterTracker<PointInT, StateT>::calcLikelihood;
      using ParticleFilterTracker<PointInT, StateT>::normalizeWeight;
      using ParticleFilterTracker<PointInT, StateT>::normalizeParticleWeight;
      using ParticleFilterTracker<PointInT, StateT>::calcBoundingBox;

      typedef Tracker<PointInT, StateT> BaseClass;
      
      typedef typename Tracker<PointInT, StateT>::PointCloudIn PointCloudIn;
      typedef typename PointCloudIn::Ptr PointCloudInPtr;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

      typedef typename Tracker<PointInT, StateT>::PointCloudState PointCloudState;
      typedef typename PointCloudState::Ptr PointCloudStatePtr;
      typedef typename PointCloudState::ConstPtr PointCloudStateConstPtr;

      typedef PointCoherence<PointInT> Coherence;
      typedef boost::shared_ptr< Coherence > CoherencePtr;
      typedef boost::shared_ptr< const Coherence > CoherenceConstPtr;

      typedef PointCloudCoherence<PointInT> CloudCoherence;
      typedef boost::shared_ptr< CloudCoherence > CloudCoherencePtr;
      typedef boost::shared_ptr< const CloudCoherence > CloudCoherenceConstPtr;

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */      
      ParticleFilterOMPTracker (unsigned int nr_threads = 0)
      : ParticleFilterTracker<PointInT, StateT> ()
      , threads_ (nr_threads)
      {
        tracker_name_ = "ParticleFilterOMPTracker";
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      inline void
      setNumberOfThreads (unsigned int nr_threads = 0) { threads_ = nr_threads; }
      
    protected:
      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

      /** \brief weighting phase of particle filter method.
          calculate the likelihood of all of the particles and set the weights.
        */
      virtual void weight ();

    };
  }
}

//#include <pcl/tracking/impl/particle_filter_omp.hpp>
#ifdef PCL_NO_PRECOMPILE
#include <pcl/tracking/impl/particle_filter_omp.hpp>
#endif

#endif
