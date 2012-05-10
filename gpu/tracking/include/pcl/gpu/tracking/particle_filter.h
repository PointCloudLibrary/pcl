#ifndef PCL_GPU_TRACKING_PARTICLE_FILTER_H_
#define PCL_GPU_TRACKING_PARTICLE_FILTER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/pcl_macros.h>

#include <pcl/tracking/particle_filter.h>

#include <Eigen/Dense>

namespace pcl
{
  namespace gpu
  {
     template <typename PointInT, typename StateT>
    class ParticleFilterGPUTracker : public tracking::ParticleFilterTracker<PointInT, StateT>
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

			/** \brief Empty constructor. */
			ParticleFilterGPUTracker ()
				: ParticleFilterTracker<PointInT, StateT> ()
			{
				tracker_name_ = "ParticleFilterGPUTracker";
			}
			
    protected:
			virtual bool 
			initCompute();

			// sampling
			virtual void 
			resample ();
			
			virtual void 
			weight ();
			
			virtual void
			normalizeWeight();

			// Resampling : Particle re-allocation
			virtual void
			update ();

			virtual void 
			computeTracking();

			int finite_redraw_times_;


    };
  }
}

#endif // PCL_GPU_TRACKING_PARTICLE_FILTER_H_
