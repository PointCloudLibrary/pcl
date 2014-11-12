/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


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
