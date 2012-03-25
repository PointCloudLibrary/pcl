/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id$
 *
 */

#ifndef PCL_SAMPLE_CONSENSUS_H_
#define PCL_SAMPLE_CONSENSUS_H_

#include <pcl/sample_consensus/sac_model.h>
#include <boost/random.hpp>
#include <ctime>
#include <set>

namespace pcl
{
  /** \brief SampleConsensus represents the base class. All sample consensus methods must inherit from this class.
    * \author Radu Bogdan Rusu
    * \ingroup sample_consensus
    */
  template <typename T>
  class SampleConsensus
  {
    typedef typename SampleConsensusModel<T>::Ptr SampleConsensusModelPtr;

    private:
      /** \brief Constructor for base SAC. */
      SampleConsensus () {};

    public:
      typedef boost::shared_ptr<SampleConsensus> Ptr;
      typedef boost::shared_ptr<const SampleConsensus> ConstPtr;

      /** \brief Constructor for base SAC.
        * \param[in] model a Sample Consensus model
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensus (const SampleConsensusModelPtr &model, bool random = false) : 
        sac_model_ (model), 
        model_ (), 
        inliers_ (), 
        model_coefficients_ (), 
        probability_ (0.99), 
        iterations_ (0), 
        threshold_ (std::numeric_limits<double>::max()),
        max_iterations_ (1000), 
        rng_alg_ (), 
        rng_ (new boost::uniform_01<boost::mt19937> (rng_alg_))
      {
         // Create a random number generator object
         if (random)
           rng_->base ().seed (static_cast<unsigned> (std::time(0)));
         else
           rng_->base ().seed (12345u);
      };

      /** \brief Constructor for base SAC.
        * \param[in] model a Sample Consensus model
        * \param[in] threshold distance to model threshol
        * \param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
        */
      SampleConsensus (const SampleConsensusModelPtr &model, double threshold, bool random = false) : 
        sac_model_ (model), 
        model_ (), 
        inliers_ (), 
        model_coefficients_ (), 
        probability_ (0.99), 
        iterations_ (0), 
        threshold_ (threshold), 
        max_iterations_ (1000), 
        rng_alg_ (), 
        rng_ (new boost::uniform_01<boost::mt19937> (rng_alg_))
      {
         // Create a random number generator object
         if (random)
           rng_->base ().seed (static_cast<unsigned> (std::time(0)));
         else
           rng_->base ().seed (12345u);
       };

      /** \brief Destructor for base SAC. */
      virtual ~SampleConsensus () {};

      /** \brief Set the distance to model threshold.
        * \param[in] threshold distance to model threshold
        */
      inline void 
      setDistanceThreshold (double threshold)  { threshold_ = threshold; }

      /** \brief Get the distance to model threshold, as set by the user. */
      inline double 
      getDistanceThreshold () { return (threshold_); }

      /** \brief Set the maximum number of iterations.
        * \param[in] max_iterations maximum number of iterations
        */
      inline void 
      setMaxIterations (int max_iterations) { max_iterations_ = max_iterations; }

      /** \brief Get the maximum number of iterations, as set by the user. */
      inline int 
      getMaxIterations () { return (max_iterations_); }

      /** \brief Set the desired probability of choosing at least one sample free from outliers.
        * \param[in] probability the desired probability of choosing at least one sample free from outliers
        * \note internally, the probability is set to 99% (0.99) by default.
        */
      inline void 
      setProbability (double probability) { probability_ = probability; }

      /** \brief Obtain the probability of choosing at least one sample free from outliers, as set by the user. */
      inline double 
      getProbability () { return (probability_); }

      /** \brief Compute the actual model. Pure virtual. */
      virtual bool 
      computeModel (int debug_verbosity_level = 0) = 0;

      /** \brief Get a set of randomly selected indices.
        * \param[in] indices the input indices vector
        * \param[in] nr_samples the desired number of point indices to randomly select
        * \param[out] indices_subset the resultant output set of randomly selected indices
        */
      inline void
      getRandomSamples (const boost::shared_ptr <std::vector<int> > &indices, 
                        size_t nr_samples, 
                        std::set<int> &indices_subset)
      {
        indices_subset.clear ();
        while (indices_subset.size () < nr_samples)
          //indices_subset.insert ((*indices)[(int) (indices->size () * (rand () / (RAND_MAX + 1.0)))]);
          indices_subset.insert ((*indices)[static_cast<int> (static_cast<double>(indices->size ()) * rnd ())]);
      }

      /** \brief Return the best model found so far. 
        * \param[out] model the resultant model
        */
      inline void 
      getModel (std::vector<int> &model) { model = model_; }

      /** \brief Return the best set of inliers found so far for this model. 
        * \param[out] inliers the resultant set of inliers
        */
      inline void 
      getInliers (std::vector<int> &inliers) { inliers = inliers_; }

      /** \brief Return the model coefficients of the best model found so far. 
        * \param[out] model_coefficients the resultant model coefficients
        */
      inline void 
      getModelCoefficients (Eigen::VectorXf &model_coefficients) { model_coefficients = model_coefficients_; }

    protected:
      /** \brief The underlying data model used (i.e. what is it that we attempt to search for). */
      SampleConsensusModelPtr sac_model_;

      /** \brief The model found after the last computeModel () as point cloud indices. */
      std::vector<int> model_;

      /** \brief The indices of the points that were chosen as inliers after the last computeModel () call. */
      std::vector<int> inliers_;

      /** \brief The coefficients of our model computed directly from the model found. */
      Eigen::VectorXf model_coefficients_;

      /** \brief Desired probability of choosing at least one sample free from outliers. */
      double probability_;

      /** \brief Total number of internal loop iterations that we've done so far. */
      int iterations_;
      
      /** \brief Distance to model threshold. */
      double threshold_;
      
      /** \brief Maximum number of iterations before giving up. */
      int max_iterations_;

      /** \brief Boost-based random number generator algorithm. */
      boost::mt19937 rng_alg_;

      /** \brief Boost-based random number generator distribution. */
      boost::shared_ptr<boost::uniform_01<boost::mt19937> > rng_;

      /** \brief Boost-based random number generator. */
      inline double
      rnd ()
      {
        return ((*rng_) ());
      }
   };
}

#endif  //#ifndef PCL_SAMPLE_CONSENSUS_H_
