#ifndef PCL_TRACKING_PARTICLE_FILTER_H_
#define PCL_TRACKING_PARTICLE_FILTER_H_

#include "pcl/tracking/tracking.h"
#include "pcl/tracking/tracker.h"
#include "pcl/tracking/coherence.h"
#include <Eigen/Dense>

namespace pcl
{

  namespace tracking
  {
    
    /** \brief @b ParticleFilterTracker tracks the PointCloud which is given by
        setReferenceCloud within the measured PointCloud using particle filter method.
      * \author Ryohei Ueda
      * \ingroup tracking
      */
    template <typename PointInT, typename StateT>
    class ParticleFilterTracker: public Tracker<PointInT, StateT>
    {
    protected:
      using Tracker<PointInT, StateT>::deinitCompute;
      
    public:
      using Tracker<PointInT, StateT>::tracker_name_;
      using Tracker<PointInT, StateT>::search_;
      using Tracker<PointInT, StateT>::input_;
      using Tracker<PointInT, StateT>::indices_;
      using Tracker<PointInT, StateT>::getClassName;
      
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
      ParticleFilterTracker ()
      : iteration_num_ (1)
      , min_indices_ (1)
      , resample_likelihood_thr_ (0.0)
      , occlusion_angle_thr_ (M_PI / 2.0)
      {
        tracker_name_ = "ParticleFilterTracker";
      }
      
      /** \brief set the number of iteration.
        * \param iteration_num the number of iteration.
        */
      inline void
      setIterationNum (const int iteration_num) { iteration_num_ = iteration_num; }

      /** \brief get the number of iteration. */
      inline int
      getIterationNum () const { return iteration_num_; }

      /** \brief set the number of the particles.
        * \param particle_num the number of the particles.
        */
      inline void
      setParticleNum (const int particle_num) { particle_num_ = particle_num; }

      /** \brief get the number of the particles. */
      inline int
      getParticleNum () const { return particle_num_; }

      /** \brief set a pointer to a reference dataset to be tracked.
        * \param cloud a pointer to a PointCloud message
        */
      inline void
      setReferenceCloud (const PointCloudInConstPtr &ref) { ref_ = ref; }

      /** \brief get a pointer to a reference dataset to be tracked. */
      inline PointCloudInConstPtr const
      getReferenceCloud () { return ref_; }

      /** \brief set the PointCloudCoherence as likelihood.
        * \param coherence a pointer to PointCloudCoherence.
        */
      inline void
      setCloudCoherence (const CloudCoherencePtr &coherence) { coherence_ = coherence; }
      
      /** \brief get the PointCloudCoherence to compute likelihood. */
      inline CloudCoherencePtr
      getCloudCoherence () const { return coherence_; }
      

      /** \brief set the covariance of step noise.
        * \param step_noise_covariance the diagonal elements of covariance matrix of step noise.
        */
      inline void
      setStepNoiseCovariance (const std::vector<double> &step_noise_covariance)
      {
        step_noise_covariance_ = step_noise_covariance;
      }

      /** \brief set the covariance of the initial noise.
          it will be used when initializing the particles.
        * \param initial_noise_covariance the diagonal elements of covariance matrix of initial noise.
        */
      inline void
      setInitialNoiseCovariance (const std::vector<double> &initial_noise_covariance)
      {
        initial_noise_covariance_ = initial_noise_covariance;
      }

      /** \brief set the mean of the initial noise.
          it will be used when initializing the particles.
        * \param initial_noise_mean the mean values of initial noise.
        */
      inline void
      setInitialNoiseMean (const std::vector<double> &initial_noise_mean)
      {
        initial_noise_mean_ = initial_noise_mean;
      }

      /** \brief set the threshold to re-initialize the particles.
        * \param resample_likelihood_thr threshold to re-initialize.
        */
      inline void
      setResampleLikelihoodThr (const double resample_likelihood_thr)
      {
        resample_likelihood_thr_ = resample_likelihood_thr;
      }
      
      /** \brief set the threshold of angle to be considered occlusion (default: pi/2).
          ParticleFilterTracker does not take the occluded points into account according to the angle
          between the normal and the position. 
        * \param occlusion_angle_thr threshold of angle to be considered occlusion.
        */
      inline void
      setOcclusionAngleThe (const double occlusion_angle_thr)
      {
        occlusion_angle_thr_ = occlusion_angle_thr;
      }
      
      /** \brief set the minimum number of indices (default: 1).
          ParticleFilterTracker does not take into account the hypothesis
          whose the number of points is smaller than the minimum indices.
        * \param min_indices the minimum number of indices.
        */
      inline void
      setMinIndices (const int min_indices) { min_indices_ = min_indices; }

      /** \brief set the transformation from the world coordinates to the frame of the particles.
        * \param trans Affine transformation from the worldcoordinates to the frame of the particles.
        */
      inline void setTrans (const Eigen::Affine3f &trans) { trans_ = trans; }
      /** \brief get the transformation from the world coordinates to the frame of the particles. */
      inline Eigen::Affine3f getTrans () const { return trans_; }

      /** \brief Get an instance of the result of tracking. */
      inline StateT getResult () const { return representative_state_; }
      
      /** \brief convert a state to affine transformation from the world coordinates frame.
        * \param particle an instance of StateT.
        */
      inline Eigen::Affine3f toEigenMatrix (const StateT& particle);

      /** \brief get a pointer to a pointcloud of the particles.*/
      inline PointCloudStatePtr getParticles () const { return particles_; }
      
    protected:
      
      /** \brief This method should get called before starting the actual computation. */
      virtual bool initCompute ();
      
      /** \brief weighting phase of particle filter method.
          calculate the likelihood of all of the particles and set the weights.
        */
      virtual void weight ();
      
      /** \brief resampling phase of particle filter method.
          sampling the particles according to the weights calculated in weight method.
          in particular, "sample with replacement" is archieved by walker's alias method.
        */
      virtual void resample ();
      
      /** \brief calculate the weighted mean of the particles and set it as the result */
      virtual void update ();

      /** \brief normalize the weights of all the particels. */
      void normalizeWeight ();

      /** \brief initialize the particles. initial_noise_covariance_ and initial_noise_mean_ are
          used for gausiaan sampling.
        */
      void initParticles ();
      
      /** \brief track the pointcloud using particle filter method.
        */
      void computeTracking ();
      
      /** \brief implementation of "sample with replacement" using Walker's alias method.
          about Walker's alias method, you can check the paper below:
          @article{355749,
           author = {Walker, Alastair J.},
           title = {An Efficient Method for Generating Discrete
           Random Variables with General Distributions},
           journal = {ACM Trans. Math. Softw.},
           volume = {3},
           number = {3},
           year = {1977},
           issn = {0098-3500},
           pages = {253--256},
           doi = {http://doi.acm.org/10.1145/355744.355749},
           publisher = {ACM},
           address = {New York, NY, USA},
           }
           \param a an alias table, which generated by genAliasTable.
           \param q a table of weight, which generated by genAliasTable.
       */
      inline int sampleWithReplacement (const std::vector<int>& a, const std::vector<double>& q);
      
      /** \brief generate the tables for walker's alias method */
      inline void genAliasTable (std::vector<int> &a, std::vector<double> &q);

      /** \brief calculate the likelihood of hypothesis. the likelihood is defined by coherence_.
        * \param hypothesis an instance of hypothesis.
        */
      inline double calcLikelihood (const StateT& hypothesis);
      
      /** \brief the number of iteration of particlefilter. */
      int iteration_num_;

      /** \brief the number of the particles. */
      int particle_num_;

      /** \brief the minimum number of points which the hypothesis should have. */
      int min_indices_;

      /** \brief a pointer to reference point cloud. */
      PointCloudInConstPtr ref_;

      /** \brief a pointer to the particles  */
      PointCloudStatePtr particles_;

      /** \brief a pointer to PointCloudCoherence. */
      CloudCoherencePtr coherence_;

      /** \brief the diagonal elements of covariance matrix of the step noise. the covariance matrix is used
          at every resample method.
        */
      std::vector<double> step_noise_covariance_;

      /** \brief the diagonal elements of covariance matrix of the initial noise. the covariance matrix is used
          when initialize the particles.
        */
      std::vector<double> initial_noise_covariance_;
      
      /** \brief the mean values of initial noise.*/
      std::vector<double> initial_noise_mean_;

      /** \brief the threshold for the particles to be re-initialized*/
      double resample_likelihood_thr_;

      /** \brief the threshold for the points to be considered as occluded*/
      double occlusion_angle_thr_;
      
      /** \brief the result of tracking. */
      StateT representative_state_;

      /** \brief an affine transformation from the world coordinates frame to the origin of the particles*/
      Eigen::Affine3f trans_;
      
    };
    
  }
}

#include "pcl/tracking/impl/particle_filter.hpp"

#endif //PCL_TRACKING_PARTICLE_FILTER_H_
