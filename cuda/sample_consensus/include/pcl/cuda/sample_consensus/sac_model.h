/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef PCL_CUDA_SAMPLE_CONSENSUS_MODEL_H_
#define PCL_CUDA_SAMPLE_CONSENSUS_MODEL_H_

#include <float.h>
#include <thrust/sequence.h>
#include <thrust/count.h>
#include <thrust/remove.h>
#include <pcl/cuda/point_cloud.h>
#include <thrust/random/linear_congruential_engine.h>

#include <pcl/pcl_exports.h>

namespace pcl
{
  namespace cuda
  {
    // Forward declaration
    //template<class T> class ProgressiveSampleConsensus;

    /** \brief Check if a certain tuple is a point inlier. */
    struct DeleteIndices
    {
      template <typename Tuple> __inline__ __host__ __device__ int
      operator () (const Tuple &t);
    };

    /** \brief Check if a certain tuple is a point inlier. */
    struct isInlier
    {
        __inline__ __host__ __device__ bool 
        operator()(int x) { return (x != -1); }
    };

    struct isNaNPoint
    {
        __inline__ __host__ __device__ bool 
        operator ()(PointXYZRGB pt) 
        { 
#ifdef __CUDACC__
            return (isnan (pt.x) | isnan (pt.y) | isnan (pt.z)) == 1; 
#else
            return (pcl_isnan (pt.x) | pcl_isnan (pt.y) | pcl_isnan (pt.z)) == 1;
#endif
        }
    };

    /** \brief @b SampleConsensusModel represents the base model class. All sample consensus models must inherit from 
      * this class.
      */
    template <template <typename> class Storage>
    class SampleConsensusModel
    {
      public:
        typedef PointCloudAOS<Storage> PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;

        typedef boost::shared_ptr<SampleConsensusModel> Ptr;
        typedef boost::shared_ptr<const SampleConsensusModel> ConstPtr;

        typedef typename Storage<int>::type Indices;
        typedef boost::shared_ptr<typename Storage<int>::type> IndicesPtr;
        typedef boost::shared_ptr<const typename Storage<int>::type> IndicesConstPtr;

        typedef typename Storage<float>::type Coefficients;
        typedef boost::shared_ptr <Coefficients> CoefficientsPtr;
        typedef boost::shared_ptr <const Coefficients> CoefficientsConstPtr;

        typedef typename Storage<float4>::type Hypotheses;
        //TODO: should be vector<int> instead of int. but currently, only 1point plane model supports this
        typedef typename Storage<int>::type Samples;

      private:
        /** \brief Empty constructor for base SampleConsensusModel. */
        SampleConsensusModel () : radius_min_ (-FLT_MAX), radius_max_ (FLT_MAX) 
        {};

      public:
        /** \brief Constructor for base SampleConsensusModel.
          * \param cloud the input point cloud dataset
          */
        SampleConsensusModel (const PointCloudConstPtr &cloud) : 
          radius_min_ (-FLT_MAX), radius_max_ (FLT_MAX)
        {
          // Sets the input cloud and creates a vector of "fake" indices
          setInputCloud (cloud);
        }

        /* \brief Constructor for base SampleConsensusModel.
         * \param cloud the input point cloud dataset
         * \param indices a vector of point indices to be used from \a cloud
         */
  /*      SampleConsensusModel (const PointCloudConstPtr &cloud, const std::vector<int> &indices) :
                              input_ (cloud),
                              indices_ (boost::make_shared <std::vector<int> > (indices)),
                              radius_min_ (-DBL_MAX), radius_max_ (DBL_MAX) 
      
        {
          if (indices_->size () > input_->points.size ())
          {
            ROS_ERROR ("[pcl::SampleConsensusModel] Invalid index vector given with size %lu while the input PointCloud has size %lu!", (unsigned long) indices_->size (), (unsigned long) input_->points.size ());
            indices_->clear ();
          }
        };*/

        /** \brief Destructor for base SampleConsensusModel. */
        virtual ~SampleConsensusModel () {};

        /** \brief Get a set of random data samples and return them as point
          * indices. Pure virtual.  
          * \param iterations the internal number of iterations used by SAC methods
          * \param samples the resultant model samples, <b>stored on the device</b>
          */
        virtual void 
        getSamples (int &iterations, Indices &samples) = 0;

        /** \brief Check whether the given index samples can form a valid model,
          * compute the model coefficients from these samples and store them
          * in model_coefficients. Pure virtual.
          * \param samples the point indices found as possible good candidates
          * for creating a valid model, <b>stored on the device</b>
          * \param model_coefficients the computed model coefficients
          */
        virtual bool 
        computeModelCoefficients (const Indices &samples, Coefficients &model_coefficients) = 0;

        virtual bool 
        generateModelHypotheses (Hypotheses &h, int max_iterations) = 0;

        virtual bool 
        generateModelHypotheses (Hypotheses &h, Samples &s, int max_iterations) = 0;

        virtual bool 
        isSampleInlier (IndicesPtr &inliers_stencil, Samples &samples, unsigned int &i)
          {return ((*inliers_stencil)[samples[i]] != -1);};

        /* \brief Recompute the model coefficients using the given inlier set
          * and return them to the user. Pure virtual.
          *
          * @note: these are the coefficients of the model after refinement
          * (e.g., after a least-squares optimization)
          *
          * \param inliers the data inliers supporting the model
          * \param model_coefficients the initial guess for the model coefficients
          * \param optimized_coefficients the resultant recomputed coefficients
          * after non-linear optimization
          */
  //      virtual void 
  //      optimizeModelCoefficients (const std::vector<int> &inliers, 
  //                                 const Eigen::VectorXf &model_coefficients,
  //                                 Eigen::VectorXf &optimized_coefficients) = 0;

      /*  \brief Compute all distances from the cloud data to a given model. Pure virtual.
        * \param model_coefficients the coefficients of a model that we need to
        *   compute distances to 
        * \param distances the resultant estimated distances
        */
  //      virtual void 
  //      getDistancesToModel (const Eigen::VectorXf &model_coefficients, 
  //                           std::vector<float> &distances) = 0;

        /** \brief Select all the points which respect the given model
          * coefficients as inliers. Pure virtual.
          * 
          * \param model_coefficients the coefficients of a model that we need to
          * compute distances to
          * \param threshold a maximum admissible distance threshold for
          * determining the inliers from the outliers
          * \param inliers the resultant model inliers
          * \param inliers_stencil
          */
        virtual int
        selectWithinDistance (const Coefficients &model_coefficients, 
                              float threshold,
                              IndicesPtr &inliers, IndicesPtr &inliers_stencil) = 0;
        virtual int
        selectWithinDistance (const Hypotheses &h, int idx,
                              float threshold,
                              IndicesPtr &inliers, IndicesPtr &inliers_stencil) = 0;
        virtual int
        selectWithinDistance (Hypotheses &h, int idx,
                              float threshold,
                              IndicesPtr &inliers_stencil,
                              float3 &centroid) = 0;

        virtual int
        countWithinDistance (const Coefficients &model_coefficients, float threshold) = 0;

        virtual int
        countWithinDistance (const Hypotheses &h, int idx, float threshold) = 0;

        int
        deleteIndices (const IndicesPtr &indices_stencil );
        int
        deleteIndices (const Hypotheses &h, int idx, IndicesPtr &inliers, const IndicesPtr &inliers_delete);

        /*  \brief Create a new point cloud with inliers projected onto the model. Pure virtual.
          * \param inliers the data inliers that we want to project on the model
          * \param model_coefficients the coefficients of a model
          * \param projected_points the resultant projected points
          * \param copy_data_fields set to true (default) if we want the \a
          * projected_points cloud to be an exact copy of the input dataset minus
          * the point projections on the plane model
          */
  //      virtual void 
  //      projectPoints (const std::vector<int> &inliers, 
  //                     const Eigen::VectorXf &model_coefficients,
  //                     PointCloud &projected_points, 
  //                     bool copy_data_fields = true) = 0;

        /*  \brief Verify whether a subset of indices verifies a given set of
          * model coefficients. Pure virtual.
          *
          * \param indices the data indices that need to be tested against the model
          * \param model_coefficients the set of model coefficients
          * \param threshold a maximum admissible distance threshold for
          * determining the inliers from the outliers
          */
  //      virtual bool 
  //      doSamplesVerifyModel (const std::set<int> &indices, 
  //                            const Eigen::VectorXf &model_coefficients, 
  //                            float threshold) = 0;

        /** \brief Provide a pointer to the input dataset
          * \param cloud the const boost shared pointer to a PointCloud message
          */
        virtual void
        setInputCloud (const PointCloudConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset. */
        inline PointCloudConstPtr 
        getInputCloud () const { return (input_); }

        /* \brief Provide a pointer to the vector of indices that represents the input data.
         * \param indices a pointer to the vector of indices that represents the input data.
         */
  //      inline void 
  //      setIndices (const IndicesPtr &indices) { indices_ = indices; }

        /* \brief Provide the vector of indices that represents the input data.
         * \param indices the vector of indices that represents the input data.
         */
  //      inline void 
  //      setIndices (std::vector<int> &indices) 
  //      { 
  //        indices_ = boost::make_shared <std::vector<int> > (indices); 
  //      }

        /** \brief Get a pointer to the vector of indices used. */
        inline IndicesPtr 
        getIndices () const
        {
          if (nr_indices_in_stencil_ != indices_->size())
          {
            typename Indices::iterator last = thrust::remove_copy (indices_stencil_->begin (), indices_stencil_->end (), indices_->begin (), -1);
            indices_->erase (last, indices_->end ());
          }

          return (indices_);
        }

        /* \brief Return an unique id for each type of model employed. */
  //      virtual SacModel 
  //      getModelType () const = 0;

        /* \brief Return the size of a sample from which a model is computed */
  //      inline unsigned int 
  //      getSampleSize () const { return SAC_SAMPLE_SIZE.at (getModelType ()); }

        /** \brief Set the minimum and maximum allowable radius limits for the
          * model (applicable to models that estimate a radius)
          * \param min_radius the minimum radius model
          * \param max_radius the maximum radius model
          * \todo change this to set limits on the entire model
          */
        inline void
        setRadiusLimits (float min_radius, float max_radius)
        {
          radius_min_ = min_radius;
          radius_max_ = max_radius;
        }

        /** \brief Get the minimum and maximum allowable radius limits for the
          * model as set by the user.
          *
          * \param min_radius the resultant minimum radius model
          * \param max_radius the resultant maximum radius model
          */
        inline void
        getRadiusLimits (float &min_radius, float &max_radius)
        {
          min_radius = radius_min_;
          max_radius = radius_max_;
        }

  //      friend class ProgressiveSampleConsensus<PointT>;

        inline boost::shared_ptr<typename Storage<float4>::type>
        getNormals () { return (normals_); }

        inline
          void setNormals (boost::shared_ptr<typename Storage<float4>::type> normals) { normals_ = normals; }


      protected:
        /*  \brief Check whether a model is valid given the user constraints.
          * \param model_coefficients the set of model coefficients
          */
  //      virtual inline bool
  //      isModelValid (const Eigen::VectorXf &model_coefficients) = 0;

        /** \brief A boost shared pointer to the point cloud data array. */
        PointCloudConstPtr input_;
        boost::shared_ptr<typename Storage<float4>::type> normals_;

        /** \brief A pointer to the vector of point indices to use. */
        IndicesPtr indices_;
        /** \brief A pointer to the vector of point indices (stencil) to use. */
        IndicesPtr indices_stencil_;
        /** \brief number of indices left in indices_stencil_ */
        unsigned int nr_indices_in_stencil_;

        /** \brief The minimum and maximum radius limits for the model.
          * Applicable to all models that estimate a radius. 
          */
        float radius_min_, radius_max_;

        /** \brief Linear-Congruent random number generator engine. */
        thrust::minstd_rand rngl_;
    };

    /*  \brief @b SampleConsensusModelFromNormals represents the base model class
      * for models that require the use of surface normals for estimation.
      */
  //  template <typename PointT, typename PointNT>
  //  class SampleConsensusModelFromNormals
  //  {
  //    public:
  //      typedef typename pcl::PointCloud<PointNT>::ConstPtr PointCloudNConstPtr;
  //      typedef typename pcl::PointCloud<PointNT>::Ptr PointCloudNPtr;
  //
  //      typedef boost::shared_ptr<SampleConsensusModelFromNormals> Ptr;
  //      typedef boost::shared_ptr<const SampleConsensusModelFromNormals> ConstPtr;
  //
  //      /* \brief Empty constructor for base SampleConsensusModelFromNormals. */
  //      SampleConsensusModelFromNormals () : normal_distance_weight_ (0.0) {};
  //
  //      /*  \brief Set the normal angular distance weight.
  //        * \param w the relative weight (between 0 and 1) to give to the angular
  //        * distance (0 to pi/2) between point normals and the plane normal.
  //        * (The Euclidean distance will have weight 1-w.)
  //        */
  //      inline void 
  //      setNormalDistanceWeight (float w) { normal_distance_weight_ = w; }
  //
  //      /* \brief Get the normal angular distance weight. */
  //      inline float 
  //      getNormalDistanceWeight () { return (normal_distance_weight_); }
  //
  //      /* \brief Provide a pointer to the input dataset that contains the point
  //        * normals of the XYZ dataset.
  //        *
  //        * \param normals the const boost shared pointer to a PointCloud message
  //        */
  //      inline void 
  //      setInputNormals (const PointCloudNConstPtr &normals) { normals_ = normals; }
  //
  //      /* \brief Get a pointer to the normals of the input XYZ point cloud dataset. */
  //      inline PointCloudNConstPtr 
  //      getInputNormals () { return (normals_); }
  //
  //    protected:
  //      /* \brief The relative weight (between 0 and 1) to give to the angular
  //        * distance (0 to pi/2) between point normals and the plane normal. 
  //        */
  //      float normal_distance_weight_;
  //
  //      /* \brief A pointer to the input dataset that contains the point normals
  //        * of the XYZ dataset. 
  //        */
  //      PointCloudNConstPtr normals_;
  //  };
  } // namespace_
} // namespace_

#endif  //#ifndef PCL_CUDA_SAMPLE_CONSENSUS_MODEL_H_
