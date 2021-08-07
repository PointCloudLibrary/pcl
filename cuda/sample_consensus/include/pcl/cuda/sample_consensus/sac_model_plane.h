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

#pragma once

#include <pcl/cuda/sample_consensus/sac_model.h>

#include <thrust/random.h>

namespace pcl
{
  namespace cuda
  {
    /** \brief Check if a certain tuple is a point inlier. */
    struct CountPlanarInlier
    {
      float4 coefficients;
      float threshold;

      CountPlanarInlier (float4 coeff, float thresh) : 
        coefficients(coeff), threshold(thresh) 
      {}

      template <typename Tuple> __inline__ __host__ __device__ bool
      operator () (const Tuple &t);
    };

    /** \brief Check if a certain tuple is a point inlier. */
    struct CheckPlanarInlier
    {
      float4 coefficients;
      float threshold;

      CheckPlanarInlier (float4 coeff, float thresh) : 
        coefficients(coeff), threshold(thresh) 
      {}

      template <typename Tuple> __inline__ __host__ __device__ int
      operator () (const Tuple &t);
    };

    ////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b SampleConsensusModelPlane defines a model for 3D plane segmentation.
      */
    template <template <typename> class Storage>
    class SampleConsensusModelPlane : public SampleConsensusModel<Storage>
    {
      public:
        using SampleConsensusModel<Storage>::input_;
        using SampleConsensusModel<Storage>::indices_;
        using SampleConsensusModel<Storage>::rngl_;

        using PointCloud = typename SampleConsensusModel<Storage>::PointCloud;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        using Indices = typename SampleConsensusModel<Storage>::Indices;
        using IndicesPtr = typename SampleConsensusModel<Storage>::IndicesPtr;
        using IndicesConstPtr = typename SampleConsensusModel<Storage>::IndicesConstPtr;

        using Coefficients = typename SampleConsensusModel<Storage>::Coefficients;
        using Hypotheses = typename SampleConsensusModel<Storage>::Hypotheses;
        using Samples = typename SampleConsensusModel<Storage>::Samples;

        using Ptr = shared_ptr<SampleConsensusModelPlane>;
        using ConstPtr = shared_ptr<const SampleConsensusModelPlane>;

        /** \brief Constructor for base SampleConsensusModelPlane.
          * \param cloud the input point cloud dataset
          */
        SampleConsensusModelPlane (const PointCloudConstPtr &cloud);

        /*  \brief Constructor for base SampleConsensusModelPlane.
          * \param cloud the input point cloud dataset
          * \param indices a vector of point indices to be used from \a cloud
          */
  //      SampleConsensusModelPlane (const PointCloudConstPtr &cloud, const std::vector<int> &indices) : SampleConsensusModel<PointT> (cloud, indices) {};

        /** \brief Get 3 random non-collinear points as data samples and return them as point indices.
          * \param iterations the internal number of iterations used by SAC methods
          * \param samples the resultant model samples
          * \note assumes unique points!
          */
        void 
        getSamples (int &iterations, Indices &samples);

        /** \brief Check whether the given index samples can form a valid plane model, compute the model coefficients from
          * these samples and store them in model_coefficients. The plane coefficients are:
          * a, b, c, d (ax+by+cz+d=0)
          * \param samples the point indices found as possible good candidates for creating a valid model
          * \param model_coefficients the resultant model coefficients
          */
        bool 
        computeModelCoefficients (const Indices &samples, Coefficients &model_coefficients);

        bool 
        generateModelHypotheses (Hypotheses &h, int max_iterations);

        virtual bool 
        generateModelHypotheses (Hypotheses &h, Samples &s, int max_iterations)
        {
          // TODO: hack.. Samples should be std::vector<int>, not int..
          return generateModelHypotheses (h, max_iterations);
        };

          /*  \brief Compute all distances from the cloud data to a given plane model.
            * \param model_coefficients the coefficients of a plane model that we need to compute distances to
            * \param distances the resultant estimated distances
            */
  //      void 
  //      getDistancesToModel (const Eigen::VectorXf &model_coefficients, std::vector<float> &distances);

        /** \brief Select all the points which respect the given model coefficients as inliers.
          * \param model_coefficients the coefficients of a plane model that we need to 
          * compute distances to
          * \param threshold a maximum admissible distance threshold for determining the 
          * inliers from the outliers
          * \param inliers the resultant model inliers
          * \param inliers_stencil
          */
        int 
        selectWithinDistance (const Coefficients &model_coefficients, 
                              float threshold, IndicesPtr &inliers, IndicesPtr &inliers_stencil);
        int
        selectWithinDistance (const Hypotheses &h, int idx,
                              float threshold,
                              IndicesPtr &inliers, IndicesPtr &inliers_stencil);
        int
        selectWithinDistance (Hypotheses &h, int idx,
                              float threshold,
                              IndicesPtr &inliers_stencil,
                              float3 &centroid);

        int
        countWithinDistance (const Coefficients &model_coefficients, float threshold);

        int
        countWithinDistance (const Hypotheses &h, int idx, float threshold);

          /*  \brief Recompute the plane coefficients using the given inlier set and return them to the user.
            * @note: these are the coefficients of the plane model after refinement (eg. after SVD)
            * \param inliers the data inliers found as supporting the model
            * \param model_coefficients the initial guess for the model coefficients
            * \param optimized_coefficients the resultant recomputed coefficients after non-linear optimization
            */
  //      void 
  //      optimizeModelCoefficients (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients);

          /*  \brief Create a new point cloud with inliers projected onto the plane model.
            * \param inliers the data inliers that we want to project on the plane model
            * \param model_coefficients the *normalized* coefficients of a plane model
            * \param projected_points the resultant projected points
            * \param copy_data_fields set to true if we need to copy the other data fields
            */
  //      void 
  //      projectPoints (const std::vector<int> &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields = true);

          /*  \brief Verify whether a subset of indices verifies the given plane model coefficients.
            * \param indices the data indices that need to be tested against the plane model
            * \param model_coefficients the plane model coefficients
            * \param threshold a maximum admissible distance threshold for determining the inliers from the outliers
            */
  //      bool 
  //      doSamplesVerifyModel (const std::set<int> &indices, const Eigen::VectorXf &model_coefficients, float threshold);

          /*  \brief Return an unique id for this model (SACMODEL_PLANE). */
  //      inline pcl::SacModel getModelType () const { return (SACMODEL_PLANE); }

  //    protected:
         /*  \brief Check whether a model is valid given the user constraints.
           * \param model_coefficients the set of model coefficients
           */
  //      inline bool 
  //      isModelValid (const Eigen::VectorXf &model_coefficients)
  //      {
  //        // Needs a valid model coefficients
  //        if (model_coefficients.size () != 4)
  //        {
  //          ROS_ERROR ("[pcl::SampleConsensusModelPlane::isModelValid] Invalid number of model coefficients given (%lu)!", (unsigned long) model_coefficients.size ());
  //          return (false);
  //        }
  //        return (true);
  //      }

  //    private:
        /* \brief Define the maximum number of iterations for collinearity checks */
        const static int MAX_ITERATIONS_COLLINEAR = 1000;
    };

    /** \brief Check if a certain tuple is a point inlier. */
    template <template <typename> class Storage>
    struct CreatePlaneHypothesis
    {
      using PointCloud = typename SampleConsensusModel<Storage>::PointCloud;
      using PointCloudConstPtr = typename PointCloud::ConstPtr;

      using Indices = typename SampleConsensusModel<Storage>::Indices;
      using IndicesPtr = typename SampleConsensusModel<Storage>::IndicesPtr;
      using IndicesConstPtr = typename SampleConsensusModel<Storage>::IndicesConstPtr;

      const PointXYZRGB *input;
      const int *indices;
      int nr_indices;
      float bad_value;

      CreatePlaneHypothesis (const PointXYZRGB *_input, const int *_indices, int _nr_indices, float bad) : 
        input(_input), indices(_indices), nr_indices(_nr_indices), bad_value(bad)
      {}

      //template <typename Tuple> 
      __inline__ __host__ __device__ float4
      //operator () (const Tuple &t);
      operator () (int t);
    };


    struct parallel_random_generator 
    { 
      
      __inline__ __host__ __device__ 
      parallel_random_generator(unsigned int seed) 
      { 
        m_seed = seed; 
      } 

      __inline__ __host__ __device__ 
      unsigned int operator()(const unsigned int n) const 
      { 
        thrust::default_random_engine rng(m_seed); 
        // discard n numbers to avoid correlation 
        rng.discard(n); 
        // return a random number 
        return rng(); 
      } 
      unsigned int m_seed; 
    }; 

  } // namespace
} // namespace
