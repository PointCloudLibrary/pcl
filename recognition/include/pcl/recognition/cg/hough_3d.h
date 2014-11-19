/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id:$
 *
 */

#ifndef PCL_RECOGNITION_HOUGH_3D_H_
#define PCL_RECOGNITION_HOUGH_3D_H_

#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/boost.h>

namespace pcl
{
  namespace recognition
  {
    /** \brief HoughSpace3D is a 3D voting space. Cast votes can be interpolated in order to better deal with approximations introduced by bin quantization. A weight can also be associated with each vote. 
      * \author Federico Tombari (original), Tommaso Cavallari (PCL port)
      * \ingroup recognition
      */
    class PCL_EXPORTS HoughSpace3D
    {

      public:
      
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
        /** \brief Constructor
          *
          * \param[in] min_coord minimum (x,y,z) coordinates of the Hough space 
          * \param[in] bin_size  size of each bing of the Hough space.
          * \param[in] max_coord maximum (x,y,z) coordinates of the Hough space.
          */
        HoughSpace3D (const Eigen::Vector3d &min_coord, const Eigen::Vector3d &bin_size, const Eigen::Vector3d &max_coord);

        /** \brief Reset all cast votes. */
        void
        reset ();

        /** \brief Casting a vote for a given position in the Hough space.
          * 
          * \param[in] single_vote_coord coordinates of the vote being cast (in absolute coordinates)
          * \param[in] weight weight associated with the vote.
          * \param[in] voter_id the numeric id of the voter. Useful to trace back the voting correspondence, if the vote is returned by findMaxima as part of a maximum of the Hough Space.
          * \return the index of the bin in which the vote has been cast.
          */
        int
        vote (const Eigen::Vector3d &single_vote_coord, double weight, int voter_id);

        /** \brief Vote for a given position in the 3D space. The weight is interpolated between the bin pointed by single_vote_coord and its neighbors.
          * 
          * \param[in] single_vote_coord coordinates of the vote being cast.
          * \param[in] weight weight associated with the vote.
          * \param[in] voter_id the numeric id of the voter. Useful to trace back the voting correspondence, if the vote is returned by findMaxima as a part of a maximum of the Hough Space.
          * \return the index of the bin in which the vote has been cast.
          */
        int
        voteInt (const Eigen::Vector3d &single_vote_coord, double weight, int voter_id);

        /** \brief Find the bins with most votes.
          * 
          * \param[in] min_threshold the minimum number of votes to be included in a bin in order to have its value returned. 
          * If set to a value between -1 and 0 the Hough space maximum_vote is found and the returned values are all the votes greater than -min_threshold * maximum_vote.
          * \param[out] maxima_values the list of Hough Space bin values greater than min_threshold.
          * \param[out] maxima_voter_ids for each value returned, a list of the voter ids who cast a vote in that position. 
          * \return The min_threshold used, either set by the user or found by this method.
          */
        double
        findMaxima (double min_threshold, std::vector<double> & maxima_values, std::vector<std::vector<int> > &maxima_voter_ids);

      protected:

        /** \brief Minimum coordinate in the Hough Space. */
        Eigen::Vector3d min_coord_;

        /** \brief Size of each bin in the Hough Space. */
        Eigen::Vector3d bin_size_;

        /** \brief Number of bins for each dimension. */
        Eigen::Vector3i bin_count_;

        /** \brief Used to access hough_space_ as if it was a matrix. */
        int partial_bin_products_[4];

        /** \brief Total number of bins in the Hough Space. */
        int total_bins_count_;

        /** \brief The Hough Space. */
        std::vector<double> hough_space_;
        //boost::unordered_map<int, double> hough_space_;

        /** \brief List of voters for each bin. */
        boost::unordered_map<int, std::vector<int> > voter_ids_;
    };
  }

  /** \brief Class implementing a 3D correspondence grouping algorithm that can deal with multiple instances of a model template
    * found into a given scene. Each correspondence casts a vote for a reference point in a 3D Hough Space.
	* The remaining 3 DOF are taken into account by associating each correspondence with a local Reference Frame. 
    * The suggested PointModelRfT is pcl::ReferenceFrame
    * 
    * \note If you use this code in any academic work, please cite the original paper:
    *   - F. Tombari, L. Di Stefano:
    *     Object recognition in 3D scenes with occlusions and clutter by Hough voting.
    *     2010, Fourth Pacific-Rim Symposium on Image and Video Technology
    *
    * \author Federico Tombari (original), Tommaso Cavallari (PCL port)
    * \ingroup recognition
    */
  template<typename PointModelT, typename PointSceneT, typename PointModelRfT = pcl::ReferenceFrame, typename PointSceneRfT = pcl::ReferenceFrame>
  class Hough3DGrouping : public CorrespondenceGrouping<PointModelT, PointSceneT>
  {
    public:
      typedef pcl::PointCloud<PointModelRfT> ModelRfCloud;
      typedef typename ModelRfCloud::Ptr ModelRfCloudPtr;
      typedef typename ModelRfCloud::ConstPtr ModelRfCloudConstPtr;

      typedef pcl::PointCloud<PointSceneRfT> SceneRfCloud;
      typedef typename SceneRfCloud::Ptr SceneRfCloudPtr;
      typedef typename SceneRfCloud::ConstPtr SceneRfCloudConstPtr;

      typedef pcl::PointCloud<PointModelT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef typename pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::SceneCloudConstPtr SceneCloudConstPtr;

      /** \brief Constructor */
      Hough3DGrouping () 
        : input_rf_ ()
        , scene_rf_ ()
        , needs_training_ (true)
        , model_votes_ ()
        , hough_threshold_ (-1)
        , hough_bin_size_ (1.0)
        , use_interpolation_ (true)
        , use_distance_weight_ (false)
        , local_rf_normals_search_radius_ (0.0f)
        , local_rf_search_radius_ (0.0f)
        , hough_space_ ()
        , found_transformations_ ()
        , hough_space_initialized_ (false)
      {}

      /** \brief Provide a pointer to the input dataset.
        * \param[in] cloud the const boost shared pointer to a PointCloud message.
        */
      inline void
      setInputCloud (const PointCloudConstPtr &cloud)
      {
        PCLBase<PointModelT>::setInputCloud (cloud);
        needs_training_ = true;
        hough_space_initialized_ = false;
        input_rf_.reset();
      }

      /** \brief Provide a pointer to the input dataset's reference frames. 
        * Each point in the reference frame cloud should be the reference frame of
        * the correspondent point in the input dataset.
        * 
        * \param[in] input_rf the pointer to the input cloud's reference frames.
        */
      inline void
      setInputRf (const ModelRfCloudConstPtr &input_rf)
      {
        input_rf_ = input_rf;
        needs_training_ = true;
        hough_space_initialized_ = false;
      }

      /** \brief Getter for the input dataset's reference frames. 
        * Each point in the reference frame cloud should be the reference frame of
        * the correspondent point in the input dataset.
        * 
        * \return the pointer to the input cloud's reference frames.
        */
      inline ModelRfCloudConstPtr
      getInputRf () const
      {
        return (input_rf_);
      }
      
      /** \brief Provide a pointer to the scene dataset (i.e. the cloud in which the algorithm has to search for instances of the input model)
        * 
        * \param[in] scene the const boost shared pointer to a PointCloud message.
        */
      inline void
      setSceneCloud (const SceneCloudConstPtr &scene)
      {
        scene_ = scene;
        hough_space_initialized_ = false;
        scene_rf_.reset();
      }

      /** \brief Provide a pointer to the scene dataset's reference frames. 
        * Each point in the reference frame cloud should be the reference frame of
        * the correspondent point in the scene dataset.
        * 
        * \param[in] scene_rf the pointer to the scene cloud's reference frames.
        */
      inline void
      setSceneRf (const SceneRfCloudConstPtr &scene_rf)
      {
        scene_rf_ = scene_rf;
        hough_space_initialized_ = false;
      }

      /** \brief Getter for the scene dataset's reference frames. 
        * Each point in the reference frame cloud should be the reference frame of
        * the correspondent point in the scene dataset.
        * 
        * \return the pointer to the scene cloud's reference frames.
        */
      inline SceneRfCloudConstPtr
      getSceneRf () const
      {
        return (scene_rf_);
      }

      /** \brief Provide a pointer to the precomputed correspondences between points in the input dataset and 
        * points in the scene dataset. The correspondences are going to be clustered into different model instances
        * by the algorithm.
        * 
        * \param[in] corrs the correspondences between the model and the scene.
        */
      inline void
      setModelSceneCorrespondences (const CorrespondencesConstPtr &corrs)
      {
        model_scene_corrs_ = corrs;
        hough_space_initialized_ = false;
      }

      /** \brief Sets the minimum number of votes in the Hough space needed to infer the presence of a model instance into the scene cloud.
        * 
        * \param[in] threshold the threshold for the Hough space voting, if set between -1 and 0 the maximum vote in the
        * entire space is automatically calculated and -threshold the maximum value is used as a threshold. This means
        * that a value between -1 and 0 should be used only if at least one instance of the model is always present in
        * the scene, or if this false positive can be filtered later.
        */
      inline void
      setHoughThreshold (double threshold)
      {
        hough_threshold_ = threshold;
      }

      /** \brief Gets the minimum number of votes in the Hough space needed to infer the presence of a model instance into the scene cloud.
        * 
        * \return the threshold for the Hough space voting.
        */
      inline double
      getHoughThreshold () const
      {
        return (hough_threshold_);
      }

      /** \brief Sets the size of each bin into the Hough space.
        * 
        * \param[in] bin_size the size of each Hough space's bin.
        */
      inline void
      setHoughBinSize (double bin_size)
      {
        hough_bin_size_ = bin_size;
        hough_space_initialized_ = false;
      }

      /** \brief Gets the size of each bin into the Hough space.
        * 
        * \return the size of each Hough space's bin.
        */
      inline double
      getHoughBinSize () const
      {
        return (hough_bin_size_);
      }

      /** \brief Sets whether the vote casting procedure interpolates
        * the score between neighboring bins of the Hough space or not.
        * 
        * \param[in] use_interpolation the algorithm should interpolate the vote score between neighboring bins.
        */
      inline void
      setUseInterpolation (bool use_interpolation)
      {
        use_interpolation_ = use_interpolation;
        hough_space_initialized_ = false;
      }

      /** \brief Gets whether the vote casting procedure interpolates
        * the score between neighboring bins of the Hough space or not.
        * 
        * \return if the algorithm should interpolate the vote score between neighboring bins.
        */
      inline bool
      getUseInterpolation () const
      {
        return (use_interpolation_);
      }

      /** \brief Sets whether the vote casting procedure uses the correspondence's distance as a score.
        * 
        * \param[in] use_distance_weight the algorithm should use the weighted distance when calculating the Hough voting score.
        */
      inline void
      setUseDistanceWeight (bool use_distance_weight)
      {
        use_distance_weight_ = use_distance_weight;
        hough_space_initialized_ = false;
      }

      /** \brief Gets whether the vote casting procedure uses the correspondence's distance as a score.
        * 
        * \return if the algorithm should use the weighted distance when calculating the Hough voting score.
        */
      inline bool
      getUseDistanceWeight () const
      {
        return (use_distance_weight_);
      }	

      /** \brief If the Local reference frame has not been set for either the model cloud or the scene cloud,
        * this algorithm makes the computation itself but needs a suitable search radius to compute the normals
        * in order to subsequently compute the RF (if not set a default 15 nearest neighbors search is performed).
        *
        * \param[in] local_rf_normals_search_radius the normals search radius for the local reference frame calculation.
        */
      inline void
      setLocalRfNormalsSearchRadius (float local_rf_normals_search_radius)
      {
        local_rf_normals_search_radius_ = local_rf_normals_search_radius;
        needs_training_ = true;
        hough_space_initialized_ = false;
      }

      /** \brief If the Local reference frame has not been set for either the model cloud or the scene cloud,
        * this algorithm makes the computation itself but needs a suitable search radius to compute the normals
        * in order to subsequently compute the RF (if not set a default 15 nearest neighbors search is performed).
        *
        * \return the normals search radius for the local reference frame calculation.
        */
      inline float
      getLocalRfNormalsSearchRadius () const
      {
        return (local_rf_normals_search_radius_);
      }

      /** \brief If the Local reference frame has not been set for either the model cloud or the scene cloud,
        * this algorithm makes the computation itself but needs a suitable search radius to do so.
        * \attention This parameter NEEDS to be set if the reference frames are not precomputed externally, 
        * otherwise the recognition results won't be correct.
        *
        * \param[in] local_rf_search_radius the search radius for the local reference frame calculation.
        */
      inline void
      setLocalRfSearchRadius (float local_rf_search_radius)
      {
        local_rf_search_radius_ = local_rf_search_radius;
        needs_training_ = true;
        hough_space_initialized_ = false;
      }

      /** \brief If the Local reference frame has not been set for either the model cloud or the scene cloud,
        * this algorithm makes the computation itself but needs a suitable search radius to do so.
        * \attention This parameter NEEDS to be set if the reference frames are not precomputed externally, 
        * otherwise the recognition results won't be correct.
        *
        * \return the search radius for the local reference frame calculation.
        */
      inline float
      getLocalRfSearchRadius () const
      {
        return (local_rf_search_radius_);
      }

      /** \brief Call this function after setting the input, the input_rf and the hough_bin_size parameters to perform an off line training of the algorithm. This might be useful if one wants to perform once and for all a pre-computation of votes that only concern the models, increasing the on-line efficiency of the grouping algorithm. 
        * The algorithm is automatically trained on the first invocation of the recognize method or the cluster method if this training function has not been manually invoked.
        * 
        * \return true if the training had been successful or false if errors have occurred.
        */
      bool
      train ();

      /** \brief The main function, recognizes instances of the model into the scene set by the user.
        * 
        * \param[out] transformations a vector containing one transformation matrix for each instance of the model recognized into the scene.
        *
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      bool
      recognize (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations);

      /** \brief The main function, recognizes instances of the model into the scene set by the user.
        * 
        * \param[out] transformations a vector containing one transformation matrix for each instance of the model recognized into the scene.
        * \param[out] clustered_corrs a vector containing the correspondences for each instance of the model found within the input data (the same output of clusterCorrespondences).
        *
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      bool
      recognize (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, std::vector<pcl::Correspondences> &clustered_corrs);

    protected:
      using CorrespondenceGrouping<PointModelT, PointSceneT>::input_;
      using CorrespondenceGrouping<PointModelT, PointSceneT>::scene_;
      using CorrespondenceGrouping<PointModelT, PointSceneT>::model_scene_corrs_;

      /** \brief The input Rf cloud. */
      ModelRfCloudConstPtr input_rf_;

      /** \brief The scene Rf cloud. */
      SceneRfCloudConstPtr scene_rf_;

      /** \brief If the training of the Hough space is needed; set on change of either the input cloud or the input_rf. */
      bool needs_training_;

      /** \brief The result of the training. The vector between each model point and the centroid of the model adjusted by its local reference frame.*/
      std::vector<Eigen::Vector3f> model_votes_;

      /** \brief The minimum number of votes in the Hough space needed to infer the presence of a model instance into the scene cloud. */
      double hough_threshold_;

      /** \brief The size of each bin of the hough space. */
      double hough_bin_size_;

      /** \brief Use the interpolation between neighboring Hough bins when casting votes. */
      bool use_interpolation_;

      /** \brief Use the weighted correspondence distance when casting votes. */
      bool use_distance_weight_;

      /** \brief Normals search radius for the potential Rf calculation. */
      float local_rf_normals_search_radius_;

      /** \brief Search radius for the potential Rf calculation. */
      float local_rf_search_radius_;

      /** \brief The Hough space. */
      boost::shared_ptr<pcl::recognition::HoughSpace3D> hough_space_;

      /** \brief Transformations found by clusterCorrespondences method. */
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > found_transformations_;

      /** \brief Whether the Hough space already contains the correct votes for the current input parameters and so the cluster and recognize calls don't need to recompute each value.
        * Reset on the change of any parameter except the hough_threshold.
        */
      bool hough_space_initialized_;

      /** \brief Cluster the input correspondences in order to distinguish between different instances of the model into the scene.
        * 
        * \param[out] model_instances a vector containing the clustered correspondences for each model found on the scene.
        * \return true if the clustering had been successful or false if errors have occurred.
        */ 
      void
      clusterCorrespondences (std::vector<Correspondences> &model_instances);

      /*  \brief Finds the transformation matrix between the input and the scene cloud for a set of correspondences using a RANSAC algorithm.
        * \param[in] the scene cloud in which the PointSceneT has been converted to PointModelT.
        * \param[in] corrs a set of correspondences.
        * \param[out] transform the transformation matrix between the input cloud and the scene cloud that aligns the found correspondences.
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      //bool
      //getTransformMatrix (const PointCloudConstPtr &scene_cloud, const Correspondences &corrs, Eigen::Matrix4f &transform);

      /** \brief The Hough space voting procedure.
        * \return true if the voting had been successful or false if errors have occurred.
        */
      bool
      houghVoting ();

      /** \brief Computes the reference frame for an input cloud.
        * \param[in] input the input cloud.
        * \param[out] rf the resulting reference frame.
        */
      template<typename PointType, typename PointRfType> void
      computeRf (const boost::shared_ptr<const pcl::PointCloud<PointType> > &input, pcl::PointCloud<PointRfType> &rf);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/recognition/impl/cg/hough_3d.hpp>
#endif

#endif // PCL_RECOGNITION_HOUGH_3D_H_
