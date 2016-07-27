/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Copyright (C) 2008 Ben Gurion University of the Negev, Beer Sheva, Israel.
 *
 *  All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met
 *
 *   * The use for research only (no for any commercial application).
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

#ifndef PCL_REGISTRATION_IA_FPCS_H_
#define PCL_REGISTRATION_IA_FPCS_H_

#include <pcl/common/common.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/matching_candidate.h>

namespace pcl
{
  /** \brief Compute the mean point density of a given point cloud.
    * \param[in] cloud pointer to the input point cloud
    * \param[in] max_dist maximum distance of a point to be considered as a neighbor
    * \param[in] nr_threads number of threads to use (default = 1, only used if OpenMP flag is set)
    * \return the mean point density of a given point cloud
    */
  template <typename PointT> inline float
  getMeanPointDensity (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, float max_dist, int nr_threads = 1);

  /** \brief Compute the mean point density of a given point cloud.
    * \param[in] cloud pointer to the input point cloud
    * \param[in] indices the vector of point indices to use from \a cloud
    * \param[in] max_dist maximum distance of a point to be considered as a neighbor
    * \param[in] nr_threads number of threads to use (default = 1, only used if OpenMP flag is set)
    * \return the mean point density of a given point cloud
    */
  template <typename PointT> inline float
  getMeanPointDensity (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector <int> &indices,
    float max_dist, int nr_threads = 1);
  
  
  namespace registration
  {
    /** \brief FPCSInitialAlignment computes corresponding four point congruent sets as described in:
    * "4-points congruent sets for robust pairwise surface registration", Dror Aiger, Niloy Mitra, Daniel Cohen-Or.
    * ACM Transactions on Graphics, vol. 27(3), 2008
    * \author P.W.Theiler
    * \ingroup registration
    */
    template <typename PointSource, typename PointTarget, typename NormalT = pcl::Normal, typename Scalar = float>
    class FPCSInitialAlignment : public Registration <PointSource, PointTarget, Scalar>
    {
    public:
      /** \cond */
      typedef boost::shared_ptr <FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar> > Ptr;
      typedef boost::shared_ptr <const FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar> > ConstPtr;

      typedef pcl::search::KdTree<PointSource> KdTreeReciprocal;
      typedef typename KdTreeReciprocal::Ptr KdTreeReciprocalPtr;

      typedef pcl::PointCloud <PointTarget> PointCloudTarget;
      typedef pcl::PointCloud <PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::iterator PointCloudSourceIterator;      

      typedef pcl::PointCloud <NormalT> Normals;
      typedef typename Normals::ConstPtr NormalsConstPtr;

      typedef pcl::registration::MatchingCandidate MatchingCandidate;
      typedef pcl::registration::MatchingCandidates MatchingCandidates;
      /** \endcond */


      /** \brief Constructor.
        * Resets the maximum number of iterations to 0 thus forcing an internal computation if not set by the user.
        * Sets the number of RANSAC iterations to 1000 and the standard transformation estimation to TransformationEstimation3Point.
        */
      FPCSInitialAlignment ();

      /** \brief Destructor. */
      virtual ~FPCSInitialAlignment ()
      {};


      /** \brief Provide a pointer to the vector of target indices.
        * \param[in] target_indices a pointer to the target indices
        */
      inline void
      setTargetIndices (const IndicesPtr &target_indices)
      {
        target_indices_ = target_indices;
      };

      /** \return a pointer to the vector of target indices. */
      inline IndicesPtr
      getTargetIndices () const
      {
        return (target_indices_);
      };


      /** \brief Provide a pointer to the normals of the source point cloud.
        * \param[in] source_normals pointer to the normals of the source pointer cloud.
        */
      inline void
      setSourceNormals (const NormalsConstPtr &source_normals)
      {
        source_normals_ = source_normals;
      };

      /** \return the normals of the source point cloud. */
      inline NormalsConstPtr
      getSourceNormals () const
      {
        return (source_normals_);
      };


      /** \brief Provide a pointer to the normals of the target point cloud.
        * \param[in] target_normals point to the normals of the target point cloud.
        */
      inline void
      setTargetNormals (const NormalsConstPtr &target_normals)
      {
        target_normals_ = target_normals;
      };

      /** \return the normals of the target point cloud. */
      inline NormalsConstPtr
      getTargetNormals () const
      {
        return (target_normals_);
      };


      /** \brief Set the number of used threads if OpenMP is activated.
        * \param[in] nr_threads the number of used threads
        */
      inline void
      setNumberOfThreads (int nr_threads)
      {
        nr_threads_ = nr_threads;
      };

      /** \return the number of threads used if OpenMP is activated. */
      inline int
      getNumberOfThreads () const
      {
        return (nr_threads_);
      };


      /** \brief Set the constant factor delta which weights the internally calculated parameters.
        * \param[in] delta the weight factor delta
        * \param[in] normalize flag if delta should be normalized according to point cloud density
        */
      inline void
      setDelta (float delta, bool normalize = false)
      {
        delta_ = delta;
        normalize_delta_ = normalize;
      };

      /** \return the constant factor delta which weights the internally calculated parameters. */
      inline float
      getDelta () const
      {
        return (delta_);
      };


      /** \brief Set the approximate overlap between source and target.
        * \param[in] approx_overlap the estimated overlap
        */
      inline void
      setApproxOverlap (float approx_overlap)
      {
        approx_overlap_ = approx_overlap;
      };

      /** \return the approximated overlap between source and target. */
      inline float
      getApproxOverlap () const
      {
        return (approx_overlap_);
      };


      /** \brief Set the scoring threshold used for early finishing the method.
        * \param[in] score_threshold early terminating score criteria
        */
      inline void
      setScoreThreshold (float score_threshold)
      {
        score_threshold_ = score_threshold;
      };

      /** \return the scoring threshold used for early finishing the method. */
      inline float
      getScoreThreshold () const
      {
        return (score_threshold_);
      };


      /** \brief Set the number of source samples to use during alignment.
        * \param[in] nr_samples the number of source samples
        */
      inline void
      setNumberOfSamples (int nr_samples)
      {
        nr_samples_ = nr_samples;
      };

      /** \return the number of source samples to use during alignment. */
      inline int
      getNumberOfSamples () const
      {
        return (nr_samples_);
      };


      /** \brief Set the maximum normal difference between valid point correspondences in degree.
        * \param[in] max_norm_diff the maximum difference in degree
        */
      inline void
      setMaxNormalDifference (float max_norm_diff)
      {
        max_norm_diff_ = max_norm_diff;
      };

      /** \return the maximum normal difference between valid point correspondences in degree. */
      inline float
      getMaxNormalDifference () const
      {
        return (max_norm_diff_);
      };


      /** \brief Set the maximum computation time in seconds.
        * \param[in] max_runtime the maximum runtime of the method in seconds
        */
      inline void
      setMaxComputationTime (int max_runtime)
      {
        max_runtime_ = max_runtime;
      };

      /** \return the maximum computation time in seconds. */
      inline int
      getMaxComputationTime () const
      {
        return (max_runtime_);
      };


      /** \return the fitness score of the best scored four-point match. */
      inline float
      getFitnessScore () const
      {
        return (fitness_score_);
      };

    protected:

      using PCLBase <PointSource>::deinitCompute;
      using PCLBase <PointSource>::input_;
      using PCLBase <PointSource>::indices_;

      using Registration <PointSource, PointTarget, Scalar>::reg_name_;
      using Registration <PointSource, PointTarget, Scalar>::target_;
      using Registration <PointSource, PointTarget, Scalar>::tree_;
      using Registration <PointSource, PointTarget, Scalar>::correspondences_;
      using Registration <PointSource, PointTarget, Scalar>::target_cloud_updated_;
      using Registration <PointSource, PointTarget, Scalar>::final_transformation_;
      using Registration <PointSource, PointTarget, Scalar>::max_iterations_;
      using Registration <PointSource, PointTarget, Scalar>::ransac_iterations_;
      using Registration <PointSource, PointTarget, Scalar>::transformation_estimation_;
      using Registration <PointSource, PointTarget, Scalar>::converged_;


      /** \brief Rigid transformation computation method.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess The computed transforamtion
        */
      virtual void
      computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess);


      /** \brief Internal computation initialization. */
      virtual bool
      initCompute ();

      /** \brief Select an approximately coplanar set of four points from the source cloud.
        * \param[out] base_indices selected source cloud indices, further used as base (B)
        * \param[out] ratio the two diagonal intersection ratios (r1,r2) of the base points
        * \return
        * * < 0 no coplanar four point sets with large enough sampling distance was found
        * * = 0 a set of four congruent points was selected
        */
      int
      selectBase (std::vector <int> &base_indices, float (&ratio)[2]);

      /** \brief Select randomly a triplet of points with large point-to-point distances. The minimum point
        * sampling distance is calculated based on the estimated point cloud overlap during initialization.
        *
        * \param[out] base_indices indices of base B
        * \return
        * * < 0 no triangle with large enough base lines could be selected
        * * = 0 base triangle succesully selected
        */
      int
      selectBaseTriangle (std::vector <int> &base_indices);

      /** \brief Setup the base (four coplanar points) by ordering the points and computing intersection
        * ratios and segment to segment distances of base diagonal.
        *
        * \param[in,out] base_indices indices of base B (will be reordered)
        * \param[out] ratio diagonal intersection ratios of base points
        */
      void
      setupBase (std::vector <int> &base_indices, float (&ratio)[2]);

      /** \brief Calculate intersection ratios and segment to segment distances of base diagonals.
        * \param[in] base_indices indices of base B
        * \param[out] ratio diagonal intersection ratios of base points
        * \return quality value of diagonal intersection
        */
      float
      segmentToSegmentDist (const std::vector <int> &base_indices, float (&ratio)[2]);

      /** \brief Search for corresponding point pairs given the distance between two base points.
        *
        * \param[in] idx1 first index of current base segment (in source cloud)
        * \param[in] idx2 second index of current base segment (in source cloud)
        * \param[out] pairs resulting point pairs with point-to-point distance close to ref_dist
        * \return
        * * < 0 no corresponding point pair was found
        * * = 0 at least one point pair candidate was found
        */
      virtual int
      bruteForceCorrespondences (int idx1, int idx2, pcl::Correspondences &pairs);

      /** \brief Determine base matches by combining the point pair candidate and search for coinciding
        * intersection points using the diagonal segment ratios of base B. The coincidation threshold is
        * calculated during initialization (coincidation_limit_).
        *
        * \param[in] base_indices indices of base B
        * \param[out] matches vector of candidate matches w.r.t the base B
        * \param[in] pairs_a point pairs corresponding to points of 1st diagonal of base B
        * \param[in] pairs_b point pairs corresponding to points of 2nd diagonal of base B
        * \param[in] ratio diagonal intersection ratios of base points
        * \return
        * * < 0 no base match could be found
        * * = 0 at least one base match was found
        */
      virtual int
      determineBaseMatches (
        const std::vector <int> &base_indices,
        std::vector <std::vector <int> > &matches,
        const pcl::Correspondences &pairs_a,
        const pcl::Correspondences &pairs_b,
        const float (&ratio)[2]);

      /** \brief Check if outer rectangle distance of matched points fit with the base rectangle.
        *
        * \param[in] match_indices indices of match M
        * \param[in] ds edge lengths of base B
        * \return
        * * < 0 at least one edge of the match M has no corresponding one in the base B
        * * = 0 edges of match M fits to the ones of base B
        */
      int
      checkBaseMatch (const std::vector <int> &match_indices, const float (&ds)[4]);

      /** \brief Method to handle current candidate matches. Here we validate and evaluate the matches w.r.t the
        * base and store the best fitting match (together with its score and estimated transformation).
        * \note For forwards compatibility the results are stored in 'vectors of size 1'.
        *
        * \param[in] base_indices indices of base B
        * \param[in,out] matches vector of candidate matches w.r.t the base B. The candidate matches are 
        * reordered during this step.
        * \param[out] candidates vector which contains the candidates matches M
        */
      virtual void
      handleMatches (
        const std::vector <int> &base_indices,
        std::vector <std::vector <int> > &matches,
        MatchingCandidates &candidates);

      /** \brief Sets the correspondences between the base B and the match M by using the distance of each point
        * to the centroid of the rectangle.
        *
        * \param[in] base_indices indices of base B
        * \param[in] match_indices indices of match M
        * \param[out] correspondences resulting correspondences
        */
      virtual void
      linkMatchWithBase (
        const std::vector <int> &base_indices,
        std::vector <int> &match_indices,
        pcl::Correspondences &correspondences);

      /** \brief Validate the matching by computing the transformation between the source and target based on the
        * four matched points and by comparing the mean square error (MSE) to a threshold. The MSE limit was
        * calculated during initialization (max_mse_).
        *
        * \param[in] base_indices indices of base B
        * \param[in] match_indices indices of match M
        * \param[in] correspondences corresondences between source and target
        * \param[out] transformation resulting transformation matrix
        * \return
        * * < 0 MSE bigger than max_mse_
        * * = 0 MSE smaller than max_mse_
        */
      virtual int
      validateMatch (
        const std::vector <int> &base_indices,
        const std::vector <int> &match_indices,
        const pcl::Correspondences &correspondences,
        Eigen::Matrix4f &transformation);

      /** \brief Validate the transformation by calculating the number of inliers after transforming the source cloud.
        * The resulting fitness score is later used as the decision criteria of the best fitting match.
        *
        * \param[out] transformation updated orientation matrix using all inliers
        * \param[out] fitness_score current best fitness_score
        * \note fitness score is only updated if the score of the current transformation exceeds the input one.
        * \return
        * * < 0 if previous result is better than the current one (score remains)
        * * = 0 current result is better than the previous one (score updated)
        */
      virtual int
      validateTransformation (Eigen::Matrix4f &transformation, float &fitness_score);

      /** \brief Final computation of best match out of vector of best matches. To avoid cross thread dependencies
        *  during parallel running, a best match for each try was calculated.
        * \note For forwards compatibility the candidates are stored in vectors of 'vectors of size 1'.
        * \param[in] candidates vector of candidate matches
        */
      virtual void
      finalCompute (const std::vector <MatchingCandidates > &candidates);


      /** \brief Normals of source point cloud. */
      NormalsConstPtr source_normals_;

      /** \brief Normals of target point cloud. */
      NormalsConstPtr target_normals_;


      /** \brief Number of threads for parallelization (standard = 1).
        * \note Only used if run compiled with OpenMP.
        */
      int nr_threads_;

      /** \brief Estimated overlap between source and target (standard = 0.5). */
      float approx_overlap_;

      /** \brief Delta value of 4pcs algorithm (standard = 1.0).
        * It can be used as:
        * * absolute value (normalization = false), value should represent the point accuracy to ensure finding neighbors between source <-> target
        * * relative value (normalization = true), to adjust the internally calculated point accuracy (= point density)
        */
      float delta_;

      /** \brief Score threshold to stop calculation with success.
        * If not set by the user it is equal to the approximated overlap
        */
      float score_threshold_;

      /** \brief The number of points to uniformly sample the source point cloud. (standard = 0 => full cloud). */
      int nr_samples_;

      /** \brief Maximum normal difference of corresponding point pairs in degrees (standard = 90). */
      float max_norm_diff_;

      /** \brief Maximum allowed computation time in seconds (standard = 0 => ~unlimited). */
      int max_runtime_;


      /** \brief Resulting fitness score of the best match. */
      float fitness_score_;
      

      /** \brief Estimated diamter of the target point cloud. */
      float diameter_;

      /** \brief Estimated squared metric overlap between source and target.
        * \note Internally calculated using the estimated overlap and the extent of the source cloud.
        * It is used to derive the minimum sampling distance of the base points as well as to calculated
        * the number of trys to reliable find a correct mach.
        */
      float max_base_diameter_sqr_;

      /** \brief Use normals flag. */
      bool use_normals_;

      /** \brief Normalize delta flag. */
      bool normalize_delta_;


      /** \brief A pointer to the vector of source point indices to use after sampling. */
      pcl::IndicesPtr source_indices_;

      /** \brief A pointer to the vector of target point indices to use after sampling. */
      pcl::IndicesPtr target_indices_;

      /** \brief Maximal difference between corresponding point pairs in source and target.
        * \note Internally calculated using an estimation of the point density.
        */
      float max_pair_diff_;

      /** \brief Maximal difference between the length of the base edges and valid match edges.
        * \note Internally calculated using an estimation of the point density.
        */
      float max_edge_diff_;

      /** \brief Maximal distance between coinciding intersection points to find valid matches.
        * \note Internally calculated using an estimation of the point density.
        */
      float coincidation_limit_;

      /** \brief Maximal mean squared errors of a transformation calculated from a candidate match.
        * \note Internally calculated using an estimation of the point density.
        */
      float max_mse_;

      /** \brief Maximal squared point distance between source and target points to count as inlier.
        * \note Internally calculated using an estimation of the point density.
        */
      float max_inlier_dist_sqr_;


      /** \brief Definition of a small error. */
      const float small_error_;

    };
  }; // namespace registration  
}; // namespace pcl 

#include <pcl/registration/impl/ia_fpcs.hpp>

#endif // PCL_REGISTRATION_IA_FPCS_H_
