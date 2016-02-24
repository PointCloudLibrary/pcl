/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *	All rights reserved
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are met
 *
 *	 * The use for research only (no for any commercial application).
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

#ifndef PCL_REGISTRATION_IA_KFPCS_H_
#define PCL_REGISTRATION_IA_KFPCS_H_

#include <pcl/registration/ia_fpcs.h>

namespace pcl
{
  namespace registration
  {
    /** \brief KFPCSInitialAlignment computes corresponding four point congruent sets based on keypoints
      * as described in: "Markerless point cloud registration with keypoint-based 4-points congruent sets",
      * Pascal Theiler, Jan Dirk Wegner, Konrad Schindler. ISPRS Annals II-5/W2, 2013. Presented at ISPRS Workshop
      * Laser Scanning, Antalya, Turkey, 2013.
      * \note Method has since been improved and some variations to the paper exist.
      * \author P.W.Theiler
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename NormalT = pcl::Normal, typename Scalar = float>
    class KFPCSInitialAlignment : public virtual FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>
    {
    public:
      /** \cond */
      typedef boost::shared_ptr <KFPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar> > Ptr;
      typedef boost::shared_ptr <const KFPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar> > ConstPtr;

      typedef pcl::PointCloud <PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::iterator PointCloudSourceIterator;

      typedef pcl::PointCloud <PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::iterator PointCloudTargetIterator;

      typedef pcl::registration::MatchingCandidate MatchingCandidate;
      typedef pcl::registration::MatchingCandidates MatchingCandidates;
      /** \endcond */


      /** \brief Constructor. */
      KFPCSInitialAlignment ();

      /** \brief Destructor. */
      virtual ~KFPCSInitialAlignment ()
      {};


      /** \brief Set the upper translation threshold used for score evaluation.
        * \param[in] upper_trl_boundary upper translation threshold
        */
      inline void
      setUpperTranslationThreshold (float upper_trl_boundary)
      {
        upper_trl_boundary_ = upper_trl_boundary;
      };

      /** \return the upper translation threshold used for score evaluation. */
      inline float
      getUpperTranslationThreshold () const
      {
        return (upper_trl_boundary_);
      };


      /** \brief Set the lower translation threshold used for score evaluation.
        * \param[in] lower_trl_boundary lower translation threshold
        */
      inline void
      setLowerTranslationThreshold (float lower_trl_boundary)
      {
        lower_trl_boundary_ = lower_trl_boundary;
      };

      /** \return the lower translation threshold used for score evaluation. */
      inline float
      getLowerTranslationThreshold () const
      {
        return (lower_trl_boundary_);
      };


      /** \brief Set the weighting factor of the translation cost term. 
        * \param[in] lambda the weighting factor of the translation cost term
        */
      inline void
      setLambda (float lambda)
      {
        lambda_ = lambda;
      };

      /** \return the weighting factor of the translation cost term. */
      inline float
      getLambda () const
      {
        return (lambda_);
      };


      /** \brief Get the N best unique candidate matches according to their fitness score.
        * The method only returns unique transformations comparing the translation
        * and the 3D rotation to already returned transformations.
        *
        * \note The method may return less than N candidates, if the number of unique candidates
        * is smaller than N
        *
        * \param[in] n number of best candidates to return
        * \param[in] min_angle3d minimum 3D angle difference in radian
        * \param[in] min_translation3d minimum 3D translation difference
        * \param[out] candidates vector of unique candidates
        */
      void
      getNBestCandidates (int n, float min_angle3d, float min_translation3d, MatchingCandidates &candidates);

      /** \brief Get all unique candidate matches with fitness scores above a threshold t.
        * The method only returns unique transformations comparing the translation
        * and the 3D rotation to already returned transformations.
        *
        * \param[in] t fitness score threshold
        * \param[in] min_angle3d minimum 3D angle difference in radian
        * \param[in] min_translation3d minimum 3D translation difference
        * \param[out] candidates vector of unique candidates
        */
      void
      getTBestCandidates (float t, float min_angle3d, float min_translation3d, MatchingCandidates &candidates);
      

    protected:
      
      using PCLBase <PointSource>::deinitCompute;
      using PCLBase <PointSource>::input_;
      using PCLBase <PointSource>::indices_;

      using Registration <PointSource, PointTarget, Scalar>::reg_name_;
      using Registration <PointSource, PointTarget, Scalar>::tree_;
      using Registration <PointSource, PointTarget, Scalar>::final_transformation_;
      using Registration <PointSource, PointTarget, Scalar>::ransac_iterations_;
      using Registration <PointSource, PointTarget, Scalar>::correspondences_;
      using Registration <PointSource, PointTarget, Scalar>::converged_;

      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::delta_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::approx_overlap_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::max_pair_diff_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::max_edge_diff_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::coincidation_limit_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::max_mse_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::max_inlier_dist_sqr_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::diameter_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::normalize_delta_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::fitness_score_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::score_threshold_;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::linkMatchWithBase;
      using FPCSInitialAlignment <PointSource, PointTarget, NormalT, Scalar>::validateMatch;
      

      /** \brief Internal computation initialization. */
      virtual bool
      initCompute ();

      /** \brief Method to handle current candidate matches. Here we validate and evaluate the matches w.r.t the
        * base and store the sorted matches (together with score values and estimated transformations).
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

      /** \brief Validate the transformation by calculating the score value after transforming the input source cloud.
        * The resulting score is later used as the decision criteria of the best fitting match.
        *
        * \param[out] transformation updated orientation matrix using all inliers
        * \param[out] fitness_score current best score
        * \note fitness score is only updated if the score of the current transformation exceeds the input one.
        * \return
        * * < 0 if previous result is better than the current one (score remains)
        * * = 0 current result is better than the previous one (score updated)
        */
      virtual int
      validateTransformation (Eigen::Matrix4f &transformation, float &fitness_score);

      /** \brief Final computation of best match out of vector of matches. To avoid cross thread dependencies
        *  during parallel running, a best match for each try was calculated.
        * \note For forwards compatibility the candidates are stored in vectors of 'vectors of size 1'.
        * \param[in] candidates vector of candidate matches
        */
      virtual void
      finalCompute (const std::vector <MatchingCandidates > &candidates);


      /** \brief Lower boundary for translation costs calculation.
        * \note If not set by the user, the translation costs are not used during evaluation.
        */
      float lower_trl_boundary_;

      /** \brief Upper boundary for translation costs calculation.
        * \note If not set by the user, it is calculated from the estimated overlap and the diameter
        * of the point cloud.
        */
      float upper_trl_boundary_;

      /** \brief Weighting factor for translation costs (standard = 0.5). */
      float lambda_;


      /** \brief Container for resulting vector of registration candidates. */
      MatchingCandidates candidates_;

      /** \brief Flag if translation score should be used in validation (internal calculation). */
      bool use_trl_score_;

      /** \brief Subset of input indices on which we evaluate candidates.
        * To speed up the evaluation, we only use a fix number of indices defined during initialization.
        */
      pcl::IndicesPtr indices_validation_;

    };
  }; // namespace registration
}; // namespace pcl 

#include <pcl/registration/impl/ia_kfpcs.hpp>

#endif // PCL_REGISTRATION_IA_KFPCS_H_
