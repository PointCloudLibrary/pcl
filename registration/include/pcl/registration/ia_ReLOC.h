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

#ifndef PCL_REGISTRATION_IA_RELOC_H_
#define PCL_REGISTRATION_IA_RELOC_H_

#include <pcl/common/common.h>
#include <pcl/registration/registration.h>
#include <pcl/geometry/hough_space_3d.h>

namespace pcl
{

  /** \brief Compute the bounding box of the cloud centered in the centroid of the cloud and extended 2 x sigmaFactor x standard deviations of the 3 coordinates.
  * 
  * \param[in] cloud input cloud 
  * \param[in] sigmaFactor extension factor of the bounding box
  * \param[out] bbox bounding box of cloud as (p1_x, p1_y, p1_z, p2_x, p2_y, p2_z)
  * \ingroup registration
  */
  template <typename PointInT> void
    computeBBoxFromStddev(const pcl::PointCloud<PointInT> &cloud, std::vector<double> &bbox, const float sigmaFactor = 2.0);

  /** \brief Extend the bounding box by factor_X, factor_Y and factor_Z w.r.t. the 3 coordinates.
  * 
  * \param[in] bbox input bounding box
  * \param[in] factor_X extension factor of x coordinates
  * \param[in] factor_Y extension factor of y coordinates
  * \param[in] factor_Z extension factor of z coordinates
  * \ingroup registration
  */
  template<typename T> void 
    extendBBox(T* bbox, const T factor_X, const T factor_Y, const T factor_Z);


  /** \brief Sort a vector of indeces w.r.t. values in vector f */
  class SortIdsWrtFloatDecr
  {
  public:
    SortIdsWrtFloatDecr(std::vector<float> &f) : _f(f) {}
    bool operator()(int i, int j) {
      return _f[i] > _f[j];
    }
  private:
    std::vector<float> & _f;
  };

  namespace registration
  {

    /** \brief ReLOC initial alignment algorithm proposed in:
    * Petrelli A., Di Stefano L., "Pairwise registration by local orientation cues", Computer Graphics Forum, 2015.
    * 
    * The algorithm can be evaluated through the benchmark proposed in the same paper and downloadable from <a href="https://github.com/aliosciapetrelli/Pairwise3DRegistrationEvaluation">here</a>.
    * This <a href="https://github.com/aliosciapetrelli/ReLOC/blob/master/Pcl_ReLOC_benchmarkmain.cpp">examplecpp</a> shows how to apply the benchmark.
    * \author A. Petrelli
    * \ingroup registration
    */
    template <typename PointSource, typename PointTarget, typename NormalT = pcl::Normal, typename Scalar = float>
    class ReLOCInitialAlignment : public Registration <PointSource, PointTarget, Scalar>
    {
    public:

      typedef pcl::PointCloud<PointSource> PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef pcl::PointCloud<PointTarget> PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef pcl::PointCloud <NormalT> Normals;
      typedef typename Normals::ConstPtr NormalsConstPtr;

      typedef pcl::PointCloud<ReferenceFrame> ReferenceFrames;
      typedef typename ReferenceFrames::Ptr ReferenceFramesPtr;

      typedef typename pcl::PointCloud<float> PointCloudSignedDistance;
      typedef typename PointCloudSignedDistance::Ptr PointCloudSignedDistancePtr;
      typedef typename PointCloudSignedDistance::ConstPtr PointCloudSignedDistanceConstPtr;


      /** \brief Constructor.
      */
      ReLOCInitialAlignment ();

      /** \brief Destructor. */
      virtual ~ReLOCInitialAlignment ()
      {};

      /** \brief Set the normals of the source cloud.
      * \param[in] normals normals of source cloud.
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

      /** \brief Set the normals of the target cloud.
      * \param[in] normals normals of target cloud.
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

      /** \brief Set seed of random functions.
      * \param seed
      */
      inline void
        setSeed (unsigned int seed)
      {
        seed_ = seed;
      }

      /** \brief Get the value of the internal \a seed parameter.
      */
      inline unsigned int
        getSeed ()
      {
        return (seed_);
      }

      /** \bries If true, use random detector, otherwise use FlatKeypoint detector. */
      inline void
        useRandomDetector(const bool use_random_detector){use_random_detector_ = use_random_detector;}

      /** \brief Set the minimum number of neighbors that has to be found while applying the radius searches during the FlatKeypoint detection.
      * \param[in] flat_keypoint_min_neighbors the minimum number of neighbors required
      */
      inline void
        setFlatKeypointMinNeighbors (int flat_keypoint_min_neighbors){flat_keypoint_min_neighbors_ = flat_keypoint_min_neighbors;};

      /** \brief Set the radius used to compute flatness during the FlatKeypoint detection.
      * \param[in] flat_keypoint_Rf the radius to compute flatness.
      */
      inline void
        setFlatKeypointRf (double flat_keypoint_Rf){flat_keypoint_Rf_ = flat_keypoint_Rf;};

      /** \brief Set the radius used to discard points around seed and selected flat points  during the FlatKeypoint detection.
      * \param[in] flat_keypoint_R_discard the radius used to discard points around seed and selected flat points.
      */
      inline void
        setFlatKeypointRdiscard (double flat_keypoint_R_discard){flat_keypoint_R_discard_ = flat_keypoint_R_discard;};

      /** \brief Set the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the first step of the FlatKeypoint detection.
      * \param[in] flat_keypoint_R1_search the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the first step of the FlatKeypoint detection.
      */
      inline void
        setFlatKeypointR1search (double flat_keypoint_R1_search){flat_keypoint_R1_search_ = flat_keypoint_R1_search;};

      /** \brief Set the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the second step of the FlatKeypoint detection.
      * \param[in] flat_keypoint_R2_search the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the second step of the FlatKeypoint detection.
      */
      inline void
        setFlatKeypointR2search (double flat_keypoint_R2_search){flat_keypoint_R2_search_ = flat_keypoint_R2_search;};

      /** \brief Set the percentage of cloud points requested to be discarded during the first step of the FlatKeypoint detection.
      * \param[in] flat_keypoint_T1_search percentage of cloud points requested to be discarded during the first step of the FlatKeypoint detection.
      */
      inline void
        setFlatKeypointT1search (double flat_keypoint_T1_search){flat_keypoint_T1_search_ = flat_keypoint_T1_search;};

      /** \brief Set the percentage of cloud points requested to be discarded during the second step of the FlatKeypoint detection.
      * \param[in] flat_keypoint_T2_search percentage of cloud points requested to be discarded during the second step of the FlatKeypoint detection.
      */
      inline void
        setFlatKeypointT2search (double flat_keypoint_T2_search){flat_keypoint_T2_search_ = flat_keypoint_T2_search;};


      /** \brief Set the number of keypoints extracted by the random detector.
      * \param[in] n_random_keypoints number of keypoints extracted by the random detector.
      */
      inline void
        setNrandomKeypoints (double n_random_keypoints){n_random_keypoints_ = n_random_keypoints;};

      /** \brief Set the maximum distance of the points used to estimate the z_axis of the FLARE Reference Frame for a given point.
      *
      * \param[in] radius The search radius for z axis in the FLARE computation.
      */
      inline void
        setFlareNormalRadius (float radius)
      {
        flare_normal_radius_ = radius;
      }

      /** \brief Get the maximum distance of the points used to estimate the z_axis of the FLARE Reference Frame for a given point.
      *
      * \return The search radius for z axis in the FLARE computation.
      */
      inline float
        getFlareNormalRadius () const
      {
        return (flare_normal_radius_);
      }


      /** \brief Set the maximum distance of the points used to estimate the x_axis of the FLARE Reference Frame for a given point.
      *
      * \param[in] radius The search radius for x axis in the FLARE computation.
      */
      inline void
        setFlareTangentRadius (float radius)
      {
        flare_tangent_radius_ = radius;
      }

      /** \brief Get the maximum distance of the points used to estimate the x_axis of the FLARE Reference Frame for a given point.
      *
      * \return The search radius for x axis  in the FLARE computation.
      */
      inline float
        getFlareTangentRadius () const
      {
        return (flare_tangent_radius_);
      }

      /** \brief Set the percentage of the search tangent radius after which a point is considered part of the support in the FLARE computation.
      *
      * \param[in] margin_thresh the percentage of the search tangent radius after which a point is considered part of the support in the FLARE computation.
      */
      inline void
        setFlareMarginThresh (float flare_margin_thresh)
      {
        flare_margin_thresh_ = flare_margin_thresh;
      }

      /** \brief Get the percentage of the search tangent radius after which a point is considered part of the support in the FLARE computation.
      *
      * \return The percentage of the search tangent radius after which a point is considered part of the support in the FLARE computation.
      */
      inline float
        getFlareMarginThresh () const
      {
        return (flare_margin_thresh_);
      }


      /** \brief Set min number of neighbours required for the computation of Z axis in the FLARE computation.
      *
      * \param[in] min number of neighbours required for the computation of Z axis in the FLARE computation.
      */
      inline void
        setFlareMinNeighboursForNormalAxis (int flare_min_neighbors_for_normal_axis)
      {
        flare_min_neighbors_for_normal_axis_ = flare_min_neighbors_for_normal_axis;
      }

      /** \brief Get min number of neighbours required for the computation of Z axis in the FLARE computation.
      *
      * \return min number of neighbours required for the computation of Z axis in the FLARE computation.
      */
      inline int
        getFlareMinNeighboursForNormalAxis () const
      {
        return (flare_min_neighbors_for_normal_axis_);
      }


      /** \brief Set min number of neighbours required for the computation of X axis in the FLARE computation.
      *
      * \param[in] min number of neighbours required for the computation of X axis in the FLARE computation.
      */
      inline void
        setFlareMinNeighboursForTangentAxis (int flare_min_neighbors_for_tangent_axis)
      {
        flare_min_neighbors_for_tangent_axis_ = flare_min_neighbors_for_tangent_axis;
      }

      /** \brief Get min number of neighbours required for the computation of X axis in the FLARE computation.
      *
      * \return min number of neighbours required for the computation of X axis in the FLARE computation.
      */
      inline int
        getFlareMinNeighboursForTangentAxis () const
      {
        return (flare_min_neighbors_for_tangent_axis_);
      }

      /** \brief Set the percentage of used points in the support used for the computation of the x axis in FLARE computation.
      *
      * \param[in] flare_x_support_sampling_perc percentage of used points in the support used for the computation of the x axis in FLARE computation.
      */
      inline void
        setFlareXsupportSamplingPerc (float flare_x_support_sampling_perc)
      {
        flare_x_support_sampling_perc_ = flare_x_support_sampling_perc;
      }

      /** \brief Set the threshold used by the matcher for establishing if two feature points are a candidate correspondence. 
      *
      * \param[in] matcher_T_D threshold used by the matcher for establishing if two feature points are a candidate correspondence. 
      */
      inline void
        setMatcherTD (float matcher_TD)
      {
        matcher_TD_ = matcher_TD;
      }

      /** \brief Set the extension factor of the hough space estimated w.r.t. the target cloud.
      *
      * \param[in] hough_f extension factor of the hough space estimated w.r.t. the target cloud.
      */
      inline void
        setHoughF (double hough_f)
      {
        hough_f_ = hough_f;
      }

      /** \brief Set the side of a bin of the hough space.
      *
      * \param[in] hough_Sbin side of a bin of the hough space.
      */
      inline void
        setHoughSbin (float hough_Sbin)
      {
        hough_Sbin_ = hough_Sbin;
      }

      /** \brief Set the ransac threshold to establish inlier and outlier.
      *
      * \param[in] ransac_T Ransac threshold to establish inlier and outlier.
      */
      inline void
        setRansacT (float ransac_T)
      {
        ransac_T_ = ransac_T;
      }

      /** \brief Set the ransac max number of iterations.
      *
      * \param[in] ransac_N Ransac max number of iterations.
      */
      inline void
        setRansacN (int ransac_N)
      {
        ransac_N_ = ransac_N;
      }




    protected:

      using PCLBase <PointSource>::input_;
      using PCLBase <PointSource>::indices_;

      using Registration <PointSource, PointTarget, Scalar>::reg_name_;
      using Registration <PointSource, PointTarget, Scalar>::target_;
      using Registration <PointSource, PointTarget, Scalar>::final_transformation_;
      using Registration <PointSource, PointTarget, Scalar>::target_cloud_updated_;
      using Registration <PointSource, PointTarget, Scalar>::source_cloud_updated_;
      using Registration <PointSource, PointTarget, Scalar>::getClassName;

      /** \brief Rigid transformation computation method.
      * \param output the transformed input point cloud dataset using the rigid transformation found
      * \param guess The computed transformation
      */
      virtual void
        computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess);


      /** \brief Internal computation initialization. */
      virtual bool
        initCompute ();

      /** \brief Internal computation deinitalization. */
      virtual bool
        deinitCompute ();

      /** \brief Perform keypoint detection. Either at random or based on FlatKeypoint detector
      *
      * \param[in] cloud input cloud
      * \param[in] normals normals of input cloud
      * \param[out] keypoints extracted keypoints
      */
      template <typename PointT> void
        detectKeypoints (typename pcl::PointCloud<PointT>::ConstPtr cloud, NormalsConstPtr normals, typename pcl::PointCloud<PointT>::Ptr &keypoints);

      /** \brief Compute FLARE local reference frames for each keypoint in keypoints.
      *
      * \param[in] cloud input cloud.
      * \param[in] normals normals of input cloud.
      * \param[in] keypoints keypoints for which FLARE is computed.
      * \param[out] flares computed local reference frames.
      * \param[out] flare_signed_distances scores D of the computed local reference frames.
      */
      template <typename PointT> void
        computeFlares (typename pcl::PointCloud<PointT>::ConstPtr cloud, NormalsConstPtr normals, typename pcl::PointCloud<PointT>::ConstPtr keypoints, ReferenceFramesPtr &flares, std::vector<float> &flare_signed_distances);

      /** \brief Match feature points based on scores D of the computed local reference frames.
      *
      * \param[in] target_flare_signed_distances scores D of the local reference frames of target cloud.
      * \param[in] source_flare_signed_distances scores D of the local reference frames of source cloud.
      * \param[out] correspondences matches.
      */
      void 
        match (const std::vector<float> &target_flare_signed_distances, const std::vector<float> &source_flare_signed_distances, pcl::CorrespondencesPtr &correspondences);

      /** \brief Match Perform hough voting.
      *
      * \param[in] matcher_correspondences correspondences produced by the matcher based on scores D.
      * \param[out] hough_correspondences correspondences survived after hough voting.
      */
      void 
        houghVoting (pcl::CorrespondencesConstPtr matcher_correspondences, pcl::CorrespondencesPtr &hough_correspondences);

      /** \brief Match Perform hough voting.
      *
      * \param[in] matcher_correspondences correspondences produced by the matcher based on scores D.
      * \param[out] hough_correspondences correspondences survived after hough voting.
      */
      void
        estimateRigidMotion ( pcl::CorrespondencesConstPtr hough_correspondences, Eigen::Matrix4f &final_transformation, pcl::CorrespondencesPtr &ransac_correspondences);

      /** \brief Keypoints extracted from the source cloud.*/
      PointCloudSourcePtr source_keypoints_;

      /** \brief Keypoints extracted from the target cloud.*/
      PointCloudTargetPtr target_keypoints_;

      /** \brief FLARE local reference frames computed on the source cloud.*/
      ReferenceFramesPtr source_lrf_cloud_;

      /** \brief FLARE local reference frames computed on the target cloud.*/
      ReferenceFramesPtr target_lrf_cloud_;

      /** \brief Scores D of the FLARE local reference frames computed on the source cloud.*/
      std::vector<float> source_flare_signed_distances_;

      /** \brief Scores D of the FLARE local reference frames computed on the target cloud.*/
      std::vector<float> target_flare_signed_distances_;

      /** \brief Normals of source point cloud. */
      NormalsConstPtr source_normals_;

      /** \brief Normals of target point cloud. */
      NormalsConstPtr target_normals_;

      /** \brief Centroid of the source cloud represented w.r.t. the local reference frames of source cloud. */
      std::vector<Eigen::Vector3f> source_votes_;

      /** \brief Hough space */
      boost::shared_ptr<pcl::HoughSpace3D<unsigned short int> > hough_space_;

      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Use random detector. */
      bool use_random_detector_;

      /** \brief Minimum number of neighbors that has to be found while applying radius searches during the FlatKeypoint detection. */
      int flat_keypoint_min_neighbors_;

      /** \brief Radius used to compute flatness during the FlatKeypoint detection.*/
      double flat_keypoint_Rf_;

      /** \brief Radius used to discard points around seed and selected flat points during the FlatKeypoint detection.*/
      double flat_keypoint_R_discard_;

      /** \brief Radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the first step of the FlatKeypoint detection.*/
      double flat_keypoint_R1_search_;

      /** \brief Radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the second step of the FlatKeypoint detection.*/
      double flat_keypoint_R2_search_;

      /** \brief Percentage of cloud points requested to be discarded during the first step of the FlatKeypoint detection.*/
      double flat_keypoint_T1_search_;

      /** \brief Percentage of cloud points requested to be discarded during the second step of the FlatKeypoint detection.*/
      double flat_keypoint_T2_search_;

      /** \brief Number of keypoints extracted by the random detector.*/
      int n_random_keypoints_;

      /** \brief Radius used to find normal axis in FLARE computation. */
      float flare_normal_radius_;

      /** \brief Radius used to find tangent axis in FLARE computation. */
      float flare_tangent_radius_;

      /** \brief Threshold that define if a support point is near the margins in FLARE computation. */
      float flare_margin_thresh_; 

      /** \brief Min number of neighbours required for the computation of Z axis in FLARE computation. Otherwise, feature point normal is used. */
      int flare_min_neighbors_for_normal_axis_;

      /** \brief Min number of neighbours required for the computation of X axis in FLARE computation. Otherwise, a random X axis is set. */
      int flare_min_neighbors_for_tangent_axis_;

      /** \brief The percentage of used points in the support used for the computation of the x axis in FLARE computation. */
      float flare_x_support_sampling_perc_;

      /** \brief Threshold used by the matcher for establishing if two feature points are a candidate correspondence. */
      float matcher_TD_;

      /** \brief Extension factor of the hough space estimated w.r.t. the target cloud. */
      double hough_f_;

      /** \brief side of a bin of the hough space. */
      float hough_Sbin_;

      /** \brief Ransac threshold to establish inlier and outlier. */
      float ransac_T_;

      /** \brief Ransac max number of iterations. */
      int ransac_N_;

    };
  }; // namespace registration  
}; // namespace pcl 

//#ifdef PCL_NO_PRECOMPILE
#include <pcl/registration/impl/ia_ReLOC.hpp>
//#endif

#endif // PCL_REGISTRATION_IA_RELOC_H_
