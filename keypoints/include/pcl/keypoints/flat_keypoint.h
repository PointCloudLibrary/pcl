/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 */

#ifndef PCL_FLAT_KEYPOINT_H_
#define PCL_FLAT_KEYPOINT_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  /** \brief Flat Keypoints detector proposed in:
  * Petrelli A., Di Stefano L., "Pairwise registration by local orientation cues", Computer Graphics Forum, 2015
  *
  * and deployed in ReLOC initial alignment algorithm (pcl/registration/ia_ReLOC.h)
  * Code example:
  *
  * \code
  * pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());;
  * pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());;
  * pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
  *
  * // Fill in the model cloud
  * // Compute model normals
  *
  * double model_resolution;
  *
  * // Compute model_resolution
  *
  * pcl::FlatKeypoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> flat_keypoint_detector;
  *
  * flat_keypoint_detector.setRadiusSearch (3 * model_resolution);  //Rf in the paper
  * flat_keypoint_detector.setRdiscard (2 * model_resolution);
  * flat_keypoint_detector.setR1search (2 * model_resolution);
  * flat_keypoint_detector.setR2search (20 * model_resolution);
  * flat_keypoint_detector.setT1search (0.9);
  * flat_keypoint_detector.setT2search (0.9);
  * flat_keypoint_detector.setInputCloud (model);
  * flat_keypoint_detector.setNormals (model_normals);
  * flat_keypoint_detector.compute (*model_keypoints);
  * \endcode
  *
  * \author Alioscia Petrelli
  * \ingroup keypoints
  */
  template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
  class FlatKeypoint : public Keypoint<PointInT, PointOutT>
  {
  public:
    typedef boost::shared_ptr<FlatKeypoint<PointInT, PointOutT, NormalT> > Ptr;
    typedef boost::shared_ptr<const FlatKeypoint<PointInT, PointOutT, NormalT> > ConstPtr;

    typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
    typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef typename pcl::PointCloud<NormalT> PointCloudN;
    typedef typename PointCloudN::Ptr PointCloudNPtr;
    typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

    using Keypoint<PointInT, PointOutT>::name_;
    using Keypoint<PointInT, PointOutT>::input_;
    using Keypoint<PointInT, PointOutT>::surface_;
    using Keypoint<PointInT, PointOutT>::tree_;
    using Keypoint<PointInT, PointOutT>::search_radius_;
    using Keypoint<PointInT, PointOutT>::search_parameter_;
    using Keypoint<PointInT, PointOutT>::keypoints_indices_;


    /** \brief Constructor. */
    FlatKeypoint () :
    normals_ (new pcl::PointCloud<NormalT>),
      seed_ (static_cast<unsigned int> (time (NULL))),
      min_neighbors_ (5),
      R_discard_ (2),
      R1_search_ (2),
      R2_search_ (20),
      T1_search_ (0.9),
      T2_search_ (0.9)
    {
      name_ = "FlatKeypoint";
    }

    /** \brief Set the normals of the input cloud.
    * \param[in] normals normals of the input cloud.
    */
    inline void
      setNormals (const PointCloudNConstPtr &normals)
    {
      normals_ = normals;
    }

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

    /** \brief Set the minimum number of neighbors that has to be found while applying the radius searches.
    * \param[in] min_neighbors the minimum number of neighbors required
    */
    inline void
      setMinNeighbors (int min_neighbors){min_neighbors_ = min_neighbors;};

    /** \brief Set the radius used to discard points around seed and selected flat points.
    * \param[in] R_discard the radius used to discard points around seed and selected flat points.
    */
    inline void
      setRdiscard (double R_discard){R_discard_ = R_discard;};

    /** \brief Set the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the first step.
    * \param[in] R1_search the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the first step.
    */
    inline void
      setR1search (double R1_search){R1_search_ = R1_search;};

    /** \brief Set the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the second step.
    * \param[in] R2_search the radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the second step.
    */
    inline void
      setR2search (double R2_search){R2_search_ = R2_search;};

    /** \brief Set the percentage of cloud points requested to be discarded during the first step.
    * \param[in] T1_search percentage of cloud points requested to be discarded during the first step.
    */
    inline void
      setT1search (double T1_search){T1_search_ = T1_search;};

    /** \brief Set the percentage of cloud points requested to be discarded during the second step.
    * \param[in] T2_search percentage of cloud points requested to be discarded during the second step.
    */
    inline void
      setT2search (double T2_search){T2_search_ = T2_search;};


  protected:

    /** \brief Perform the initial checks before computing the keypoints.
    *  \return true if all the checks are passed, false otherwise
    */
    bool
      initCompute ();

    /** \brief Detect the keypoints by searching for flat points.
    * \param[out] output the resultant cloud of keypoints
    */
    void
      detectKeypoints (PointCloudOut &output);

    /** \brief apply a step of the 2-step detection process.
    *
    * Put in keypoints_indices the indices of found flat points.
    * \param[in] cloud input cloud
    * \param[in] points_flatness flatness of points of cloud.  -1.0 if not computed, -2.0 if not computable.
    * \param[in] R_discard radius used to discard points around seed and selected flat points.
    * \param[in] R_search radius of the support used to find the flat point (yellow support in Fig.6 of paper).
    * \param[in] T_search percentage of cloud points requested to be discarded.
    * \param[out] keypoints_indices 
    */
    void
      applyFlatKeypointsDetector (PointCloudInConstPtr cloud, std::vector<float> &points_flatness, const double R_discard, const double R_search, const double T_search, pcl::PointIndicesPtr keypoints_indices);


    /** \brief The cloud of normals related to the input surface. */
    PointCloudNConstPtr normals_;

    /** \brief Random number seed. */
    unsigned int seed_;

    /** \brief Minimum number of neighbors that has to be found while applying radius searches. */
    int min_neighbors_;

    /** \brief Radius used to discard points around seed and selected flat points.*/
    double R_discard_;

    /** \brief Radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the first step.*/
    double R1_search_;

    /** \brief Radius of the support used to find the flat point (yellow support in Fig.6 of paper) during the second step.*/
    double R2_search_;

    /** \brief Percentage of cloud points requested to be discarded during the first step.*/
    double T1_search_;

    /** \brief Percentage of cloud points requested to be discarded during the second step.*/
    double T2_search_;

  };

}

#include <pcl/keypoints/impl/flat_keypoint.hpp>

#endif /* PCL_FLAT_KEYPOINT_H_ */
