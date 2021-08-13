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

#pragma once

#include <pcl/keypoints/keypoint.h>
#include <pcl/octree/octree_search.h> // for OctreePointCloudSearch

namespace pcl
{
  /** \brief ISSKeypoint3D detects the Intrinsic Shape Signatures keypoints for a given
    * point cloud. This class is based on a particular implementation made by Federico
    * Tombari and Samuele Salti and it has been explicitly adapted to PCL.
    *
    * For more information about the original ISS detector, see:
    *
    *\par
    * Yu Zhong, “Intrinsic shape signatures: A shape descriptor for 3D object recognition,”
    * Computer Vision Workshops (ICCV Workshops), 2009 IEEE 12th International Conference on ,
    * vol., no., pp.689-696, Sept. 27 2009-Oct. 4 2009
    *
    * Code example:
    *
    * \code
    * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGBA> ());;
    * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    * pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    *
    * // Fill in the model cloud
    *
    * double model_resolution;
    *
    * // Compute model_resolution
    *
    * pcl::ISSKeypoint3D<pcl::PointXYZRGBA, pcl::PointXYZRGBA> iss_detector;
    *
    * iss_detector.setSearchMethod (tree);
    * iss_detector.setSalientRadius (6 * model_resolution);
    * iss_detector.setNonMaxRadius (4 * model_resolution);
    * iss_detector.setThreshold21 (0.975);
    * iss_detector.setThreshold32 (0.975);
    * iss_detector.setMinNeighbors (5);
    * iss_detector.setNumberOfThreads (4);
    * iss_detector.setInputCloud (model);
    * iss_detector.compute (*model_keypoints);
    * \endcode
    *
    * \author Gioia Ballin
    * \ingroup keypoints
    */

  template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
  class ISSKeypoint3D : public Keypoint<PointInT, PointOutT>
  {
    public:
      using Ptr = shared_ptr<ISSKeypoint3D<PointInT, PointOutT, NormalT> >;
      using ConstPtr = shared_ptr<const ISSKeypoint3D<PointInT, PointOutT, NormalT> >;

      using PointCloudIn = typename Keypoint<PointInT, PointOutT>::PointCloudIn;
      using PointCloudOut = typename Keypoint<PointInT, PointOutT>::PointCloudOut;

      using PointCloudN = pcl::PointCloud<NormalT>;
      using PointCloudNPtr = typename PointCloudN::Ptr;
      using PointCloudNConstPtr = typename PointCloudN::ConstPtr;

      using OctreeSearchIn = pcl::octree::OctreePointCloudSearch<PointInT>;
      using OctreeSearchInPtr = typename OctreeSearchIn::Ptr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::surface_;
      using Keypoint<PointInT, PointOutT>::tree_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::search_parameter_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;

      /** \brief Constructor.
        * \param[in] salient_radius the radius of the spherical neighborhood used to compute the scatter matrix.
        */
      ISSKeypoint3D (double salient_radius = 0.0001)
      : salient_radius_ (salient_radius)
      , non_max_radius_ (0.0)
      , normal_radius_ (0.0)
      , border_radius_ (0.0)
      , gamma_21_ (0.975)
      , gamma_32_ (0.975)
      , third_eigen_value_ (nullptr)
      , edge_points_ (nullptr)
      , min_neighbors_ (5)
      , normals_ (new pcl::PointCloud<NormalT>)
      , angle_threshold_ (static_cast<float> (M_PI) / 2.0f)
      , threads_ (0)
      {
        name_ = "ISSKeypoint3D";
        search_radius_ = salient_radius_;
        setNumberOfThreads(threads_); // Reset number of threads with the member's initialization value to apply input validation.
      }

      /** \brief Destructor. */
      ~ISSKeypoint3D ()
      {
        delete[] third_eigen_value_;
        delete[] edge_points_;
      }

      /** \brief Set the radius of the spherical neighborhood used to compute the scatter matrix.
        * \param[in] salient_radius the radius of the spherical neighborhood
        */
      void
      setSalientRadius (double salient_radius);

      /** \brief Set the radius for the application of the non maxima supression algorithm.
        * \param[in] non_max_radius the non maxima suppression radius
        */
      void
      setNonMaxRadius (double non_max_radius);

      /** \brief Set the radius used for the estimation of the surface normals of the input cloud. If the radius is
        * too large, the temporal performances of the detector may degrade significantly.
        * \param[in] normal_radius the radius used to estimate surface normals
        */
      void
      setNormalRadius (double normal_radius);

      /** \brief Set the radius used for the estimation of the boundary points. If the radius is too large,
        * the temporal performances of the detector may degrade significantly.
        * \param[in] border_radius the radius used to compute the boundary points
        */
      void
      setBorderRadius (double border_radius);

      /** \brief Set the upper bound on the ratio between the second and the first eigenvalue.
        * \param[in] gamma_21 the upper bound on the ratio between the second and the first eigenvalue
        */
      void
      setThreshold21 (double gamma_21);

      /** \brief Set the upper bound on the ratio between the third and the second eigenvalue.
        * \param[in] gamma_32 the upper bound on the ratio between the third and the second eigenvalue
        */
      void
      setThreshold32 (double gamma_32);

      /** \brief Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
        * \param[in] min_neighbors the minimum number of neighbors required
        */
      void
      setMinNeighbors (int min_neighbors);

      /** \brief Set the normals if pre-calculated normals are available.
        * \param[in] normals the given cloud of normals
        */
      void
      setNormals (const PointCloudNConstPtr &normals);

      /** \brief Set the decision boundary (angle threshold) that marks points as boundary or regular.
        * (default \f$\pi / 2.0\f$)
        * \param[in] angle the angle threshold
        */
      inline void
      setAngleThreshold (float angle)
      {
        angle_threshold_ = angle;
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param[in] nr_threads the number of hardware threads to use (0 sets the value back to automatic)
        */
      void
      setNumberOfThreads (unsigned int nr_threads = 0);

    protected:

      /** \brief Compute the boundary points for the given input cloud.
        * \param[in] input the input cloud
        * \param[in] border_radius the radius used to compute the boundary points
        * \param[in] angle_threshold the decision boundary that marks the points as boundary
        * \return the vector of boolean values in which the information about the boundary points is stored
        */
      bool*
      getBoundaryPoints (PointCloudIn &input, double border_radius, float angle_threshold);

      /** \brief Compute the scatter matrix for a point index.
        * \param[in] current_index the index of the point
        * \param[out] cov_m the point scatter matrix
        */
      void
      getScatterMatrix (const int &current_index, Eigen::Matrix3d &cov_m);

      /** \brief Perform the initial checks before computing the keypoints.
       *  \return true if all the checks are passed, false otherwise
        */
      bool
      initCompute () override;

      /** \brief Detect the keypoints by performing the EVD of the scatter matrix.
        * \param[out] output the resultant cloud of keypoints
        */
      void
      detectKeypoints (PointCloudOut &output) override;


      /** \brief The radius of the spherical neighborhood used to compute the scatter matrix.*/
      double salient_radius_;

      /** \brief The non maxima suppression radius. */
      double non_max_radius_;

      /** \brief The radius used to compute the normals of the input cloud. */
      double normal_radius_;

      /** \brief The radius used to compute the boundary points of the input cloud. */
      double border_radius_;

      /** \brief The upper bound on the ratio between the second and the first eigenvalue returned by the EVD. */
      double gamma_21_;

      /** \brief The upper bound on the ratio between the third and the second eigenvalue returned by the EVD. */
      double gamma_32_;

      /** \brief Store the third eigen value associated to each point in the input cloud. */
      double *third_eigen_value_;

      /** \brief Store the information about the boundary points of the input cloud. */
      bool *edge_points_;

      /** \brief Minimum number of neighbors that has to be found while applying the non maxima suppression algorithm. */
      int min_neighbors_;

      /** \brief The cloud of normals related to the input surface. */
      PointCloudNConstPtr normals_;

      /** \brief The decision boundary (angle threshold) that marks points as boundary or regular. (default \f$\pi / 2.0\f$) */
      float angle_threshold_;

      /** \brief The number of threads that has to be used by the scheduler. */
      unsigned int threads_;

  };

}

#include <pcl/keypoints/impl/iss_3d.hpp>
