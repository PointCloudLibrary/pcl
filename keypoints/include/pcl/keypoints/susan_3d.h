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
 */

#ifndef PCL_SUSAN_KEYPOINT_3D_H_
#define PCL_SUSAN_KEYPOINT_3D_H_

#include <pcl/keypoints/keypoint.h>

namespace pcl
{
  /** \brief SusanKeypoint3D exploitss the idea behind SUSAN detector, but instead of relying
    * on image intensity, it uses surface normals.
    * It should be faster than HarrisKeypoint3D since it computes first order statistics.
    * Behaviour is slightly different when input cloud is organized and not.
    *
    * \author Nizar Sallem
    * \ingroup keypoints
    */
  template <typename PointInT, typename PointOutT, typename NormalT = pcl::Normal>
  class SusanKeypoint3D : public Keypoint<PointInT, PointOutT>
  {
    public:
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudIn PointCloudIn;
      typedef typename Keypoint<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename Keypoint<PointInT, PointOutT>::KdTree KdTree;
      typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
      typedef typename pcl::PointCloud<NormalT> Normals;
      typedef typename Normals::Ptr NormalsPtr;
      typedef typename Normals::ConstPtr NormalsConstPtr;

      using Keypoint<PointInT, PointOutT>::name_;
      using Keypoint<PointInT, PointOutT>::input_;
      using Keypoint<PointInT, PointOutT>::indices_;
      using Keypoint<PointInT, PointOutT>::surface_;
      using Keypoint<PointInT, PointOutT>::keypoints_indices_;
      using Keypoint<PointInT, PointOutT>::search_radius_;
      using Keypoint<PointInT, PointOutT>::searchForNeighbors;
      using Keypoint<PointInT, PointOutT>::initCompute;

      /** \brief Constructor
        * \param[in] radius the radius for normal estimation as well as for non maxima
        * suppression, default 0.01m. If cloud is organized set the window size instead.
        * \param[in] distance to test if the nucleus is far enough from the centroid,
        * default 0.001. If your cloud is organized distance will be in pixels.
        * \param[in] angle to test if normals are parallel in radians. Good value should be
        * between 5 and 20 degrees, default 20 degrees.
        */
      SusanKeypoint3D (float radius = 0.01f, float distance = 0.001f, float angle = M_PI / 9)
        : distance_threshold_ (distance)
        , angular_threshold_ (angle)
        , threads_ (1)
      {
        name_ = "SusanKeypoint3D";
        search_radius_ = radius;
        window_size_ = 7;
        test_distance_ = true;
        test_contiguity_ = false;
        nonmax_ = true;
      }

      /** \brief set the radius for normal estimation and non maxima supression.
        * \param[in] radius
        */
      void
      setRadius (float radius) { search_radius_ = radius; }

      /// \brief Set window size, effective only if cloud is organized, default 7
      inline void
      setWindowSize (int window_size) { window_size_= window_size; }

      /// \brief \return window size i.e. window width and height
      inline int
      getWindowSize () const { return (window_size_); }

      /** \brief Set the minimal distance between nucleus and centroid to reject false
        * positive. This is only evaluated if distance test is turned on.
        * \param[in] distance minimal distance between centroid and nucleus.
        */
      void
      setDistanceThreshold (float distance) { distance_threshold_ = distance; }

      /// \brief \return distance threshold
      float
      getDistanceThreshold () const { return (distance_threshold_); }

      /** \brief set the angular threshold value for detecting corners. Normals are
        * considered as  parallel if angle (Ni, Nj) <= angular_threshold.
        * \param[in] angular_threshold
        */
      void
      setAngularThreshold (float angle) { angular_threshold_ = angle; }

      /// \brief \return angular threshold
      float
      getAngularThreshold () const { return (angular_threshold_); }

      /** \brief set normals if precalculated normals are available.
        * \param normals
        */
      void
      setNormals (const NormalsConstPtr &normals) { normals_.reset (normals); }

      /// \brief \return points normals as calculated or given
      inline void
      getNormals (const NormalsConstPtr &normals) const { return (normals_); }

      virtual void
      setSearchSurface (const PointCloudInConstPtr &cloud) { surface_ = cloud; normals_.reset (); }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (0 sets the value back to
        * automatic), default 1.
        */
      void
      setNumberOfThreads (int nr_threads) { threads_ = (nr_threads < 0) ? 1 : nr_threads; }

      /// \brief \return the number of threads
      inline unsigned int
      getNumberOfThreads () const { return (threads_); }

      /** \brief test that a point in the USAN is part of the [nucleus centroid] to filter
        * out false positives. I recommend not setting it to true unless cloud is organized
        * or really dense, default is false.
        * \param[in] test_contiguity whether or not to test for continguity
        */
      void
      setTestForContiguity (bool test_contiguity) { test_contiguity_ = test_contiguity; }

      /** \brief test if the nucleus and the centrooid are far enough to filter out false
        * positives, default true.
        * \param[in] test_distance whether or not to test for distance
        */
      void
      setTestForDistance (bool test_distance) { test_distance_ = test_distance; }

    protected:
      bool
      initCompute ();

      void
      detectKeypoints (PointCloudOut &output);

    private:
      /// detect keypoints on organized input
      void
      detectKeypointsOrganized (PointCloudOut &output);
      /// detect keypoints on non organized input
      void
      detectKeypointsNonOrganized (PointCloudOut &output);
      /** \return a const reference to the normal at (i,j) if finite else return a reference
        * to a null normal.
        * If the returned normal is valid \a counter is incremented.
        */
      inline const NormalT&
      getNormalOrNull (std::size_t pos, int& counter) const;
      /** \return a const reference to the normal at (i,j) if finite else return a reference
        * to a null normal.
        * If the returned normal is valid \a counter is incremented.
        */
      inline const NormalT&
      getNormalOrNull (int u, int v, int& counter) const;
      /// \return difference of two normals vectors
      inline float
      normalsDiff (const NormalT& a, const NormalT& b) const;
      /** get pixels indices that form a line segment between nucleus and centroid using 
        * Bresenheim algorithm within the search window.
        * param[in] centroid_x the centroid coordinate on x axis 
        * param[in] centroid_y the centroid coordinate on y axis 
        * param[out] points indices of points lying on [nucleus centroid] segment 
        */
      void
      lineToCentroid (int centroid_x, int centroid_y, std::vector<Eigen::Vector2i>& points);
      /// comparator for responses intensity
      inline bool
      greaterCornernessAtIndices (int a, int b, const pcl::PointCloud<float>::ConstPtr &response) const
      {
        return (response->points [a] > response->points [b]);
      }
      /// Window size
      int window_size_;
      /// half window size
      int half_window_size_;
      /// nucleus to centroid distance threshold
      float distance_threshold_;
      /// angular threshold between normals
      float angular_threshold_;
      /// threshold computed from angular_threshold_
      float threshold_;
      /// pointer to normals
      NormalsConstPtr normals_;
      /// number of threads
      int threads_;
      /// test for distance between centroid and nucleus
      bool test_distance_;
      /// test for contiguity inside USAN
      bool test_contiguity_;
      /// non maximal suppression
      bool nonmax_;
  };
}

#include <pcl/keypoints/impl/susan_3d.hpp>

#endif // #ifndef PCL_SUSAN_KEYPOINT_3D_H_
