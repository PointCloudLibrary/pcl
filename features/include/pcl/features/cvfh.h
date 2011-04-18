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

#ifndef PCL_FEATURES_CVFH_H_
#define PCL_FEATURES_CVFH_H_

#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

namespace pcl
{
  /** \brief @b CVFHEstimation estimates the <b>Clustered Viewpoint Feature Histogram (CVFH)</b> descriptor for a given point cloud
    * dataset containing points and normals.
    */
  template <typename PointInT, typename PointNT, typename PointOutT>
  class CVFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename pcl::KdTree<PointNormal>::Ptr KdTreePtr;
      typedef typename pcl::NormalEstimation<PointNormal, PointNormal> NormalEstimator;
      typedef typename pcl::VFHEstimation<PointInT, PointNT, pcl::VFHSignature308> VFHEstimator;

      /** \brief Empty constructor. */
      CVFHEstimation () : vpx_ (0), vpy_ (0), vpz_ (0), leaf_size_ (0.005), curv_threshold_ (0.03), cluster_tolerance_ (leaf_size_ * 3), eps_angle_threshold_ (0.125), min_percent_size_ (0.20)
      {
        search_radius_ = 0;
        k_ = 1;
        feature_name_ = "CVFHEstimation";
      };

      /** \brief Removes normals with high curvature caused by real edges or noisy data
        * \param cloud pointcloud to be filtered
        * \param indices_out the indices of the points with higher curvature than threshold
        * \param indices_in the indices of the remaining points after filtering
        * \param threshold threshold value for curvature
        */
      void
      filterNormalsWithHighCurvature (const pcl::PointCloud<PointNT> & cloud, 
                                      std::vector<int> & indices_out,
                                      std::vector<int> & indices_in, float threshold);

      /** \brief Set the viewpoint.
        * \param vpx the X coordinate of the viewpoint
        * \param vpy the Y coordinate of the viewpoint
        * \param vpz the Z coordinate of the viewpoint
        */
      inline void
      setViewPoint (float vpx, float vpy, float vpz)
      {
        vpx_ = vpx;
        vpy_ = vpy;
        vpz_ = vpz;
      }

      /** \brief Get the viewpoint. 
        * \param vpx the X coordinate of the viewpoint
        * \param vpy the Y coordinate of the viewpoint
        * \param vpz the Z coordinate of the viewpoint
        */
      inline void
      getViewPoint (float &vpx, float &vpy, float &vpz)
      {
        vpx = vpx_;
        vpy = vpy_;
        vpz = vpz_;
      }

      /** \brief Get the centroids used to compute different CVFH descriptors
        *  \param centroids vector to hold the centroids
        */
      inline void
      getCentroidClusters (std::vector<Eigen::Vector3f> & centroids) 
      {
        for(size_t i = 0; i < centroids_dominant_orientations_.size (); ++i)
          centroids.push_back (centroids_dominant_orientations_[i]);
      }

    private:
      /** \brief Values describing the viewpoint ("pinhole" camera model assumed). 
        * By default, the viewpoint is set to 0,0,0. 
        */
      float vpx_, vpy_, vpz_;

      /** \brief Size of the voxels after voxel gridding. IMPORTANT: Must match the voxel 
        * size of the training data 
        */
      float leaf_size_;

      /** \brief Curvature threshold for removing normals */
      float curv_threshold_;

      /** \brief allowed Euclidean distance between points to be added to the cluster */
      float cluster_tolerance_;

      /** \brief deviation of the normals between two points so they can be clustered 
        * together 
        */
      float eps_angle_threshold_;

      /** \brief Percentage of points in a region to be considered stable for CVFH 
        * computation 
        */
      float min_percent_size_;

      /** \brief Estimate the Clustered Viewpoint Feature Histograms (CVFH) descriptors at 
        * a set of points given by <setInputCloud (), setIndices ()> using the surface in 
        * setSearchSurface ()
        * 
        * \param output the resultant point cloud model dataset that contains the CVFH 
        * feature estimates
        */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief Region growing method using Euclidean distances and neighbors normals to 
        * add points to a region.
        *  \param cloud point cloud to split into regions
        *  \param normals are the normals of cloud
        *  \param tolerance is the allowed Euclidean distance between points to be added to 
        *  the cluster
        *  \param tree is the spatial search structure for nearest neighbour search
        *  \param clusters vector of indices representing the clustered regions
        *  \param eps_angle deviation of the normals between two points so they can be 
        *  clustered together
        *  \param min_pts_per_cluster minimum cluster size. (default: 1 point)
        *  \param max_pts_per_cluster maximum cluster size. (default: all the points)
        */
      void
      extractEuclideanClustersSmooth (
          const pcl::PointCloud<pcl::PointNormal> &cloud, 
          const pcl::PointCloud<pcl::PointNormal> &normals,
          float tolerance, const pcl::KdTree<pcl::PointNormal>::Ptr &tree,
          std::vector<pcl::PointIndices> &clusters, double eps_angle,
          unsigned int min_pts_per_cluster = 1,
          unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

    protected:
      /** \brief Centroids that were used to compute different CVFH descriptors */
      std::vector<Eigen::Vector3f> centroids_dominant_orientations_;
  };
}

#endif  //#ifndef PCL_FEATURES_VFH_H_
