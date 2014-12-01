/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FEATURES_CVFH_H_
#define PCL_FEATURES_CVFH_H_

#include <pcl/features/feature.h>
#include <pcl/features/vfh.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/common.h>

namespace pcl
{
  /** \brief CVFHEstimation estimates the Clustered Viewpoint Feature Histogram (CVFH) descriptor for a given 
    * point cloud dataset containing XYZ data and normals, as presented in: 
    *   - CAD-Model Recognition and 6 DOF Pose Estimation
    *     A. Aldoma, N. Blodow, D. Gossow, S. Gedikli, R.B. Rusu, M. Vincze and G. Bradski
    *     ICCV 2011, 3D Representation and Recognition (3dRR11) workshop
    *     Barcelona, Spain, (2011) 
    *
    * The suggested PointOutT is pcl::VFHSignature308.
    *
    * \author Aitor Aldoma
    * \ingroup features
    */
  template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
  class CVFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<CVFHEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const CVFHEstimation<PointInT, PointNT, PointOutT> > ConstPtr;

      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename pcl::search::Search<PointNormal>::Ptr KdTreePtr;
      typedef typename pcl::VFHEstimation<PointInT, PointNT, pcl::VFHSignature308> VFHEstimator;

      /** \brief Empty constructor. */
      CVFHEstimation () :
        vpx_ (0), vpy_ (0), vpz_ (0), 
        leaf_size_ (0.005f), 
        normalize_bins_ (false),
        curv_threshold_ (0.03f), 
        cluster_tolerance_ (leaf_size_ * 3), 
        eps_angle_threshold_ (0.125f), 
        min_points_ (50),
        radius_normals_ (leaf_size_ * 3),
        centroids_dominant_orientations_ (),
        dominant_normals_ ()
      {
        search_radius_ = 0;
        k_ = 1;
        feature_name_ = "CVFHEstimation";
      }
      ;

      /** \brief Removes normals with high curvature caused by real edges or noisy data
        * \param[in] cloud pointcloud to be filtered
        * \param[in] indices_to_use the indices to use
        * \param[out] indices_out the indices of the points with higher curvature than threshold
        * \param[out] indices_in the indices of the remaining points after filtering
        * \param[in] threshold threshold value for curvature
        */
      void
      filterNormalsWithHighCurvature (const pcl::PointCloud<PointNT> & cloud, std::vector<int> & indices_to_use, std::vector<int> &indices_out,
                                      std::vector<int> &indices_in, float threshold);

      /** \brief Set the viewpoint.
        * \param[in] vpx the X coordinate of the viewpoint
        * \param[in] vpy the Y coordinate of the viewpoint
        * \param[in] vpz the Z coordinate of the viewpoint
        */
      inline void
      setViewPoint (float vpx, float vpy, float vpz)
      {
        vpx_ = vpx;
        vpy_ = vpy;
        vpz_ = vpz;
      }

      /** \brief Set the radius used to compute normals
        * \param[in] radius_normals the radius
        */
      inline void
      setRadiusNormals (float radius_normals)
      {
        radius_normals_ = radius_normals;
      }

      /** \brief Get the viewpoint. 
        * \param[out] vpx the X coordinate of the viewpoint
        * \param[out] vpy the Y coordinate of the viewpoint
        * \param[out] vpz the Z coordinate of the viewpoint
        */
      inline void
      getViewPoint (float &vpx, float &vpy, float &vpz)
      {
        vpx = vpx_;
        vpy = vpy_;
        vpz = vpz_;
      }

      /** \brief Get the centroids used to compute different CVFH descriptors
        * \param[out] centroids vector to hold the centroids
        */
      inline void
      getCentroidClusters (std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > & centroids)
      {
        for (size_t i = 0; i < centroids_dominant_orientations_.size (); ++i)
          centroids.push_back (centroids_dominant_orientations_[i]);
      }

      /** \brief Get the normal centroids used to compute different CVFH descriptors
        * \param[out] centroids vector to hold the normal centroids
        */
      inline void
      getCentroidNormalClusters (std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > & centroids)
      {
        for (size_t i = 0; i < dominant_normals_.size (); ++i)
          centroids.push_back (dominant_normals_[i]);
      }

      /** \brief Sets max. Euclidean distance between points to be added to the cluster 
        * \param[in] d the maximum Euclidean distance 
        */

      inline void
      setClusterTolerance (float d)
      {
        cluster_tolerance_ = d;
      }

      /** \brief Sets max. deviation of the normals between two points so they can be clustered together
        * \param[in] d the maximum deviation 
        */
      inline void
      setEPSAngleThreshold (float d)
      {
        eps_angle_threshold_ = d;
      }

      /** \brief Sets curvature threshold for removing normals
        * \param[in] d the curvature threshold 
        */
      inline void
      setCurvatureThreshold (float d)
      {
        curv_threshold_ = d;
      }

      /** \brief Set minimum amount of points for a cluster to be considered
        * \param[in] min the minimum amount of points to be set 
        */
      inline void
      setMinPoints (size_t min)
      {
        min_points_ = min;
      }

      /** \brief Sets wether if the CVFH signatures should be normalized or not
        * \param[in] normalize true if normalization is required, false otherwise 
        */
      inline void
      setNormalizeBins (bool normalize)
      {
        normalize_bins_ = normalize;
      }

      /** \brief Overloaded computed method from pcl::Feature.
        * \param[out] output the resultant point cloud model dataset containing the estimated features
        */
      void
      compute (PointCloudOut &output);

    private:
      /** \brief Values describing the viewpoint ("pinhole" camera model assumed). 
        * By default, the viewpoint is set to 0,0,0.
        */
      float vpx_, vpy_, vpz_;

      /** \brief Size of the voxels after voxel gridding. IMPORTANT: Must match the voxel 
        * size of the training data or the normalize_bins_ flag must be set to true.
        */
      float leaf_size_;

      /** \brief Wether to normalize the signatures or not. Default: false. */
      bool normalize_bins_;

      /** \brief Curvature threshold for removing normals. */
      float curv_threshold_;

      /** \brief allowed Euclidean distance between points to be added to the cluster. */
      float cluster_tolerance_;

      /** \brief deviation of the normals between two points so they can be clustered together. */
      float eps_angle_threshold_;

      /** \brief Minimum amount of points in a clustered region to be considered stable for CVFH
        * computation.
        */
      size_t min_points_;

      /** \brief Radius for the normals computation. */
      float radius_normals_;

      /** \brief Estimate the Clustered Viewpoint Feature Histograms (CVFH) descriptors at 
        * a set of points given by <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface ()
        *
        * \param[out] output the resultant point cloud model dataset that contains the CVFH
        * feature estimates
        */
      void
      computeFeature (PointCloudOut &output);

      /** \brief Region growing method using Euclidean distances and neighbors normals to 
        * add points to a region.
        * \param[in] cloud point cloud to split into regions
        * \param[in] normals are the normals of cloud
        * \param[in] tolerance is the allowed Euclidean distance between points to be added to
        * the cluster
        * \param[in] tree is the spatial search structure for nearest neighbour search
        * \param[out] clusters vector of indices representing the clustered regions
        * \param[in] eps_angle deviation of the normals between two points so they can be
        * clustered together
        * \param[in] min_pts_per_cluster minimum cluster size. (default: 1 point)
        * \param[in] max_pts_per_cluster maximum cluster size. (default: all the points)
        */
      void
      extractEuclideanClustersSmooth (const pcl::PointCloud<pcl::PointNormal> &cloud,
                                      const pcl::PointCloud<pcl::PointNormal> &normals, float tolerance,
                                      const pcl::search::Search<pcl::PointNormal>::Ptr &tree,
                                      std::vector<pcl::PointIndices> &clusters, double eps_angle,
                                      unsigned int min_pts_per_cluster = 1,
                                      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

    protected:
      /** \brief Centroids that were used to compute different CVFH descriptors */
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > centroids_dominant_orientations_;
      /** \brief Normal centroids that were used to compute different CVFH descriptors */
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > dominant_normals_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/cvfh.hpp>
#endif

#endif  //#ifndef PCL_FEATURES_CVFH_H_
