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

#ifndef DOMINANT_PLANE_SEGMENTATION_H_
#define DOMINANT_PLANE_SEGMENTATION_H_

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/pcl_search.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl
{
  namespace apps
  {
    /** \brief @b DominantPlaneSegmentation performs euclidean segmentation on a scene assuming that a dominant plane exists.
       * \author Aitor Aldoma
       * \ingroup apps
       */

    template<typename PointType>
      class PCL_EXPORTS DominantPlaneSegmentation
      {
      public:
        typedef pcl::PointCloud<PointType> Cloud;
        typedef typename Cloud::Ptr CloudPtr;
        typedef typename Cloud::ConstPtr CloudConstPtr;
        typedef typename pcl::search::KdTree<PointType>::Ptr KdTreePtr;

        DominantPlaneSegmentation ()
        {
          min_z_bounds_ = 0;
          max_z_bounds_ = 1.5;
          object_min_height_ = 0.01;
          object_max_height_ = 0.7;
          object_cluster_tolerance_ = 0.05f;
          object_cluster_min_size_ = 500;
          k_ = 50;
          sac_distance_threshold_ = 0.01;
          downsample_leaf_ = 0.005f;
          wsize_ = 5;
        }

        /* \brief Extract the clusters.
         * \param clusters Clusters extracted from the initial point cloud at the resolution size
         * specified by downsample_leaf_
         */
        void
        compute (std::vector<CloudPtr> & clusters);

        /* \brief Extract the clusters.
         * \param clusters Clusters extracted from the initial point cloud. The returned
         * clusters are not downsampled.
         */
        void
        compute_full (std::vector<CloudPtr> & clusters);

        /* \brief Extract clusters on a plane using connected components on an organized pointcloud.
         * The method expects a the input cloud to have the is_dense attribute set to false.
          * \param clusters Clusters extracted from the initial point cloud. The returned
          * clusters are not downsampled.
          */
        void
        compute_fast (std::vector<CloudPtr> & clusters);

        /* \brief Computes the table plane.
         */
        void
        compute_table_plane();

        /* \brief Sets the input point cloud.
         * \param cloud_in The input point cloud.
         */
        void
        setInputCloud (CloudPtr & cloud_in)
        {
          input_ = cloud_in;
        }

        /* \brief Returns the table coefficients after computation
         * \param model represents the normal and the position of the plane (a,b,c,d)
         */
        void
        getTableCoefficients (Eigen::Vector4f & model)
        {
          model = table_coeffs_;
        }

        /* \brief Sets minimum distance between clusters
         * \param d distance (in meters)
         */
        void
        setDistanceBetweenClusters (float d) 
        {
          object_cluster_tolerance_ = d;
        }

        /* \brief Sets minimum size of the clusters.
         * \param size number of points
         */
        void 
        setMinClusterSize (int size) 
        {
          object_cluster_min_size_ = size;
        }

        /* \brief Sets the min height of the clusters in order to be considered.
         * \param h minimum height (in meters)
         */
        void
        setObjectMinHeight (double h)
        {
          object_min_height_ = h;
        }

        /* \brief Sets the max height of the clusters in order to be considered.
         * \param h max height (in meters)
         */
        void
        setObjectMaxHeight (double h)
        {
          object_max_height_ = h;
        }

        /* \brief Sets minimum distance from the camera for a point to be considered.
         * \param z distance (in meters)
         */
        void
        setMinZBounds (double z)
        {
          min_z_bounds_ = z;
        }
        /* \brief Sets maximum distance from the camera for a point to be considered.
         * \param z distance (in meters)
         */
        void
        setMaxZBounds (double z)
        {
          max_z_bounds_ = z;
        }

        /* \brief Sets the number of neighbors used for normal estimation.
         * \param k number of neighbors
         */
        void setKNeighbors(int k) {
          k_ = k;
        }

        /* \brief Set threshold for SAC plane segmentation
         * \param d threshold (in meters)
         */
        void setSACThreshold(double d) {
          sac_distance_threshold_ = d;
        }

        /* \brief Set downsampling resolution.
         * \param d resolution (in meters)
         */
        void 
        setDownsamplingSize (float d) 
        {
          downsample_leaf_ = d;
        }

        /* \brief Set window size in pixels for CC used in compute_fast method
         * \param w window size (in pixels)
         */
        void setWSize(int w) {
         wsize_ = w;
        }

        /* \brief Returns the indices of the clusters found by the segmentation
         * NOTE: This function returns only valid indices if the compute_fast method is used
         * \param indices indices of the clusters
         */
        void getIndicesClusters(std::vector<pcl::PointIndices> & indices) {
          indices = indices_clusters_;
        }

      private:

        int
        check (pcl::PointXYZI & p1, pcl::PointXYZI & p2, float, float max_dist)
        {
          if (p1.intensity == 0) //new label
            return 1;
          else
          {
            //compute distance and check aginst max_dist
            if ((p1.getVector3fMap () - p2.getVector3fMap ()).norm () <= max_dist)
            {
              p2.intensity = p1.intensity;
              return 0;
            }
            else //new label
              return 1;
          }
        }

        //components needed for cluster segmentation and plane extraction
        pcl::PassThrough<PointType> pass_;
        pcl::VoxelGrid<PointType> grid_;
        pcl::NormalEstimation<PointType, pcl::Normal> n3d_;
        pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg_;
        pcl::ProjectInliers<PointType> proj_;
        pcl::ProjectInliers<PointType> bb_cluster_proj_;
        pcl::ConvexHull<PointType> hull_;
        pcl::ExtractPolygonalPrismData<PointType> prism_;
        pcl::EuclideanClusterExtraction<PointType> cluster_;

        /** \brief Input cloud from which to extract clusters */
        CloudPtr input_;
        /** \brief Table coefficients (a,b,c,d) */
        Eigen::Vector4f table_coeffs_;
        /** \brief Downsampling resolution. */
        float downsample_leaf_;
        /** \brief Number of neighbors for normal estimation */
        int k_;
        /** \brief Keep points farther away than min_z_bounds */
        double min_z_bounds_;
        /** \brief Keep points closer than max_z_bounds */
        double max_z_bounds_;
        /** \brief Threshold for SAC plane segmentation */
        double sac_distance_threshold_;
        /** \brief Min height from the table plane object points will be considered from */
        double object_min_height_;
        /** \brief Max height from the table plane */
        double object_max_height_;
        /** \brief Tolerance between different clusters */
        float object_cluster_tolerance_;
        /** \brief Minimum size for a cluster, clusters smaller than this won't be returned */
        int object_cluster_min_size_;
        /** \brief Window size in pixels for CC in compute_fast method */
        int wsize_;
        /** \brief Indices of the clusters to the main cloud found by the segmentation */
        std::vector<pcl::PointIndices> indices_clusters_;

      };
  }
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/apps/impl/dominant_plane_segmentation.hpp>
#endif

#endif /* DOMINANT_PLANE_SEGMENTATION_H_ */
