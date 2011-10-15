/*
 * dominant_plane_segmentation.h
 *
 *  Created on: May 30, 2011
 *      Author: aitor
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
    template<typename PointType>
      class DominantPlaneSegmentation
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
          object_cluster_tolerance_ = 0.05;
          object_cluster_min_size_ = 500;
          k_ = 50;
          sac_distance_threshold_ = 0.01;
          downsample_leaf_ = 0.005;
        }

        /* \brief Extract the clusters.
         * \param clusters Clusters extracted from the initial point cloud.
         */
        void
        compute (std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr> > & clusters);

        /* \brief Extract the clusters.
         * \param clusters Clusters extracted from the initial point cloud. The returned
         * clusters are not downsampled.
         */
        void
        compute_full (std::vector<CloudPtr, Eigen::aligned_allocator<CloudPtr> > & clusters);

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
        setDistanceBetweenClusters(double d) {
          object_cluster_tolerance_ = d;
        }

        /* \brief Sets minimum size of the clusters.
         * \param size number of points
         */
        void setMinClusterSize(int size) {
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
        void setDownsamplingSize(double d) {
          downsample_leaf_ = d;
        }

      private:
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
        double downsample_leaf_;
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
        double object_cluster_tolerance_;
        /** \brief Minimum size for a cluster, clusters smaller than this won't be returned */
        double object_cluster_min_size_;
      };
  }
}

#endif /* DOMINANT_PLANE_SEGMENTATION_H_ */
