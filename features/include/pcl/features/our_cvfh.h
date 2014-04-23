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
 * $Id: cvfh.h 4936 2012-03-07 11:12:45Z aaldoma $
 *
 */

#ifndef PCL_FEATURES_OURCVFH_H_
#define PCL_FEATURES_OURCVFH_H_

#include <pcl/features/feature.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/common.h>

namespace pcl
{
  /** \brief OURCVFHEstimation estimates the Oriented, Unique and Repetable Clustered Viewpoint Feature Histogram (CVFH) descriptor for a given
   * point cloud dataset given XYZ data and normals, as presented in:
   *     - OUR-CVFH – Oriented, Unique and Repeatable Clustered Viewpoint Feature Histogram for Object Recognition and 6DOF Pose Estimation
   *     A. Aldoma, F. Tombari, R.B. Rusu and M. Vincze
   *     DAGM-OAGM 2012
   *     Graz, Austria
   * The suggested PointOutT is pcl::VFHSignature308.
   *
   * \author Aitor Aldoma
   * \ingroup features
   */
  template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
  class OURCVFHEstimation : public FeatureFromNormals<PointInT, PointNT, PointOutT>
  {
    public:
      typedef boost::shared_ptr<OURCVFHEstimation<PointInT, PointNT, PointOutT> > Ptr;
      typedef boost::shared_ptr<const OURCVFHEstimation<PointInT, PointNT, PointOutT> > ConstPtr;
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::surface_;
      using FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
      typedef typename pcl::search::Search<PointNormal>::Ptr KdTreePtr;
      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
      /** \brief Empty constructor. */
      OURCVFHEstimation () :
        vpx_ (0), vpy_ (0), vpz_ (0), leaf_size_ (0.005f), normalize_bins_ (false), curv_threshold_ (0.03f), cluster_tolerance_ (leaf_size_ * 3),
            eps_angle_threshold_ (0.125f), min_points_ (50), radius_normals_ (leaf_size_ * 3), centroids_dominant_orientations_ (),
            dominant_normals_ ()
      {
        search_radius_ = 0;
        k_ = 1;
        feature_name_ = "OURCVFHEstimation";
        refine_clusters_ = 1.f;
        min_axis_value_ = 0.925f;
        axis_ratio_ = 0.8f;
      }
      ;

      /** \brief Creates an affine transformation from the RF axes
       * \param[in] evx the x-axis
       * \param[in] evy the z-axis
       * \param[in] evz the z-axis
       * \param[out] transformPC the resulting transformation
       * \param[in] center_mat 4x4 matrix concatenated to the resulting transformation
       */
      inline Eigen::Matrix4f
      createTransFromAxes (Eigen::Vector3f & evx, Eigen::Vector3f & evy, Eigen::Vector3f & evz, Eigen::Affine3f & transformPC,
                           Eigen::Matrix4f & center_mat)
      {
        Eigen::Matrix4f trans;
        trans.setIdentity (4, 4);
        trans (0, 0) = evx (0, 0);
        trans (1, 0) = evx (1, 0);
        trans (2, 0) = evx (2, 0);
        trans (0, 1) = evy (0, 0);
        trans (1, 1) = evy (1, 0);
        trans (2, 1) = evy (2, 0);
        trans (0, 2) = evz (0, 0);
        trans (1, 2) = evz (1, 0);
        trans (2, 2) = evz (2, 0);

        Eigen::Matrix4f homMatrix = Eigen::Matrix4f ();
        homMatrix.setIdentity (4, 4);
        homMatrix = transformPC.matrix ();

        Eigen::Matrix4f trans_copy = trans.inverse ();
        trans = trans_copy * center_mat * homMatrix;
        return trans;
      }

      /** \brief Computes SGURF and the shape distribution based on the selected SGURF
       * \param[in] processed the input cloud
       * \param[out] output the resulting signature
       * \param[in] cluster_indices the indices of the stable cluster
       */
      void
      computeRFAndShapeDistribution (PointInTPtr & processed, PointCloudOut &output, std::vector<pcl::PointIndices> & cluster_indices);

      /** \brief Computes SGURF
       * \param[in] centroid the centroid of the cluster
       * \param[in] normal_centroid the average of the normals
       * \param[in] processed the input cloud
       * \param[out] transformations the transformations aligning the cloud to the SGURF axes
       * \param[out] grid the cloud transformed internally
       * \param[in] indices the indices of the stable cluster
       */
      bool
      sgurf (Eigen::Vector3f & centroid, Eigen::Vector3f & normal_centroid, PointInTPtr & processed, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & transformations,
             PointInTPtr & grid, pcl::PointIndices & indices);

      /** \brief Removes normals with high curvature caused by real edges or noisy data
       * \param[in] cloud pointcloud to be filtered
       * \param[in] indices_to_use
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
      getCentroidClusters (std::vector<Eigen::Vector3f> & centroids)
      {
        for (size_t i = 0; i < centroids_dominant_orientations_.size (); ++i)
          centroids.push_back (centroids_dominant_orientations_[i]);
      }

      /** \brief Get the normal centroids used to compute different CVFH descriptors
       * \param[out] centroids vector to hold the normal centroids
       */
      inline void
      getCentroidNormalClusters (std::vector<Eigen::Vector3f> & centroids)
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

      /** \brief Sets wether if the signatures should be normalized or not
       * \param[in] normalize true if normalization is required, false otherwise
       */
      inline void
      setNormalizeBins (bool normalize)
      {
        normalize_bins_ = normalize;
      }

      /** \brief Gets the indices of the original point cloud used to compute the signatures
       * \param[out] indices vector of point indices
       */
      inline void
      getClusterIndices (std::vector<pcl::PointIndices> & indices)
      {
        indices = clusters_;
      }

      /** \brief Sets the refinement factor for the clusters
       * \param[in] rc the factor used to decide if a point is used to estimate a stable cluster
       */
      void
      setRefineClusters (float rc)
      {
        refine_clusters_ = rc;
      }

      /** \brief Returns the transformations aligning the point cloud to the corresponding SGURF
       * \param[out] trans vector of transformations
       */
      void
      getTransforms (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & trans)
      {
        trans = transforms_;
      }

      /** \brief Returns a boolean vector indicating of the transformation obtained by getTransforms() represents
       * a valid SGURF
       * \param[out] valid vector of booleans
       */
      void
      getValidTransformsVec (std::vector<bool> & valid)
      {
        valid = valid_transforms_;
      }

      /** \brief Sets the min axis ratio between the SGURF axes to decide if disambiguition is feasible
       * \param[in] f the ratio between axes
       */
      void
      setAxisRatio (float f)
      {
        axis_ratio_ = f;
      }

      /** \brief Sets the min disambiguition axis value to generate several SGURFs for the cluster when disambiguition is difficult
       * \param[in] f the min axis value
       */
      void
      setMinAxisValue (float f)
      {
        min_axis_value_ = f;
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

      /** \brief Factor for the cluster refinement */
      float refine_clusters_;

      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_;
      std::vector<bool> valid_transforms_;

      float axis_ratio_;
      float min_axis_value_;

      /** \brief Estimate the OUR-CVFH descriptors at
       * a set of points given by <setInputCloud (), setIndices ()> using the surface in
       * setSearchSurface ()
       *
       * \param[out] output the resultant point cloud model dataset that contains the OUR-CVFH
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
      extractEuclideanClustersSmooth (const pcl::PointCloud<pcl::PointNormal> &cloud, const pcl::PointCloud<pcl::PointNormal> &normals,
                                      float tolerance, const pcl::search::Search<pcl::PointNormal>::Ptr &tree,
                                      std::vector<pcl::PointIndices> &clusters, double eps_angle, unsigned int min_pts_per_cluster = 1,
                                      unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ());

    protected:
      /** \brief Centroids that were used to compute different OUR-CVFH descriptors */
      std::vector<Eigen::Vector3f> centroids_dominant_orientations_;
      /** \brief Normal centroids that were used to compute different OUR-CVFH descriptors */
      std::vector<Eigen::Vector3f> dominant_normals_;
      /** \brief Indices to the points representing the stable clusters */
      std::vector<pcl::PointIndices> clusters_;
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/features/impl/our_cvfh.hpp>
#endif

#endif  //#ifndef PCL_FEATURES_VFH_H_
