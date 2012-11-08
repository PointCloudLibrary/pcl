 
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Jeremie Papon
 * Email  : jpapon@gmail.com
 *
 */

#ifndef PCL_VOXEL_SUPERPIXELS_H_
#define PCL_VOXEL_SUPERPIXELS_H_

#include <pcl/pcl_base.h>
#include <pcl/octree/octree.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <list>
#include <math.h>
#include <time.h>

namespace pcl
{
  /** \brief
    * Implements a superpixel algorithm based on voxel structure, normals, and rgb values
    */
  template <typename PointT>
  class PCL_EXPORTS VoxelSuperpixels : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::octree::OctreePointCloudSearch <PointT> OctreeSearch;
      typedef typename OctreeSearch::Ptr OctreeSearchPtr;

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      VoxelSuperpixels ();

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding neighbors. In other words it frees memory.
        */
      virtual
      ~VoxelSuperpixels ();

      /** \brief Get the minimum number of points that a Superpixel needs to contain in order to be considered valid. */
      int
      getMinSuperpixelSize () const;

      /** \brief Set the minimum number of points that a Superpixel needs to contain in order to be considered valid. */
      void
      setMinSuperpixelSize (int min_superpixel_size);

      /** \brief Get the maximum number of points that a Superpixel needs to contain in order to be considered valid. */
      int
      getMaxSuperpixelSize () const;

      /** \brief Set the maximum number of points that a Superpixel needs to contain in order to be considered valid. */
      void
      setMaxSuperpixelSize (int max_superpixel_size);

      /** \brief Set the resolution of the octree voxels */
      void
      setVoxelResolution (double resolution);
      
      /** \brief Get the resolution of the octree voxels */
      double 
      getVoxelResolution () const;
      
      /** \brief Set the resolution of the octree seed voxels */
      void
      setSeedResolution (double seed_resolution);
      
      /** \brief Get the resolution of the octree seed voxels */
      double 
      getSeedResolution () const;
      
      /** \brief Set the importance of color for superpixels */
      void
      setColorImportance (float val);
      
      /** \brief Set the importance of spatial distance for superpixels */
      void
      setSpatialImportance (float val);
      
      /** \brief Set the importance of 3D shape for superpixels */
      void
      setFPFHImportance (float val);
      
      /** \brief Gets the vector containing the seed point indices */
      void
      getSeedIndices (std::vector<int>& seed_indices);
      
      /** \brief Returns normals. */
      pcl::PointCloud<pcl::Normal>::ConstPtr
      getNormals () const;

      
      /** \brief This method launches the segmentation algorithm and returns the superpixels that were
        * obtained during the segmentation.
        * \param[out] superpixels superpixels that were obtained. Each superpixel is an array of point indices.
        */
      virtual void
      extract (std::vector <pcl::PointIndices>& superpixels);

      /** \brief If the cloud was successfully segmented, then function
        * returns colored cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
        */
      typename pcl::PointCloud<PointXYZRGB>::Ptr
      getColoredCloud ();
      
      /** \brief If the cloud was successfully segmented, then function
        * returns labeled cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same label.
        */
      typename pcl::PointCloud<PointXYZL>::Ptr
      getLabeledCloud ();
      
      /** \brief This function cleans the superpixels
       * Eliminates small pixels, cleans up edges
       */
      void
      cleanSuperpixels ();
      
      /** \brief Function for testing - returns an RGB labeled cloud showing seeds and search*/
      typename pcl::PointCloud<PointXYZRGB>::Ptr
      getSeedCloud ();

      //TODO: MOVE BACK TO PROTECTED, ONLY HERE FOR TESTING
      /** \brief This performs one iteration of evolving the superpixels */
      void
      iterateSuperpixelClusters ();
      
    protected:

      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();

      /** \brief This method finds neighbors for each point and saves them to the array  */
      void
      findPointNeighbors ();

      /** \brief This method finds connected voxels for each seed within radius S  */
      void
      findSeedConstituency (float edge_length);
      
      /** \brief Recursive function used to find all neighbors for a seed */
      void 
      recursiveFind (int index, std::vector<int> &possible_constituents,std::vector<std::pair<int,float> > &constituents);
      
      /** \brief This method places seed voxels for superpixels based on regular grid */
      void
      placeSeedVoxels ();
      
      /** \brief Returns the difference in color between two points */
      float
      calcColorDifference (const PointT &a, const PointT &b);
      
      /** \brief Returns the difference in LAB space of voxel cloud indices a and b */
      float 
      calcColorDifferenceLAB (int index_a, int index_b);
      
      /** \brief Returns the difference in LAB space of voxel cloud indices a and b */
      float 
      calcDifferenceCurvature (int index_a, int index_b);
      
      /** \brief Returns the distance in space of voxel cloud indices a and b */
      float 
      calcDistanceSquared (int index_a, int index_b);
      
      /** \brief Calc distance between a point and a cluster center */
      float
      calcFeatureDistance (int point_index, int seed_index);
      
      /** \brief Calculates the LAB values of each voxel point, used for evolving superpixels */
      void
      calcVoxelLABValues ();
      
      /** \brief Calculates the FPFH values of each voxel point, used for evolving superpixels */
      void
      calcVoxelFPFHValues ();
      
      /** \brief Calculates gradient for a point */
      float
      calcGradient (int point_index);
      
      /** \brief This performs the superpixel evolution */
      void
      evolveSuperpixels ();
      
      /** \brief Initialize superpixels using constituents within r */
      void
      initSuperpixelClusters (float search_radius);
      
      /** \brief Updates the values of the superpixel features */
      void 
      updateSuperpixelClusters ();
      
      /** \brief This computes the voxelized cloud */
      void 
      computeVoxelCloud ();
      
      /** \brief This computes the normals on the voxelized cloud */
      void 
      computeNormals ();
      
      /** \brief Computes the labeled voxel cloud */
      void
      computeLabeledVoxelCloud ();
 
  private:
      /** \brief Stores the minimum number of points that a superpixel needs to contain in order to be considered valid. */
      int min_pts_per_superpixel_;

      /** \brief Stores the maximum number of points that a superpixel needs to contain in order to be considered valid. */
      int max_pts_per_superpixel_;

      /** \brief Stores the resolution used in the octree */
      double resolution_;
      
      /** \brief Stores the resolution used to seed the superpixels */
      double seed_resolution_;
      
      /** \brief Octree Search structure */
      OctreeSearchPtr search_;

      /** \brief Importance of color in clustering */
      float color_importance_;
      /** \brief Importance of distance from seed center in clustering */
      float spatial_importance_;
      /** \brief Importance of similarity in 3D shape for clustering */
      float fpfh_importance_;
      
      /** \brief Contains normals of the points that will be segmented. */
      pcl::PointCloud<pcl::Normal>::Ptr normals_;
      
      /** \brief Cloud which contains the output labeled pointcloud */
      pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_voxel_cloud_;
      
      /** \brief Contains the Voxelized Cloud */
      typename pcl::PointCloud<PointT>::Ptr voxel_cloud_;

      /** \brief Contains a KDtree for the voxelized cloud */
      typename pcl::search::KdTree<PointT>::Ptr voxel_kdtree_;
      
      /** \brief Contains the FPFH values for Voxel cloud */
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr voxel_fpfh_;
      
      /** \brief Indices of the superpixel seed points */
      std::vector<int> seed_indices_;
      
      /** \brief Indices of the superpixel seed points before they are adjusted */
      std::vector<int> seed_indices_orig_;
      
      /** \brief Indices of the superpixel seed points after adjustment */
      std::vector<int> seed_indices_shifted_;
      
      /** \brief Contains neighbors of each point. */
      std::vector<std::vector<int> > point_neighbors_;
     
      /** \brief Contains vector of each point's neighbors in pair (idx, dist) form */
      std::vector<std::vector<std::pair<int, float> > > point_neighbor_dist_;
      
      /** \brief Point labels that tells to which superpixel each point belongs. */
      std::vector<int> point_labels_;

      /** \brief Tells how many points each superpixel contains. Used for reserving memory. */
      std::vector<int> num_pts_in_superpixel_;

      /** \brief After the iterations this will contain the superpixels. */
      std::vector <pcl::PointIndices> superpixels_;

      /** \brief Stores the CIELab values of the voxel points  */
      boost::multi_array<float, 2> voxel_LAB_;
      
      /** \brief Stores the constituents for seeds and their distance from seed center*/
      std::vector <std::vector <std::pair<int, float> > > seed_constituents_;
      
      /** \brief Stores the current vote and certainty for voxels */
      std::vector <std::pair<int, float> > voxel_votes_;
      
      /** \brief Stores the feature vector for the superpixel clusters */
      boost::multi_array<float, 2> superpixel_features_;
      
      /** \brief Stores the number of superpixels. */
      int number_of_superpixels_;
      
      /** \brief Stores the colors used for the superpixel labels*/
      std::vector<uint32_t> superpixel_colors_;

      float max_sd_, min_sd_;
      float max_cd_, min_cd_;
      float max_fd_, min_fd_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/voxel_superpixels.hpp>
#endif

#endif
