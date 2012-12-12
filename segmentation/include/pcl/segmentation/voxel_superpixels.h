/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 */

#ifndef PCL_VOXEL_SUPERPIXELS_H_
#define PCL_VOXEL_SUPERPIXELS_H_

#include <pcl/pcl_base.h>
#include <pcl/octree/octree.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

namespace pcl
{
  /** \brief Implements a superpixel algorithm based on voxel structure, normals, 
    * and rgb values
    * \author Jeremie Papon (jpapon@gmail.com)
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
      VoxelSuperpixels (float voxel_resolution, float seed_resolution);

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding neighbors. In other words it frees memory.
        */
      virtual
      ~VoxelSuperpixels ();

      /** \brief Set the resolution of the octree voxels */
      void
      setVoxelResolution (float resolution);
      
      /** \brief Get the resolution of the octree voxels */
      float 
      getVoxelResolution () const;
      
      /** \brief Set the radius used for normals */
      void
      setNormalRadius (float radius);
      
      /** \brief Get the radius used for normals */
      float 
      getNormalRadius () const;
      
            /** \brief Set the radius used for FPFH features */
      void
      setFPFHRadius (float radius);
      
      /** \brief Get the radius used for FPFH features */
      float 
      getFPFHRadius () const;
      
      /** \brief Set the resolution of the octree seed voxels */
      void
      setSeedResolution (float seed_resolution);
      
      /** \brief Get the resolution of the octree seed voxels */
      float 
      getSeedResolution () const;
      
      
      /** \brief Get the size of the voxel cloud */
      int 
      getVoxelCloudSize () const;
      
      /** \brief Returns the voxel cloud */
      typename pcl::PointCloud<PointT>::Ptr
      getVoxelCloud ();
      
      /** \brief Set the importance of color for supervoxels */
      void
      setColorImportance (float val);
      
      /** \brief Set the importance of spatial distance for supervoxels */
      void
      setSpatialImportance (float val);
      
      /** \brief Set the importance of 3D shape for supervoxels */
      void
      setFPFHImportance (float val);
      
      /** \brief Set the importance of scalar normal product for supervoxels */
      void
      setNormalImportance (float val);
      
      /** \brief Returns normals calculated in the Voxel Cloud. */
      pcl::PointCloud<pcl::Normal>::ConstPtr
      getNormals () const;

      /** \brief This method launches the segmentation algorithm and returns the superpixels that were
       * obtained during the segmentation.
       * \param[out] voxel_cloud voxelized cloud which was segmented
       * \param[out] superpixels superpixels that were obtained. Each superpixel is an array of point indices in the voxelized cloud.
       */
      virtual void
      extract (typename pcl::PointCloud<PointT>::Ptr &voxel_cloud, std::vector<pcl::PointIndices>& superpixels);

      ////////////////////////////////////////////////////////////
      /** \brief Compute hierarchical segmentation tree from superpixels 
       *  This is similar to the method of Felzenszwalb and Huttenlocher, 
       *  "Efficient Graph-Based Image Segmentation".
       *  It computes a hierarchical segmentation tree starting with the
       *  supervoxels by iteratively merging them.
       */
      void
      computeHierarchicalSegmentation ();
      
      ////////////////////////////////////////////////////////////
      /** \brief Returns an RGB colorized cloud showing superpixels
        * Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
        */
      typename pcl::PointCloud<PointXYZRGB>::Ptr
      getColoredCloud ();
      
      /** \brief Returns labeled cloud
        * Points that belong to the same segment have the same label.
        * Labels for segments start from 1, unlabled points have label 0
        */
      typename pcl::PointCloud<PointXYZL>::Ptr
      getLabeledCloud ();
      
      /** \brief Returns an RGB colorized voxelized cloud showing superpixels
       * Otherwise it returns an empty pointer.
       * Points that belong to the same segment have the same color.
       * But this function doesn't guarantee that different segments will have different
       * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
       */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredVoxelCloud ();
      
      /** \brief Returns labeled voxelized cloud
       * Points that belong to the same segment have the same label.
       * Labels for segments start from 1, unlabled points have label 0
       */      
      pcl::PointCloud<pcl::PointXYZL>::Ptr
      getLabeledVoxelCloud ();
      
            
      /** \brief Function for testing - returns an RGB labeled cloud showing seeds and search*/
      typename pcl::PointCloud<PointXYZRGB>::Ptr
      getSeedCloud ();
      
      /** \brief Function for testing - returns an RGB labeled cloud showing seeds and search*/
      typename pcl::PointCloud<PointXYZRGB>::Ptr
      getVoxelSeedCloud ();

      /** \brief Get a vector of the sets of neighbors for each superpixel */
      void
      getSuperpixelNeighbors (std::vector <std::set<int> > &superpixel_neighbors);
      
      /** \brief Get a vector of points specifying the spatial "center" of each superpixel */
      void 
      getSuperpixelCenters (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > &centers);
      
      /** \brief Get the adjacency list of voxels 
          adjacency_list[0..voxel_cloud_size][0..n neighbors]
          This gives you the indices of the neighbors of voxels
      */
      void 
      getAdjacencyList (std::vector<std::vector<int> > &adjacency_list);
      
      
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
      
      /** \brief Recursive function used to find all neighbors for a seed 
        * \param[in] index
        * \param[out] possible_constituents
        * \param[out] constituents
        */
      void 
      recursiveFind (int index, 
                     std::vector<int> &possible_constituents,
                     std::vector<int> &constituents);
      
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
      
      /** \brief Returns the abs value of scalar product of normals a and b */
      float 
      calcNormalScalarProduct (int index_a, int index_b);
      
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
      
      /** \brief This performs one iteration of evolving the superpixels */
      void
      iterateSuperpixelClusters (int max_depth);
      
      /** \brief Updates the values of the superpixel features */
      void 
      updateSuperpixelClusters ();
      
      /** \brief This computes the voxelized cloud */
      void 
      computeVoxelCloud (typename pcl::PointCloud<PointT>::ConstPtr input, typename pcl::PointCloud<PointT>::Ptr &output, typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr &octree,float resolution);
      
      /** \brief This computes the normals on the voxelized cloud */
      void 
      computeNormals ();

      /** \brief This function eliminates unnecessary labels, ie seeds that have no points
       */
      void
      cleanSuperpixels ();
      
      /** \brief Stores the resolution used in the octree */
      float resolution_;
      
      /** \brief Stores the resolution used to seed the superpixels */
      float seed_resolution_;
      
      /** \brief After the iterations this will contain the superpixels. */
      std::vector<pcl::PointIndices> superpixels_;
      
      /** \brief Point labels that tells to which superpixel each point belongs. */
      std::vector<int> point_labels_;
      
      /** \brief Contains the Voxelized Cloud at voxel resolution */
      typename pcl::PointCloud<PointT>::Ptr voxel_cloud_;
      
      /** \brief Contains the Voxelized Cloud at seed resolution */
      typename pcl::PointCloud<PointT>::Ptr seed_cloud_;
      
      /** \brief Stores the feature vector for the superpixel clusters */
      boost::multi_array<float, 2> superpixel_features_;
      
      /** \brief Indices of the superpixel seed points */
      std::vector<int> seed_indices_;
      
      /** \brief Contains a KDtree for the voxelized cloud */
      typename pcl::search::KdTree<PointT>::Ptr voxel_kdtree_;
      
      /** \brief Octree Search structure with leaves at voxel resolution */
      OctreeSearchPtr input_octree_;
      
      /** \brief Octree Search structure with leaves at voxel resolution */
      OctreeSearchPtr voxel_octree_;
            
      /** \brief Octree Search structure with leaves at seed resolution */
      OctreeSearchPtr seed_octree_;
      
  private:
   
      /** \brief Stores the radius used for calculating normals, default resolution_*2.0 */
      float normal_radius_;
      
      /** \brief Stores the radius used for calculating fpfh features, default resolution_*2.0 */
      float fpfh_radius_;

      /** \brief Importance of color in clustering */
      float color_importance_;
      /** \brief Importance of distance from seed center in clustering */
      float spatial_importance_;
      /** \brief Importance of similarity in 3D shape for clustering */
      float fpfh_importance_;
      /** \brief Importance of similarity in normals for clustering */
      float normal_importance_;
      
      /** \brief Contains normals of the points that will be segmented. */
      pcl::PointCloud<pcl::Normal>::Ptr normals_;

      /** \brief Contains the FPFH values for Voxel cloud */
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr voxel_fpfh_;
      
      /** \brief Indices of the superpixel seed points before they are adjusted */
      std::vector<int> seed_indices_orig_;
      
      /** \brief Indices of the superpixel seed points after adjustment */
      std::vector<int> seed_indices_unshifted_;
      
      /** \brief Contains vector of each point's neighbors in pair (idx, color dist) form */
      std::vector<std::vector<std::pair<int, float> > > point_neighbor_dist_;
      
      /** \brief Stores the CIELab values of the voxel points  */
      boost::multi_array<float, 2> voxel_LAB_;
      
      /** \brief Stores the constituents for seeds and their distance from seed center*/
      std::vector<std::vector<int> > seed_constituents_;
      
      /** \brief Stores the current vote and certainty for voxels */
      std::vector <std::pair<int, float> > voxel_votes_;
      
      /** \brief Stores the colors used for the superpixel labels*/
      std::vector<uint32_t> superpixel_colors_;
      
      /** \brief Stores a remapping of labels once they are cleaned */
      std::vector<int> label_remapping_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/voxel_superpixels.hpp>
#endif

#endif
