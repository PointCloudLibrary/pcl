 
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
 * Author : jpapon@gmail.com
 * Email  : jpapon@gmail.com
 *
 */

#ifndef PCL_SEGMENTATION_SUPERVOXELS_H_
#define PCL_SEGMENTATION_SUPERVOXELS_H_

#include <pcl/pcl_base.h>
#include <pcl/octree/octree.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/octree/octree_pointcloud_supervoxel.h>

//DEBUG TODO REMOVE
#include <pcl/common/time.h>

namespace pcl
{
  
  /** \brief Implements a superpixel algorithm based on voxel structure, normals, 
    * and rgb values
    */
  template <typename PointT>
  class PCL_EXPORTS SuperVoxels : public pcl::PCLBase<PointT>
  {
    #define MAX_LABEL 16384
    
    public:
      typedef std::pair<uint32_t, PointSuperVoxel> LabelCenterT;
      typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, PointSuperVoxel, octree::EdgeProperties> VoxelAdjacencyList;
      typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
      typedef VoxelAdjacencyList::edge_descriptor EdgeID;
      

      using PCLBase <PointT>::input_;
      using PCLBase <PointT>::indices_;
      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      SuperVoxels (float voxel_resolution, float seed_resolution);

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding neighbors. In other words it frees memory.
        */
      virtual
      ~SuperVoxels ();

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
      typename pcl::PointCloud<pcl::PointSuperVoxel>::Ptr
      getVoxelCloud ();
      
      /** \brief Set the importance of color for supervoxels */
      void
      setColorImportance (float val);
      
      /** \brief Set the importance of spatial distance for supervoxels */
      void
      setSpatialImportance (float val);
            
      /** \brief Set the importance of scalar normal product for supervoxels */
      void
      setNormalImportance (float val);
      
      /** \brief This method launches the segmentation algorithm and returns the superpixels that were
       * obtained during the segmentation.
       * \param[out] voxel_cloud voxelized cloud which was segmented
       * \param[out] superpixels superpixels that were obtained. Each superpixel is an array of point indices in the voxelized cloud.
       */
      virtual void
      extract (typename pcl::PointCloud<PointSuperVoxel>::Ptr &voxel_cloud);

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

      /** \brief Gets the adjacency list which gives connections between supervoxels */
      void
      getSuperVoxelAdjacencyList (VoxelAdjacencyList &adjacency_list_arg);
      
      /** \brief Get a map of label to PointSuperVoxel specifying the "center" of each supervoxel */
      void 
      getSuperVoxelCenters (std::map<uint32_t, PointSuperVoxel> &label_centers_arg);
            
      
    protected:
      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();

      /** \brief This method places seed voxels for superpixels based on regular grid */
      void
      placeSeedVoxels ();
      
      /** \brief Calculates the LAB values of each voxel point, used for evolving superpixels */
      void
      calcVoxelLABValues ();
               
      /** \brief This performs the superpixel evolution */
      void
      evolveSuperpixels ();
      
      /** \brief Initialize superpixels using constituents within r */
      void
      initSuperpixelClusters ();
      
      /** \brief This computes the normals on the voxelized cloud */
      void 
      computeNormals ();
  
      /** \brief Stores the resolution used in the octree */
      float resolution_;
      
      /** \brief Stores the resolution used to seed the superpixels */
      float seed_resolution_;
      
      /** \brief Contains the Voxelized Cloud at voxel resolution */
      typename pcl::PointCloud<pcl::PointSuperVoxel>::Ptr voxel_cloud_;
                 
      /** \brief Indices of the superpixel seed points */
      std::vector<int> seed_indices_;
      
      /** \brief Contains a KDtree for the voxelized cloud */
      typename pcl::search::KdTree<pcl::PointSuperVoxel>::Ptr voxel_kdtree_;
      
      /** \brief Octree Search structure with leaves at voxel resolution */
      typename pcl::octree::OctreePointCloudSuperVoxel<PointT>::Ptr voxel_octree_;
      
      /** \brief Octree Search structure with leaves at seed resolution */
      pcl::octree::OctreePointCloudSearch <pcl::PointSuperVoxel>::Ptr seed_octree_;
      
  private:
   
      /** \brief Stores the radius used for calculating normals, default resolution_*2.0 */
      float normal_radius_;
      
      /** \brief Importance of color in clustering */
      float color_importance_;
      /** \brief Importance of distance from seed center in clustering */
      float spatial_importance_;
      /** \brief Importance of similarity in normals for clustering */
      float normal_importance_;

      /** \brief Contains the FPFH values for Voxel cloud */
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr voxel_fpfh_;
      
      /** \brief Indices of the superpixel seed points before they are adjusted */
      std::vector<int> seed_indices_orig_;
      
      /** \brief Indices of the superpixel seed points after adjustment */
      std::vector<int> seed_indices_unshifted_;

      /** \brief Stores the colors used for the superpixel labels*/
      std::vector<uint32_t> label_colors_;
      
      /** \brief Stores a remapping of labels once they are cleaned */
      std::vector<int> label_remapping_;
      
      std::map<uint32_t, PointSuperVoxel> label_centers_;

      VoxelAdjacencyList supervoxel_adjacency_graph_;
      
      //TODO DEBUG REMOVE
      StopWatch timer_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/supervoxels.hpp>
#endif

#endif
