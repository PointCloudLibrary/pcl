 
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

#pragma once

#include <boost/version.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/boost.h>



//DEBUG TODO REMOVE
#include <pcl/common/time.h>


namespace pcl
{
  /** \brief Supervoxel container class - stores a cluster extracted using supervoxel clustering 
   */
  template <typename PointT>
  class Supervoxel
  {
    public:
      Supervoxel () :
        voxels_ (new pcl::PointCloud<PointT> ()),
        normals_ (new pcl::PointCloud<Normal> ())
        {  } 

      typedef boost::shared_ptr<Supervoxel<PointT> > Ptr;
      typedef boost::shared_ptr<const Supervoxel<PointT> > ConstPtr;

      /** \brief Gets the centroid of the supervoxel
       *  \param[out] centroid_arg centroid of the supervoxel
       */ 
      void
      getCentroidPoint (PointXYZRGBA &centroid_arg)
      {
        centroid_arg = centroid_;
      }

      /** \brief Gets the point normal for the supervoxel 
       * \param[out] normal_arg Point normal of the supervoxel
       * \note This isn't an average, it is a normal computed using all of the voxels in the supervoxel as support
       */ 
      void
      getCentroidPointNormal (PointNormal &normal_arg)
      {
        normal_arg.x = centroid_.x;
        normal_arg.y = centroid_.y;
        normal_arg.z = centroid_.z;
        normal_arg.normal_x = normal_.normal_x;
        normal_arg.normal_y = normal_.normal_y;
        normal_arg.normal_z = normal_.normal_z;
        normal_arg.curvature = normal_.curvature;
      }

      /** \brief The normal calculated for the voxels contained in the supervoxel */
      pcl::Normal normal_;
      /** \brief The centroid of the supervoxel - average voxel */
      pcl::PointXYZRGBA centroid_;
      /** \brief A Pointcloud of the voxels in the supervoxel */
      typename pcl::PointCloud<PointT>::Ptr voxels_;
      /** \brief A Pointcloud of the normals for the points in the supervoxel */
      typename pcl::PointCloud<Normal>::Ptr normals_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
  };
  
  /** \brief Implements a supervoxel algorithm based on voxel structure, normals, and rgb values
   *   \note Supervoxels are oversegmented volumetric patches (usually surfaces) 
   *   \note Usually, color isn't needed (and can be detrimental)- spatial structure is mainly used
    * - J. Papon, A. Abramov, M. Schoeler, F. Woergoetter
    *   Voxel Cloud Connectivity Segmentation - Supervoxels from PointClouds
    *   In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR) 2013
    *  \ingroup segmentation 
    *  \author Jeremie Papon (jpapon@gmail.com)
    */
  template <typename PointT>
  class PCL_EXPORTS SupervoxelClustering : public pcl::PCLBase<PointT>
  {
    //Forward declaration of friended helper class
    class SupervoxelHelper;
    friend class SupervoxelHelper;
    public:
      /** \brief VoxelData is a structure used for storing data within a pcl::octree::OctreePointCloudAdjacencyContainer
       *  \note It stores xyz, rgb, normal, distance, an index, and an owner.
       */
      class VoxelData
      {
        public:
          VoxelData ():
            xyz_ (0.0f, 0.0f, 0.0f),
            rgb_ (0.0f, 0.0f, 0.0f),
            normal_ (0.0f, 0.0f, 0.0f, 0.0f),
            curvature_ (0.0f),
            owner_ (nullptr)
            {}

          /** \brief Gets the data of in the form of a point
           *  \param[out] point_arg Will contain the point value of the voxeldata
           */  
          void
          getPoint (PointT &point_arg) const;

          /** \brief Gets the data of in the form of a normal
           *  \param[out] normal_arg Will contain the normal value of the voxeldata
           */            
          void
          getNormal (Normal &normal_arg) const;

          Eigen::Vector3f xyz_;
          Eigen::Vector3f rgb_;
          Eigen::Vector4f normal_;
          float curvature_;
          float distance_;
          int idx_;
          SupervoxelHelper* owner_;

        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

      typedef pcl::octree::OctreePointCloudAdjacencyContainer<PointT, VoxelData> LeafContainerT;
      typedef std::vector <LeafContainerT*> LeafVectorT;

      typedef pcl::PointCloud<PointT> PointCloudT;
      typedef pcl::PointCloud<Normal> NormalCloudT;
      typedef pcl::octree::OctreePointCloudAdjacency<PointT, LeafContainerT> OctreeAdjacencyT;
      typedef pcl::octree::OctreePointCloudSearch <PointT> OctreeSearchT;
      typedef pcl::search::KdTree<PointT> KdTreeT;
      typedef boost::shared_ptr<std::vector<int> > IndicesPtr;

      using PCLBase <PointT>::initCompute;
      using PCLBase <PointT>::deinitCompute;
      using PCLBase <PointT>::input_;

      typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, uint32_t, float> VoxelAdjacencyList;
      typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
      typedef VoxelAdjacencyList::edge_descriptor EdgeID;

    public:

      /** \brief Constructor that sets default values for member variables. 
       *  \param[in] voxel_resolution The resolution (in meters) of voxels used
       *  \param[in] seed_resolution The average size (in meters) of resulting supervoxels
       */
      SupervoxelClustering (float voxel_resolution, float seed_resolution);

      [[deprecated("constructor with flag for using the single camera transform is deprecated. Default behavior is now to use the transform for organized clouds, and not use it for unorganized. Use setUseSingleCameraTransform() to override the defaults.")]]
      SupervoxelClustering (float voxel_resolution, float seed_resolution, bool) : SupervoxelClustering (voxel_resolution, seed_resolution) { }

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding neighbors. In other words it frees memory.
        */
      
      ~SupervoxelClustering ();

      /** \brief Set the resolution of the octree voxels */
      void
      setVoxelResolution (float resolution);

      /** \brief Get the resolution of the octree voxels */
      float 
      getVoxelResolution () const;

      /** \brief Set the resolution of the octree seed voxels */
      void
      setSeedResolution (float seed_resolution);

      /** \brief Get the resolution of the octree seed voxels */
      float 
      getSeedResolution () const;

      /** \brief Set the importance of color for supervoxels */
      void
      setColorImportance (float val);

      /** \brief Set the importance of spatial distance for supervoxels */
      void
      setSpatialImportance (float val);

      /** \brief Set the importance of scalar normal product for supervoxels */
      void
      setNormalImportance (float val);

      /** \brief Set whether or not to use the single camera transform 
       *  \note By default it will be used for organized clouds, but not for unorganized - this parameter will override that behavior
       *  The single camera transform scales bin size so that it increases exponentially with depth (z dimension).
       *  This is done to account for the decreasing point density found with depth when using an RGB-D camera.
       *  Without the transform, beyond a certain depth adjacency of voxels breaks down unless the voxel size is set to a large value.
       *  Using the transform allows preserving detail up close, while allowing adjacency at distance.
       *  The specific transform used here is:
       *  x /= z; y /= z; z = ln(z);
       *  This transform is applied when calculating the octree bins in OctreePointCloudAdjacency
       */
      void
      setUseSingleCameraTransform (bool val);
      
      /** \brief This method launches the segmentation algorithm and returns the supervoxels that were
       * obtained during the segmentation.
       * \param[out] supervoxel_clusters A map of labels to pointers to supervoxel structures
       */
      virtual void
      extract (std::map<uint32_t,typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

      /** \brief This method sets the cloud to be supervoxelized
       * \param[in] cloud The cloud to be supervoxelize
       */
      void
      setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr& cloud) override;

      /** \brief This method sets the normals to be used for supervoxels (should be same size as input cloud)
      * \param[in] normal_cloud The input normals                         
      */
      virtual void
      setNormalCloud (typename NormalCloudT::ConstPtr normal_cloud);

      /** \brief This method refines the calculated supervoxels - may only be called after extract
       * \param[in] num_itr The number of iterations of refinement to be done (2 or 3 is usually sufficient)
       * \param[out] supervoxel_clusters The resulting refined supervoxels
       */
      virtual void
      refineSupervoxels (int num_itr, std::map<uint32_t,typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

      ////////////////////////////////////////////////////////////
      /** \brief Returns an RGB colorized cloud showing superpixels
        * Otherwise it returns an empty pointer.
        * Points that belong to the same supervoxel have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it's random). Points that are unlabeled will be black
        * \note This will expand the label_colors_ vector so that it can accommodate all labels
        */
      [[deprecated("use getLabeledCloud() instead. An example of how to display and save with colorized labels can be found in examples/segmentation/example_supervoxels.cpp")]]
      typename pcl::PointCloud<PointXYZRGBA>::Ptr
      getColoredCloud () const
      { 
        return pcl::PointCloud<PointXYZRGBA>::Ptr (new pcl::PointCloud<PointXYZRGBA>);
      }

      /** \brief Returns a deep copy of the voxel centroid cloud */
      typename pcl::PointCloud<PointT>::Ptr
      getVoxelCentroidCloud () const;

      /** \brief Returns labeled cloud
        * Points that belong to the same supervoxel have the same label.
        * Labels for segments start from 1, unlabled points have label 0
        */
      typename pcl::PointCloud<PointXYZL>::Ptr
      getLabeledCloud () const;

      /** \brief Returns an RGB colorized voxelized cloud showing superpixels
       * Otherwise it returns an empty pointer.
       * Points that belong to the same supervoxel have the same color.
       * But this function doesn't guarantee that different segments will have different
       * color(it's random). Points that are unlabeled will be black
       * \note This will expand the label_colors_ vector so that it can accommodate all labels
       */
      [[deprecated("use getLabeledVoxelCloud() instead. An example of how to display and save with colorized labels can be found in examples/segmentation/example_supervoxels.cpp")]]
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      getColoredVoxelCloud () const
      {
        return pcl::PointCloud<PointXYZRGBA>::Ptr (new pcl::PointCloud<PointXYZRGBA>);
      }

      /** \brief Returns labeled voxelized cloud
       * Points that belong to the same supervoxel have the same label.
       * Labels for segments start from 1, unlabled points have label 0
       */      
      pcl::PointCloud<pcl::PointXYZL>::Ptr
      getLabeledVoxelCloud () const;

      /** \brief Gets the adjacency list (Boost Graph library) which gives connections between supervoxels
       *  \param[out] adjacency_list_arg BGL graph where supervoxel labels are vertices, edges are touching relationships
       */
      void
      getSupervoxelAdjacencyList (VoxelAdjacencyList &adjacency_list_arg) const;

      /** \brief Get a multimap which gives supervoxel adjacency
       *  \param[out] label_adjacency Multi-Map which maps a supervoxel label to all adjacent supervoxel labels
       */
      void 
      getSupervoxelAdjacency (std::multimap<uint32_t, uint32_t> &label_adjacency) const;

      /** \brief Static helper function which returns a pointcloud of normals for the input supervoxels 
       *  \param[in] supervoxel_clusters Supervoxel cluster map coming from this class
       *  \returns Cloud of PointNormals of the supervoxels
       * 
       */
      static pcl::PointCloud<pcl::PointNormal>::Ptr
      makeSupervoxelNormalCloud (std::map<uint32_t,typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

      /** \brief Returns the current maximum (highest) label */
      int
      getMaxLabel () const;

    private:
      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();

      /** \brief This selects points to use as initial supervoxel centroids
       *  \param[out] seed_indices The selected leaf indices
       */
      void
      selectInitialSupervoxelSeeds (std::vector<int> &seed_indices);

      /** \brief This method creates the internal supervoxel helpers based on the provided seed points
       *  \param[in] seed_indices Indices of the leaves to use as seeds
       */
      void
      createSupervoxelHelpers (std::vector<int> &seed_indices);

      /** \brief This performs the superpixel evolution */
      void
      expandSupervoxels (int depth);

      /** \brief This sets the data of the voxels in the tree */
      void 
      computeVoxelData ();

      /** \brief Reseeds the supervoxels by finding the voxel closest to current centroid */
      void
      reseedSupervoxels ();

      /** \brief Constructs the map of supervoxel clusters from the internal supervoxel helpers */
      void
      makeSupervoxels (std::map<uint32_t,typename Supervoxel<PointT>::Ptr > &supervoxel_clusters);

      /** \brief Stores the resolution used in the octree */
      float resolution_;

      /** \brief Stores the resolution used to seed the superpixels */
      float seed_resolution_;

      /** \brief Distance function used for comparing voxelDatas */
      float
      voxelDataDistance (const VoxelData &v1, const VoxelData &v2) const;

      /** \brief Transform function used to normalize voxel density versus distance from camera */
      void
      transformFunction (PointT &p);

      /** \brief Contains a KDtree for the voxelized cloud */
      typename pcl::search::KdTree<PointT>::Ptr voxel_kdtree_;

      /** \brief Octree Adjacency structure with leaves at voxel resolution */
      typename OctreeAdjacencyT::Ptr adjacency_octree_;

      /** \brief Contains the Voxelized centroid Cloud */
      typename PointCloudT::Ptr voxel_centroid_cloud_;

      /** \brief Contains the Voxelized centroid Cloud */
      typename NormalCloudT::ConstPtr input_normals_;

      /** \brief Importance of color in clustering */
      float color_importance_;
      /** \brief Importance of distance from seed center in clustering */
      float spatial_importance_;
      /** \brief Importance of similarity in normals for clustering */
      float normal_importance_;
      
      /** \brief Whether or not to use the transform compressing depth in Z 
       *  This is only checked if it has been manually set by the user.
       *  The default behavior is to use the transform for organized, and not for unorganized.
       */
      bool use_single_camera_transform_;
      /** \brief Whether to use default transform behavior or not */
      bool use_default_transform_behaviour_;
      
      /** \brief Internal storage class for supervoxels 
       * \note Stores pointers to leaves of clustering internal octree, 
       * \note so should not be used outside of clustering class 
       */
      class SupervoxelHelper
      {
        public:
          /** \brief Comparator for LeafContainerT pointers - used for sorting set of leaves
           * \note Compares by index in the overall leaf_vector. Order isn't important, so long as it is fixed.
           */
          struct compareLeaves
          {
            bool operator() (LeafContainerT* const &left, LeafContainerT* const &right) const
            {
              const VoxelData& leaf_data_left = left->getData ();
              const VoxelData& leaf_data_right = right->getData ();
              return leaf_data_left.idx_ < leaf_data_right.idx_;
            }
          };
          typedef std::set<LeafContainerT*, typename SupervoxelHelper::compareLeaves> LeafSetT;
          typedef typename LeafSetT::iterator iterator;
          typedef typename LeafSetT::const_iterator const_iterator;

          SupervoxelHelper (uint32_t label, SupervoxelClustering* parent_arg):
            label_ (label),
            parent_ (parent_arg)
          { }

          void
          addLeaf (LeafContainerT* leaf_arg);

          void
          removeLeaf (LeafContainerT* leaf_arg);

          void
          removeAllLeaves ();

          void 
          expand ();

          void 
          refineNormals ();

          void 
          updateCentroid ();

          void 
          getVoxels (typename pcl::PointCloud<PointT>::Ptr &voxels) const;

          void 
          getNormals (typename pcl::PointCloud<Normal>::Ptr &normals) const;

          typedef float (SupervoxelClustering::*DistFuncPtr)(const VoxelData &v1, const VoxelData &v2);

          uint32_t
          getLabel () const 
          { return label_; }

          Eigen::Vector4f 
          getNormal () const 
          { return centroid_.normal_; }

          Eigen::Vector3f 
          getRGB () const 
          { return centroid_.rgb_; }

          Eigen::Vector3f
          getXYZ () const 
          { return centroid_.xyz_;}

          void
          getXYZ (float &x, float &y, float &z) const
          { x=centroid_.xyz_[0]; y=centroid_.xyz_[1]; z=centroid_.xyz_[2]; }

          void
          getRGB (uint32_t &rgba) const
          { 
            rgba = static_cast<uint32_t>(centroid_.rgb_[0]) << 16 | 
                   static_cast<uint32_t>(centroid_.rgb_[1]) << 8 | 
                   static_cast<uint32_t>(centroid_.rgb_[2]); 
          }

          void 
          getNormal (pcl::Normal &normal_arg) const 
          { 
            normal_arg.normal_x = centroid_.normal_[0];
            normal_arg.normal_y = centroid_.normal_[1];
            normal_arg.normal_z = centroid_.normal_[2];
            normal_arg.curvature = centroid_.curvature_;
          }

          void
          getNeighborLabels (std::set<uint32_t> &neighbor_labels) const;

          VoxelData
          getCentroid () const
          { return centroid_; }

          size_t
          size () const { return leaves_.size (); }
        private:
          //Stores leaves
          LeafSetT leaves_;
          uint32_t label_;
          VoxelData centroid_;
          SupervoxelClustering* parent_;
        public:
          //Type VoxelData may have fixed-size Eigen objects inside
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      };

      //Make boost::ptr_list can access the private class SupervoxelHelper
#if BOOST_VERSION >= 107000
      friend void boost::checked_delete<> (const typename pcl::SupervoxelClustering<PointT>::SupervoxelHelper *) BOOST_NOEXCEPT;
#else
      friend void boost::checked_delete<> (const typename pcl::SupervoxelClustering<PointT>::SupervoxelHelper *);
#endif

      typedef boost::ptr_list<SupervoxelHelper> HelperListT;
      HelperListT supervoxel_helpers_;

      //TODO DEBUG REMOVE
      StopWatch timer_;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/segmentation/impl/supervoxel_clustering.hpp>
#endif
