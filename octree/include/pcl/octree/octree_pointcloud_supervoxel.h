/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon
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
 *  Author : jpapon@gmail.com
 *  Email  : jpapon@gmail.com
 */

#ifndef PCL_OCTREE_POINTCLOUD_SUPER_VOXEL_H_
#define PCL_OCTREE_POINTCLOUD_SUPER_VOXEL_H_

#include "octree_pointcloud.h"
#include "octree_base.h"
#include "octree2buf_base.h"
#include "octree_iterator.h"


#include <set>

namespace pcl
{
  namespace octree
  {
    /** \brief Simple container to hold edge properties in adjacency list */
     struct EdgeProperties 
    {
      float weight;
    };
    
    /** \brief @b Octree pointcloud super voxel leaf node class
     * \note This class implements a leaf node that stores pointers to neighboring leaves
     * as well as a PointSuperVoxel  
     */
    template<typename PointT>
    class OctreePointCloudSuperVoxelContainer : public OctreeContainerBase<int>
    {
    public:
      
      typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, PointSuperVoxel, EdgeProperties> VoxelAdjacencyList;
      typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
      
      
      /** \brief Class initialization. */
      OctreePointCloudSuperVoxelContainer ()
      {
        this->reset();
        temp_label_ = 0;
      }
      
      /** \brief Empty class deconstructor. */
      virtual ~OctreePointCloudSuperVoxelContainer ()
      {
      }
      
      /** \brief deep copy function */
      virtual OctreePointCloudSuperVoxelContainer *
      deepCopy () const
      {
        return (new OctreePointCloudSuperVoxelContainer (*this));
      }
      
      /** \brief Add new point to voxel.
       * \param[in] new_point the new point to add  
       */
      void 
      addPoint (const PointT& new_point)
      {
        point_counter_ += 1.0f;
        //Running average
        centroid_point_.x += (new_point.x - centroid_point_.x) / point_counter_ ;
        centroid_point_.y += (new_point.y - centroid_point_.y) / point_counter_ ;
        centroid_point_.z += (new_point.z - centroid_point_.z) / point_counter_ ;
        
        float sumRGB = static_cast<float>(new_point.r) + new_point.g + new_point.b; 
        centroid_point_.R += ((new_point.r / sumRGB) - centroid_point_.R)/point_counter_;
        centroid_point_.G += ((new_point.g / sumRGB) - centroid_point_.G)/point_counter_;
        centroid_point_.B += ((new_point.b / sumRGB) - centroid_point_.B)/point_counter_;
        
        centroid_point_.label = 0.0f;
        centroid_point_.distance = std::numeric_limits<float>::max ();
      }
      
      /** \brief Returns a const reference to the centroid to avoid copying
       */
      const PointSuperVoxel&
      getCentroid () const
      {
          return centroid_point_;
      }
      
      /** \brief Get the centroid point
       */
      void
      getCentroid (PointSuperVoxel& centroid_arg) const
      {
        centroid_arg = centroid_point_;
      }
      
      /** \brief Set the centroid point
       */
      void
      setCentroid (const PointSuperVoxel& centroid_arg) 
      {
        centroid_point_ = centroid_arg;
      }
      
      /** \brief Set the label for this voxel */
      void
      setLabel (uint32_t label_arg)
      {
        centroid_point_.label = label_arg;
      }
      
      /** \brief Set the label for this voxel temporarily 
           \note This is done so we don't affect this iteration with this change
       */
      void
      setTempLabel (uint32_t label_arg)
      {
        temp_label_ = label_arg;
      }
      
      /** \brief Push temp label into main */
      void
      pushLabel ()
      {
        centroid_point_.label = temp_label_;
      }
      
      /** \brief Gets the label for this voxel */
      uint32_t 
      getLabel () const
      {
        return centroid_point_.label;
      }
      
      /** \brief Set the distance to label for this voxel */
      void
      setDistance (float distance_arg)
      {
        centroid_point_.distance = distance_arg;
      }
      
      /** \brief Returns true if the voxel exists */
      bool 
      exists () const
      {
        return frames_since_observed_ >= 0;
      }
      
      /** \brief Reset the voxel - used for when it is no longer observed */
      virtual void 
      reset ()
      {
        frames_since_observed_ = -1;
        point_counter_ = 0.0f;
      }
      
      /** \brief Add new neighbor to voxel.
       * \param[in] new_point the new point to add  
       */
      void 
      addNeighbor (OctreePointCloudSuperVoxelContainer *neighbor)
      {
        neighbors_.insert (neighbor);
        
      }
      
      /** \brief Returns a pair of iterators to begin and end of neighbors set*/
      std::pair< typename std::set<OctreePointCloudSuperVoxelContainer*>::iterator
                ,typename std::set<OctreePointCloudSuperVoxelContainer*>::iterator>
      getAdjacentVoxels() const
      {
        return std::make_pair (neighbors_.begin (), neighbors_.end ());
      }
      
    private:
      int frames_since_observed_;
      float point_counter_;
      uint32_t temp_label_;
      PointSuperVoxel centroid_point_;
      std::set<OctreePointCloudSuperVoxelContainer*> neighbors_;
    };
    
   
    
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud voxel class used for supervoxel calculation & tracking
     *  \note This pointcloud octree class generate an octrees from a point cloud (zero-copy). 
     *  \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
     *  \note This class maintains adjacency information for all of its voxels and adjusts it dynamically as points and voxels are added/removed
     *  \ingroup octree
     *  \author Jeremie Papon (jpapon@gmail.com)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template< typename PointT, 
              typename LeafContainerT = OctreePointCloudSuperVoxelContainer <PointT>,    
              typename BranchContainerT = OctreeContainerEmpty<int> >
    class OctreePointCloudSuperVoxel : public OctreePointCloud< PointT, LeafContainerT, BranchContainerT>

    {
      
    public:
      
      typedef OctreePointCloudSuperVoxel<PointT, LeafContainerT, BranchContainerT> OctreeSuperVoxelT;
      typedef boost::shared_ptr<OctreeSuperVoxelT> Ptr;
      typedef boost::shared_ptr<const OctreeSuperVoxelT> ConstPtr;
      
      typedef OctreePointCloud<PointT, LeafContainerT, BranchContainerT> OctreePointCloudT;
      typedef typename OctreePointCloudT::LeafNode LeafNode;
      typedef typename OctreePointCloudT::BranchNode BranchNode;
      
      typedef OctreeBase<int, LeafContainerT, BranchContainerT> OctreeBaseT;
      
      
      // iterators are friends
      friend class OctreeIteratorBase<int, OctreeSuperVoxelT> ;
      friend class OctreeDepthFirstIterator<int, OctreeSuperVoxelT> ;
      friend class OctreeBreadthFirstIterator<int, OctreeSuperVoxelT> ;
      friend class OctreeLeafNodeIterator<int, OctreeSuperVoxelT> ;
      
      // Octree default iterators
      typedef OctreeDepthFirstIterator<int, OctreeSuperVoxelT> Iterator;
      typedef const OctreeDepthFirstIterator<int, OctreeSuperVoxelT> ConstIterator;
      Iterator begin(unsigned int maxDepth_arg = 0) {return Iterator(this, maxDepth_arg);};
      const Iterator end() {return Iterator();};
      
      // Octree leaf node iterators
      typedef OctreeLeafNodeIterator<int, OctreeSuperVoxelT> LeafNodeIterator;
      typedef const OctreeLeafNodeIterator<int, OctreeSuperVoxelT> ConstLeafNodeIterator;
      LeafNodeIterator leaf_begin(unsigned int maxDepth_arg = 0) {return LeafNodeIterator(this, maxDepth_arg);};
      const LeafNodeIterator leaf_end() {return LeafNodeIterator();};
            
      typedef std::pair<uint32_t, PointSuperVoxel> LabelCenterT;
      typedef OctreePointCloudSuperVoxelContainer<PointT> VoxelContainerT;
      
      typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, PointSuperVoxel, EdgeProperties> VoxelAdjacencyList;
      typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
      typedef VoxelAdjacencyList::edge_descriptor EdgeID;

      /** \brief Constructor.
        *  \param resolution_arg  octree resolution at lowest octree level
        *  \param max_dist Maximum spatial distance from supervoxel center that will be considered for labeling
        *  \param color_weight Weight given to distance from centroid in the normalized RGB color space
        *  \param normal_weight Weight given to difference from centroid in normals (using 1 - scalar product)
        *  \param spatial_weight Weight given to distance from centroid in spatial space
        * */
      OctreePointCloudSuperVoxel (const double resolution_arg, float max_dist = 0.1, float color_weight = 0.2, float normal_weight = 1.0, float spatial_weight = 0.5);


      /** \brief Empty class destructor. */
      virtual ~OctreePointCloudSuperVoxel ()
      {
      }

      /** \brief Add DataT object to leaf node at octree key.
       * \param[in] key_arg octree key addressing a leaf node.
       * \param[in] data_arg DataT object to be added.
       */
      virtual void 
      addData (const OctreeKey& key_arg, const int& data_arg)
      {
        LeafNode* new_leaf = 0;
        this->createLeafRecursive (key_arg, this->depthMask_, data_arg, this->rootNode_, new_leaf);
        
        if (new_leaf)
        {
          const PointT& cloudPoint = this->getPointByIndex (data_arg);
          
          // add data to leaf
          LeafContainerT* container = new_leaf;
          container->addPoint (cloudPoint);
          this->objectCount_++;
        }
      }
      
      /** \brief Get a single voxel addressed by a PointT point.
       * \param[in] point_arg point addressing a voxel in octree
       * \param[out] voxel_centroid_arg centroid is written to this PointSuperVoxel reference
       * \return "true" if voxel is found; "false" otherwise
       */
      bool
      getVoxelCentroidAtPoint (const PointT& point_arg, PointSuperVoxel& voxel_centroid_arg) const;
      
      /** \brief Get PointSuperVoxel vector of centroids for all occupied voxels.
       * \param[out] voxel_centroid_list_arg results are written to this vector of PointSuperVoxel elements
       * \return number of occupied voxels
       */
      size_t
      getVoxelCentroids (std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > &voxel_centroids_arg) const;
      
      /** \brief Inserts normals into the centroids in leaves using vector
        * \note THIS IS ONLY VALID IF THE VECTOR HAS ORDER FROM getVoxelCentroids
        */
      void
      insertNormals (const std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > &voxel_centroids_arg);
      
      /** \brief Fills in the neighbors fields for all occupied voxels in the tree */
      void
      computeNeighbors ();
      
      /** \brief Fills in cloud with the voxel centroids in the octree */
      void 
      getCentroidCloud (pcl::PointCloud<PointSuperVoxel>::Ptr &cloud ); 
      
      /** \brief Sets the label for a particular point to a value and returns a const reference to it*/
      const PointSuperVoxel& 
      setPointLabel (const PointSuperVoxel &point_arg, uint32_t label_arg);
      
      /** \brief Expands the labels to neighbors one step*/
      void
      iterateVoxelLabels (const std::map<uint32_t,PointSuperVoxel> &supervoxel_centers);
      
      /** \brief Updates the label centers based on tree labels*/
      void
      updateCenters (std::map<uint32_t,PointSuperVoxel> &supervoxel_centers);
       
      /** \brief Traverses the octree and computes an adjacency_list for the supervoxel labels
       *   \param[in] supervoxel_centers The centroids of the supervoxels from iterateVoxelLabels and updateCenters 
       *   \param[in] supervoxel_adjacency_graph The adjacency_list which will have vertices containing centroids and edges with weights of connection between supervoxels 
       */
      void
      computeSuperVoxelAdjacencyGraph (const std::map<uint32_t,PointSuperVoxel> &supervoxel_centers, VoxelAdjacencyList &supervoxel_adjacency_graph);
    protected:

      /** \brief Recursively explore the octree and output a PointSuperVoxel vector of centroids for all occupied voxels.
       * \param[in] binaryTreeOut_arg binary output vector
       * \param[in] branch_arg current branch node
       * \param[out] voxel_centroid_list_arg results are written to this vector of PointSuperVoxel elements
       */
      void
      getVoxelCentroidsRecursive (const BranchNode* branch_arg, 
                                  OctreeKey& key_arg, 
                                  std::vector<PointSuperVoxel, Eigen::aligned_allocator<PointSuperVoxel> > &voxel_centroid_list_arg) const;
       
      /** \brief Checks neighbors of point to see if they should have their label switched to match it */
      void 
      checkNeighbors (const LeafContainerT *leaf_arg, const PointSuperVoxel &supervoxel_center );
      
      /** \brief Computes the distance between two points */
      float
      distance (const PointSuperVoxel &p1, const PointSuperVoxel &p2) const;
      
      
      float max_dist_, max_dist_sqr_;
      float color_weight_, normal_weight_, spatial_weight_;
      
    };
    
      
  }
}

//#ifdef PCL_NO_PRECOMPILE
#include <pcl/octree/impl/octree_pointcloud_supervoxel.hpp>
//#endif

#endif //PCL_OCTREE_POINTCLOUD_SUPER_VOXEL_H_

