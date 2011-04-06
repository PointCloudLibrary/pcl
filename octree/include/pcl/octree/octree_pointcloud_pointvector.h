#ifndef OCTREE_POINT_VECTOR_H
#define OCTREE_POINT_VECTOR_H

#include "octree_pointcloud.h"

#include "octree_base.h"
#include "octree2buf_base.h"

#include "octree_nodes.h"

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud point vector class
     *  \note This pointcloud octree class generate an octrees from a point cloud (zero-copy). Every leaf node contains a list of point indices of the dataset given by \a setInputCloud.
     *  \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeLeafDataTVector<int> , typename OctreeT = OctreeBase<int, LeafT> >
      class OctreePointCloudPointVector : public OctreePointCloud<PointT, LeafT, OctreeT>
      {

      public:
        // public typedefs for single/double buffering
        typedef OctreePointCloudPointVector<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloudPointVector<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;

        /** \brief Constructor.
         *  \param resolution_arg: octree resolution at lowest octree level
         * */
        OctreePointCloudPointVector (const double resolution) :
          OctreePointCloud<PointT, LeafT, OctreeT> (resolution)
        {
        }

        /** \brief Empty class constructor. */
        virtual
        ~OctreePointCloudPointVector ()
        {
        }

      };
  }
}

#define PCL_INSTANTIATE_OctreePointCloudPointVector(T) template class pcl::octree::OctreePointCloudPointVector<T>;

#endif
