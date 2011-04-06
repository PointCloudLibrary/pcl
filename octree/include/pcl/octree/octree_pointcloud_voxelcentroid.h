/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef OCTREE_VOXELCENTROID_H
#define OCTREE_VOXELCENTROID_H

#include "octree_pointcloud.h"

#include "octree_base.h"
#include "octree2buf_base.h"

namespace pcl
{
  namespace octree
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud voxel centroid class
     *  \note This class generate an octrees from a point cloud (zero-copy). It provides a vector of centroids for all occupied voxels. .
     *  \note The octree pointcloud is initialized with its voxel resolution. Its bounding box is automatically adjusted or can be predefined.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT = OctreeLeafDataTVector<int> , typename OctreeT = OctreeBase<int, LeafT> >
      class OctreePointCloudVoxelCentroids : public OctreePointCloud<PointT, LeafT, OctreeT>
      {

      public:
        // public typedefs for single/double buffering
        typedef OctreePointCloudVoxelCentroids<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloudVoxelCentroids<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;

        /** \brief OctreePointCloudVoxelCentroids class constructor.
         *  \param resolution_arg:  octree resolution at lowest octree level
         * */
        OctreePointCloudVoxelCentroids (const double resolution) :
          OctreePointCloud<PointT, LeafT, OctreeT> (resolution)
        {
        }

        /** \brief Empty class deconstructor. */
        virtual
        ~OctreePointCloudVoxelCentroids ()
        {
        }

        /** \brief Get PointT vector of centroids for all occupied voxels.
         * \param voxelCentroidList_arg: results are written to this vector of PointT elements
         * \return number of occupied voxels
         */
        unsigned int
        getVoxelCentroids (std::vector<PointT> &voxelCentroidList_arg)
        {

          size_t i;
          unsigned int pointCounter;
          typename OctreePointCloud<PointT, LeafT, OctreeT>::OctreeKey keyC, keyP;
          PointT meanPoint;
          PointT idxPoint;

          std::vector<int> indicesVector;

          // serialize leafs - this returns a list of point indices. Points indices from the same voxel are locates next to each other within this vector.
          this->serializeLeafs (indicesVector);

          // initializing
          keyP.x = keyP.y = keyP.z = std::numeric_limits<unsigned int>::max ();
          meanPoint.x = meanPoint.y = meanPoint.z = 0.0;
          pointCounter = 0;

          // iterate over all point indices
          for (i = 0; i < indicesVector.size (); i++)
          {
            idxPoint = this->input_->points[i];

            // get octree key for point (key specifies octree voxel)
            this->genOctreeKeyforPoint (idxPoint, keyC);

            if (keyC == keyP)
            {
              // voxel did not change - add point
              meanPoint.x += idxPoint.x;
              meanPoint.y += idxPoint.y;
              meanPoint.z += idxPoint.z;

              pointCounter++;
            }
            else
            {
              // voxel key did change - calculate centroid and push it to result vector
              if (pointCounter > 0)
              {
                meanPoint.x /= (float)pointCounter;
                meanPoint.y /= (float)pointCounter;
                meanPoint.z /= (float)pointCounter;

                voxelCentroidList_arg.push_back (meanPoint);
              }

              // reset centroid to current input point
              meanPoint.x = idxPoint.x;
              meanPoint.y = idxPoint.y;
              meanPoint.z = idxPoint.z;
              pointCounter = 1;

              keyP = keyC;
            }
          }

          // push last centroid to result vector if necessary
          if (pointCounter > 0)
          {
            meanPoint.x /= (float)pointCounter;
            meanPoint.y /= (float)pointCounter;
            meanPoint.z /= (float)pointCounter;

            voxelCentroidList_arg.push_back (meanPoint);
          }

          // return size of centroid vector
          return voxelCentroidList_arg.size ();
        }

      };
  }

}

#define PCL_INSTANTIATE_OctreePointCloudVoxelCentroids(T) template class pcl::octree::OctreePointCloudVoxelCentroids<T>;

#endif

