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

#ifndef OCTREE_COMPRESSION_H
#define OCTREE_COMPRESSION_H

#include "pcl/common/common.h"
#include "pcl/io/io.h"
#include "pcl/octree/octree_pointcloud.h"
#include "entropy_range_coder.h"
#include "color_coding.h"
#include "point_coding.h"

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

namespace pcl
{
  namespace octree
  {
    using namespace std;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief @b Octree pointcloud compression class
     *  \note This class enables compression and decompression of point cloud data based on octree data structures.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT>

      class PointCloudCompression : public OctreePointCloud<PointT, OctreeLeafDataTVector<int> , Octree2BufBase<
          int, OctreeLeafDataTVector<int> > >

      {
      public:

        // public typedefs
        typedef OctreeLeafDataTVector<int> LeafT;
        typedef Octree2BufBase<int, LeafT> OctreeT;

        typedef typename OctreePointCloud<PointT, LeafT, OctreeT>::PointCloud PointCloud;
        typedef typename OctreePointCloud<PointT, LeafT, OctreeT>::PointCloudPtr PointCloudPtr;
        typedef typename OctreePointCloud<PointT, LeafT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

        typedef typename OctreePointCloud<PointT, LeafT, OctreeT>::OctreeKey OctreeKey;
        typedef typename OctreeT::OctreeLeaf OctreeLeaf;

        /** \brief Constructor
         *  \param octreeResolution_arg:  octree resolution at lowest octree level
         *  \param pointResolution_arg:  precision of point coordinates
         *  \param doVoxelGridDownDownSampling_arg:  voxel grid filtering
         *  \param iFrameRate_arg:  i-frame encoding rate
         *  \param doColorEncoding_arg:  enable/disable color coding
         *  \param colorBitResolution_arg:  color bit depth
         *  \param showStatistics_arg:  output compression statistics
         * */
        PointCloudCompression (const double pointResolution_arg = 0.001,
                                     const double octreeResolution_arg = 0.01,
                                     bool doVoxelGridDownDownSampling_arg = false,
                                     unsigned int iFrameRate_arg = 30,
                                     const unsigned char colorBitResolution_arg = 6,
                                     bool doColorEncoding_arg = false,
                                     bool showStatistics_arg = true) :

          OctreePointCloud<PointT, LeafT, OctreeT> (octreeResolution_arg),
              doVoxelGridEnDecoding_ (doVoxelGridDownDownSampling_arg), iFrameRate_ (iFrameRate_arg),
              iFrameCounter_ (0), frameID_ (0), pointCount_ (0), iFrame_ (true),
              doColorEncoding_ (doColorEncoding_arg), cloudWithColor_ (false), dataWithColor_ (false),
              pointColorOffset_ (0), bShowStatistics (showStatistics_arg)

        {
          output_ = PointCloudPtr ();

          // configure point & color coder
          pointCoder_.setPrecision (pointResolution_arg);
          colorCoder_.setBitDepth (colorBitResolution_arg);

        }

        /** \brief Empty deconstructor. */
        virtual
        ~PointCloudCompression ()
        {
        }

        /** \brief Provide a pointer to the output data set.
         *  \param cloud_arg: the boost shared pointer to a PointCloud message
         */
        inline void
        setOutputCloud (const PointCloudPtr &cloud_arg)
        {
          if (output_ != cloud_arg)
          {
            output_ = cloud_arg;
          }
        }

        /** \brief Get a pointer to the output point cloud dataset.
         *  \return pointer to pointcloud output class.
         * */
        inline PointCloudPtr
        getOutputCloud ()
        {
          return (output_);
        }

        /** \brief Encode point cloud to output stream
         *  \param cloud_arg:  point cloud to be compressed
         *  \param compressedTreeDataOut_arg:  binary output stream containing compressed data
         * */
        void
        encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressedTreeDataOut_arg);

        /** \brief Decode point cloud from input stream
         *  \param compressedTreeDataIn_arg: binary input stream containing compressed data
         *  \param cloud_arg: reference to decoded point cloud
         * */
        void
        decodePointCloud (std::istream& compressedTreeDataIn_arg, PointCloudPtr &cloud_arg);

      protected:

        /** \brief Write frame information to output stream
         *  \param compressedTreeDataOut_arg: binary output stream
         * */
        void
        writeFrameHeader (std::ostream& compressedTreeDataOut_arg);

        /** \brief Read frame information to output stream
         *  \param compressedTreeDataOut_arg: binary input stream
         * */
        void
        readFrameHeader (std::istream& compressedTreeDataIn_arg);

        /** \brief Apply entropy encoding to encoded information and output to binary stream
         *  \param compressedTreeDataOut_arg: binary output stream
         * */
        void
        entropyEncoding (std::ostream& compressedTreeDataOut_arg);

        /** \brief Entropy decoding of input binary stream and output to information vectors
         *  \param compressedTreeDataOut_arg: binary input stream
         * */
        void
        entropyDecoding (std::istream& compressedTreeDataIn_arg);

        /** \brief Encode leaf node information during serialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         **/
        virtual void
        serializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg);

        /** \brief Decode leaf nodes information during deserialization
         *  \param leaf_arg: reference to new leaf node
         *  \param key_arg: octree key of new leaf node
         **/
        virtual void
        deserializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg);

        /** \brief Pointer to output point cloud dataset. */
        PointCloudPtr output_;

        /** \brief Vector for storing binary tree structure */
        std::vector<char> binaryTreeDataVector_;

        /** \brief Interator on binary tree structure vector */
        std::vector<char> binaryColorTreeVector_;

        /** \brief Vector for storing points per voxel information  */
        std::vector<unsigned int> pointCountDataVector_;

        /** \brief Interator on points per voxel vector */
        std::vector<unsigned int>::const_iterator pointCountDataVectorIterator_;

        /** \brief Color coding instance */
        ColorCoding<PointT> colorCoder_;

        /** \brief Point coding instance */
        PointCoding<PointT> pointCoder_;

        /** \brief Static range coder instance */
        StaticRangeCoder entropyCoder_;

        bool doVoxelGridEnDecoding_;
        unsigned int iFrameRate_;
        unsigned int iFrameCounter_;
        unsigned int frameID_;
        unsigned long pointCount_;
        bool iFrame_;

        bool doColorEncoding_;
        bool cloudWithColor_;
        bool dataWithColor_;
        unsigned char pointColorOffset_;

        //bool activating statistics
        bool bShowStatistics;
        unsigned long compressedPointDataLen_;
        unsigned long compressedColorDataLen_;

        // frame header identifier
        static const char* frameHeaderIdentifier_;

      };

      // define frame header initialization
      template<typename PointT>
      const char* PointCloudCompression<PointT>::frameHeaderIdentifier_ = "<PCL-COMPRESSED>";
  }

}

#define PCL_INSTANTIATE_PointCloudCompression(T) template class pcl::octree::PointCloudCompression<T>;

#endif

