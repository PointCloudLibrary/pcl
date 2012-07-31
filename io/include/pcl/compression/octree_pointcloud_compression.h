/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 */

#ifndef OCTREE_COMPRESSION_H
#define OCTREE_COMPRESSION_H

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_pointcloud.h>
#include "entropy_range_coder.h"
#include "color_coding.h"
#include "point_coding.h"

#include "compression_profiles.h"

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

using namespace pcl::octree;

namespace pcl
{
  namespace io
  {
    /** \brief @b Octree pointcloud compression class
     *  \note This class enables compression and decompression of point cloud data based on octree data structures.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename PointT, typename LeafT = OctreeContainerDataTVector<int>,
        typename BranchT = OctreeContainerEmpty<int>,
        typename OctreeT = Octree2BufBase<int, LeafT, BranchT> >
    class OctreePointCloudCompression : public OctreePointCloud<PointT, LeafT,
        BranchT, OctreeT>
    {
      public:
        // public typedefs
        typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
        typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
        typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        typedef OctreePointCloudCompression<PointT, LeafT, BranchT, Octree2BufBase<int, LeafT, BranchT> > RealTimeStreamCompression;
        typedef OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeBase<int, LeafT, BranchT> > SinglePointCloudCompressionLowMemory;


        /** \brief Constructor
          * \param compressionProfile_arg:  define compression profile
          * \param octreeResolution_arg:  octree resolution at lowest octree level
          * \param pointResolution_arg:  precision of point coordinates
          * \param doVoxelGridDownDownSampling_arg:  voxel grid filtering
          * \param iFrameRate_arg:  i-frame encoding rate
          * \param doColorEncoding_arg:  enable/disable color coding
          * \param colorBitResolution_arg:  color bit depth
          * \param showStatistics_arg:  output compression statistics
          */
        OctreePointCloudCompression (compression_Profiles_e compressionProfile_arg = MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
                               bool showStatistics_arg = false,
                               const double pointResolution_arg = 0.001,
                               const double octreeResolution_arg = 0.01,
                               bool doVoxelGridDownDownSampling_arg = false,
                               const unsigned int iFrameRate_arg = 30,
                               bool doColorEncoding_arg = true,
                               const unsigned char colorBitResolution_arg = 6) :
          OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (octreeResolution_arg),
          output_ (PointCloudPtr ()),
          binaryTreeDataVector_ (),
          binaryColorTreeVector_ (),
          pointCountDataVector_ (),
          pointCountDataVectorIterator_ (),
          colorCoder_ (),
          pointCoder_ (),
          entropyCoder_ (),
          doVoxelGridEnDecoding_ (doVoxelGridDownDownSampling_arg), iFrameRate_ (iFrameRate_arg),
          iFrameCounter_ (0), frameID_ (0), pointCount_ (0), iFrame_ (true),
          doColorEncoding_ (doColorEncoding_arg), cloudWithColor_ (false), dataWithColor_ (false),
          pointColorOffset_ (0), bShowStatistics (showStatistics_arg), 
          compressedPointDataLen_ (), compressedColorDataLen_ (), selectedProfile_(compressionProfile_arg),
          pointResolution_(pointResolution_arg), octreeResolution_(octreeResolution_arg), colorBitResolution_(colorBitResolution_arg)
        {
          initialization();
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreePointCloudCompression ()
        {
        }

        /** \brief Initialize globals */
        void initialization () {
          if (selectedProfile_ != MANUAL_CONFIGURATION)
          {
            // apply selected compression profile

            // retrieve profile settings
            const configurationProfile_t selectedProfile = compressionProfiles_[selectedProfile_];

            // apply profile settings
            iFrameRate_ = selectedProfile.iFrameRate;
            doVoxelGridEnDecoding_ = selectedProfile.doVoxelGridDownSampling;
            this->setResolution (selectedProfile.octreeResolution);
            pointCoder_.setPrecision (static_cast<float> (selectedProfile.pointResolution));
            doColorEncoding_ = selectedProfile.doColorEncoding;
            colorCoder_.setBitDepth (selectedProfile.colorBitResolution);

          }
          else 
          {
            // configure point & color coder
            pointCoder_.setPrecision (static_cast<float> (pointResolution_));
            colorCoder_.setBitDepth (colorBitResolution_);
          }

          if (pointCoder_.getPrecision () == this->getResolution ())
            //disable differential point colding
            doVoxelGridEnDecoding_ = true;

        }

        /** \brief Provide a pointer to the output data set.
          * \param cloud_arg: the boost shared pointer to a PointCloud message
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
          * \return pointer to pointcloud output class.
          */
        inline PointCloudPtr
        getOutputCloud () const
        {
          return (output_);
        }

        /** \brief Encode point cloud to output stream
          * \param cloud_arg:  point cloud to be compressed
          * \param compressedTreeDataOut_arg:  binary output stream containing compressed data
          */
        void
        encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressedTreeDataOut_arg);

        /** \brief Decode point cloud from input stream
          * \param compressedTreeDataIn_arg: binary input stream containing compressed data
          * \param cloud_arg: reference to decoded point cloud
          */
        void
        decodePointCloud (std::istream& compressedTreeDataIn_arg, PointCloudPtr &cloud_arg);

      protected:

        /** \brief Write frame information to output stream
          * \param compressedTreeDataOut_arg: binary output stream
          */
        void
        writeFrameHeader (std::ostream& compressedTreeDataOut_arg);

        /** \brief Read frame information to output stream
          * \param compressedTreeDataIn_arg: binary input stream
          */
        void
        readFrameHeader (std::istream& compressedTreeDataIn_arg);

        /** \brief Synchronize to frame header
          * \param compressedTreeDataIn_arg: binary input stream
          */
        void
        syncToHeader (std::istream& compressedTreeDataIn_arg);

        /** \brief Apply entropy encoding to encoded information and output to binary stream
          * \param compressedTreeDataOut_arg: binary output stream
          */
        void
        entropyEncoding (std::ostream& compressedTreeDataOut_arg);

        /** \brief Entropy decoding of input binary stream and output to information vectors
          * \param compressedTreeDataIn_arg: binary input stream
          */
        void
        entropyDecoding (std::istream& compressedTreeDataIn_arg);

        /** \brief Encode leaf node information during serialization
          * \param leaf_arg: reference to new leaf node
          * \param key_arg: octree key of new leaf node
         */
        virtual void
        serializeTreeCallback (LeafNode &leaf_arg, const OctreeKey& key_arg);

        /** \brief Decode leaf nodes information during deserialization
          * \param leaf_arg: reference to new leaf node
         * \param key_arg: octree key of new leaf node
         */
        virtual void
        deserializeTreeCallback (LeafNode&, const OctreeKey& key_arg);


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
        uint32_t iFrameRate_;
        uint32_t iFrameCounter_;
        uint32_t frameID_;
        uint64_t pointCount_;
        bool iFrame_;

        bool doColorEncoding_;
        bool cloudWithColor_;
        bool dataWithColor_;
        unsigned char pointColorOffset_;

        //bool activating statistics
        bool bShowStatistics;
        uint64_t compressedPointDataLen_;
        uint64_t compressedColorDataLen_;

        // frame header identifier
        static const char* frameHeaderIdentifier_;

        const compression_Profiles_e selectedProfile_;
        const double pointResolution_;
        const double octreeResolution_;
        const unsigned char colorBitResolution_;

      };

    // define frame identifier
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
      const char* OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::frameHeaderIdentifier_ = "<PCL-OCT-COMPRESSED>";
  }

}


#endif

