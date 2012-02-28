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

#ifndef OCTREE_COMPRESSION_HPP
#define OCTREE_COMPRESSION_HPP

#include "pcl/octree/octree_pointcloud.h"
#include "pcl/compression/entropy_range_coder.h"

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>


namespace pcl
{
  namespace octree
  {
    using namespace std;

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::encodePointCloud (const PointCloudConstPtr &cloud_arg,
                                                                       std::ostream& compressedTreeDataOut_arg)
      {
        unsigned char recentTreeDepth;
        recentTreeDepth = this->getTreeDepth ();

        // initialize octree
        this->switchBuffers ();
        this->setInputCloud (cloud_arg);

        // color field analysis
        cloudWithColor_ = false;
        std::vector<sensor_msgs::PointField> fields;
        int rgba_index = -1;
        rgba_index = pcl::getFieldIndex (*this->input_, "rgb", fields);
        if (rgba_index == -1)
        {
          rgba_index = pcl::getFieldIndex (*this->input_, "rgba", fields);
        }
        if (rgba_index >= 0)
        {
          pointColorOffset_ = fields[rgba_index].offset;
          cloudWithColor_ = true;
        }

        // apply encoding configuration
        cloudWithColor_ &= doColorEncoding_;

        // add point to octree
        this->addPointsFromInputCloud ();

        // if octree depth changed, we enforce I-frame encoding
        iFrame_ = (recentTreeDepth != this->getTreeDepth ());// | !(iFrameCounter%10);

        // enable I-frame rate
        if (iFrameCounter_++==iFrameRate_) {
          iFrameCounter_ =0;
          iFrame_ = true;
        }

        // increase frameID
        frameID_++;

        // do octree encoding
        if (!doVoxelGridEnDecoding_)
        {
          pointCountDataVector_.clear ();
          pointCountDataVector_.reserve (cloud_arg->points.size ());
        }

        // initialize color encoding
        colorCoder_.initializeEncoding ();
        colorCoder_.setPointCount ((unsigned int) cloud_arg->points.size ());
        colorCoder_.setVoxelCount (this->leafCount_);

        // initialize point encoding
        pointCoder_.initializeEncoding ();
        pointCoder_.setPointCount ((unsigned int) cloud_arg->points.size ());

        // serialize octree
        if (iFrame_) {
          // i-frame encoding - encode tree structure without referencing previous buffer
          this->serializeTree (binaryTreeDataVector_, false);
        } else {
          // p-frame encoding - XOR encoded tree structure
          this->serializeTree (binaryTreeDataVector_, true);
        }

        // write frame header information to stream
        this->writeFrameHeader (compressedTreeDataOut_arg);

        // apply entropy coding to the content of all data vectors and send data to output stream
        this->entropyEncoding (compressedTreeDataOut_arg);

        if (bShowStatistics)
        {
          float bytesPerXYZ;
          float bytesPerColor;

          bytesPerXYZ = (float)compressedPointDataLen_ / (float)pointCount_;
          bytesPerColor = (float)compressedColorDataLen_ / (float)pointCount_;


          std::cerr << "*** POINTCLOUD ENCODING ***" << std::endl;
          std::cerr << "Frame ID: " << frameID_ << std::endl;
          if (iFrame_)
          {
            std::cerr << "Encoding Frame: Intra frame" << std::endl;
          }
          else
          {
            std::cerr << "Encoding Frame: Prediction frame" << std::endl;
          }
          std::cerr << "Number of encoded points: " << pointCount_ << std::endl;
          std::cerr << "XYZ compression percentage: " << bytesPerXYZ / (3.0f * sizeof(float)) * 100.0f
              << "%" << std::endl;
          std::cerr << "XYZ bytes per point: " << bytesPerXYZ << " bytes" << std::endl;
          std::cerr << "Color compression percentage: " << bytesPerColor / (sizeof(int)) * 100.0f << "%" << std::endl;
          std::cerr << "Color bytes per point: " << bytesPerColor << " bytes" << std::endl;
          std::cerr << "Size of uncompressed point cloud: " <<
              pointCount_* (sizeof(int) + 3.0f  * sizeof(float))  / (1024) << " kBytes" << std::endl;
          std::cerr << "Size of compressed point cloud: " <<
              (compressedPointDataLen_ + compressedColorDataLen_) / (1024) << " kBytes" << std::endl;
          std::cerr << "Total bytes per point: " << bytesPerXYZ + bytesPerColor << " bytes" << std::endl;
          std::cerr << "Total compression percentage: " << (bytesPerXYZ + bytesPerColor) / (sizeof(int) + 3.0f
              * sizeof(float)) * 100.0f << "%" << std::endl;
          std::cerr << "Compression ratio: " << (float)(sizeof(int) + 3.0f  * sizeof(float))
              / (float)(bytesPerXYZ + bytesPerColor)  << std::endl << std::endl;
        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::decodePointCloud (std::istream& compressedTreeDataIn_arg,
                                                             PointCloudPtr &cloud_arg)
      {

        // initialize octree
        this->switchBuffers ();
        this->setOutputCloud (cloud_arg);

        // color field analysis
        cloudWithColor_ = false;
        std::vector<sensor_msgs::PointField> fields;
        int rgba_index = -1;
        rgba_index = pcl::getFieldIndex (*output_, "rgb", fields);
        if (rgba_index == -1)
        {
          rgba_index = pcl::getFieldIndex (*output_, "rgba", fields);
        }
        if (rgba_index >= 0)
        {
          pointColorOffset_ = fields[rgba_index].offset;
          cloudWithColor_ = true;
        }


        // read header from input stream
        this->readFrameHeader (compressedTreeDataIn_arg);

        // decode data vectors from stream
        this->entropyDecoding (compressedTreeDataIn_arg);

        // initialize color and point encoding
        colorCoder_.initializeDecoding ();
        pointCoder_.initializeDecoding ();

        // initialize output cloud
        output_->points.clear ();
        output_->points.reserve (pointCount_);

        if (iFrame_)
          // i-frame decoding - decode tree structure without referencing previous buffer
          this->deserializeTree (binaryTreeDataVector_, false);
        else
          // p-frame decoding - decode XOR encoded tree structure
          this->deserializeTree (binaryTreeDataVector_, true);

        // assign point cloud properties
        output_->height = 1;
        output_->width = (uint32_t) cloud_arg->points.size ();
        output_->is_dense = false;

        if (bShowStatistics)
        {
          float bytesPerXYZ;
          float bytesPerColor;

          bytesPerXYZ = (float)compressedPointDataLen_ / (float)pointCount_;
          bytesPerColor = (float)compressedColorDataLen_ / (float)pointCount_;

          std::cerr << "*** POINTCLOUD DECODING ***" << std::endl;
          std::cerr << "Frame ID: " << frameID_ << std::endl;
          if (iFrame_)
            std::cerr << "Encoding Frame: Intra frame" << std::endl;
          else
            std::cerr << "Encoding Frame: Prediction frame" << std::endl;
          std::cerr << "Number of encoded points: " << pointCount_ << std::endl;
          std::cerr << "XYZ compression percentage: " << bytesPerXYZ / (3.0f * sizeof(float)) * 100.0f
                    << "%" << std::endl;
          std::cerr << "XYZ bytes per point: " << bytesPerXYZ << " bytes" << std::endl;
          std::cerr << "Color compression percentage: " << bytesPerColor / (sizeof(int)) * 100.0f 
                    << "%" << std::endl;
          std::cerr << "Color bytes per point: " << bytesPerColor << " bytes" << std::endl;
          std::cerr << "Size of uncompressed point cloud: " 
                    << pointCount_* (sizeof(int) + 3.0f  * sizeof(float))  / (1024) << " kBytes" << std::endl;
          std::cerr << "Size of compressed point cloud: " 
                    << (compressedPointDataLen_ + compressedColorDataLen_) / (1024) << " kBytes" << std::endl;
          std::cerr << "Total bytes per point: " << bytesPerXYZ + bytesPerColor << " bytes" << std::endl;
          std::cerr << "Total compression percentage: " 
                    << (bytesPerXYZ + bytesPerColor) / (sizeof(int) + 3.0f * sizeof(float)) * 100.0f 
                    << "%" << std::endl;
          std::cerr << "Compression ratio: " 
                    << (float)(sizeof(int) + 3.0f  * sizeof(float)) / (float)(bytesPerXYZ + bytesPerColor) 
                    << std::endl << std::endl;
        }
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::entropyEncoding (std::ostream& compressedTreeDataOut_arg)
      {
        uint64_t binaryTreeDataVector_size;
        uint64_t pointAvgColorDataVector_size;

        compressedPointDataLen_ = 0;
        compressedColorDataLen_ = 0;

        // encode binary octree structure
        binaryTreeDataVector_size = binaryTreeDataVector_.size ();
        compressedTreeDataOut_arg.write ((const char*)&binaryTreeDataVector_size, sizeof(binaryTreeDataVector_size));
        compressedPointDataLen_ += entropyCoder_.encodeCharVectorToStream (binaryTreeDataVector_,
                                                                           compressedTreeDataOut_arg);

        if (cloudWithColor_)
        {
          // encode averaged voxel color information
          std::vector<char>& pointAvgColorDataVector = colorCoder_.getAverageDataVector ();
          pointAvgColorDataVector_size = pointAvgColorDataVector.size ();
          compressedTreeDataOut_arg.write ((const char*)&pointAvgColorDataVector_size,
                                           sizeof(pointAvgColorDataVector_size));
          compressedColorDataLen_ += entropyCoder_.encodeCharVectorToStream (pointAvgColorDataVector,
                                                                             compressedTreeDataOut_arg);
        }


        if (!doVoxelGridEnDecoding_)
        {
          uint64_t pointCountDataVector_size;
          uint64_t pointDiffDataVector_size;
          uint64_t pointDiffColorDataVector_size;

          // encode amount of points per voxel
          pointCountDataVector_size = pointCountDataVector_.size ();
          compressedTreeDataOut_arg.write ((const char*)&pointCountDataVector_size, sizeof(pointCountDataVector_size));
          compressedPointDataLen_ += entropyCoder_.encodeIntVectorToStream (pointCountDataVector_,
                                                                            compressedTreeDataOut_arg);

          // encode differential point information
          std::vector<char>& pointDiffDataVector = pointCoder_.getDifferentialDataVector ();
          pointDiffDataVector_size = pointDiffDataVector.size ();
          compressedTreeDataOut_arg.write ((const char*)&pointDiffDataVector_size, sizeof(pointDiffDataVector_size));
          compressedPointDataLen_ += entropyCoder_.encodeCharVectorToStream (pointDiffDataVector,
                                                                             compressedTreeDataOut_arg);
          if (cloudWithColor_)
          {
            // encode differential color information
            std::vector<char>& pointDiffColorDataVector = colorCoder_.getDifferentialDataVector ();
            pointDiffColorDataVector_size = pointDiffColorDataVector.size ();
            compressedTreeDataOut_arg.write ((const char*)&pointDiffColorDataVector_size,
                                             sizeof(pointDiffColorDataVector_size));
            compressedColorDataLen_ += entropyCoder_.encodeCharVectorToStream (pointDiffColorDataVector,
                                                                               compressedTreeDataOut_arg);
          }

        }

        // flush output stream
        compressedTreeDataOut_arg.flush ();
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::entropyDecoding (std::istream& compressedTreeDataIn_arg)
      {
        uint64_t binaryTreeDataVector_size;
        uint64_t pointAvgColorDataVector_size;

        compressedPointDataLen_ = 0;
        compressedColorDataLen_ = 0;

        // decode binary octree structure
        compressedTreeDataIn_arg.read ((char*)&binaryTreeDataVector_size, sizeof(binaryTreeDataVector_size));
        binaryTreeDataVector_.resize (binaryTreeDataVector_size);
        compressedPointDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                           binaryTreeDataVector_);

        if (dataWithColor_)
        {
          // decode averaged voxel color information
          std::vector<char>& pointAvgColorDataVector = colorCoder_.getAverageDataVector ();
          compressedTreeDataIn_arg.read ((char*)&pointAvgColorDataVector_size, sizeof(pointAvgColorDataVector_size));
          pointAvgColorDataVector.resize (pointAvgColorDataVector_size);
          compressedColorDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                             pointAvgColorDataVector);
        }

        if (!doVoxelGridEnDecoding_)
        {
          uint64_t pointCountDataVector_size;
          uint64_t pointDiffDataVector_size;
          uint64_t pointDiffColorDataVector_size;

          // decode amount of points per voxel
          compressedTreeDataIn_arg.read ((char*)&pointCountDataVector_size, sizeof(pointCountDataVector_size));
          pointCountDataVector_.resize (pointCountDataVector_size);
          compressedPointDataLen_ += entropyCoder_.decodeStreamToIntVector (compressedTreeDataIn_arg, pointCountDataVector_);
          pointCountDataVectorIterator_ = pointCountDataVector_.begin ();

          // decode differential point information
          std::vector<char>& pointDiffDataVector = pointCoder_.getDifferentialDataVector ();
          compressedTreeDataIn_arg.read ((char*)&pointDiffDataVector_size, sizeof(pointDiffDataVector_size));
          pointDiffDataVector.resize (pointDiffDataVector_size);
          compressedPointDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                             pointDiffDataVector);

          if (dataWithColor_)
          {
            // decode differential color information
            std::vector<char>& pointDiffColorDataVector = colorCoder_.getDifferentialDataVector ();
            compressedTreeDataIn_arg.read ((char*)&pointDiffColorDataVector_size, sizeof(pointDiffColorDataVector_size));
            pointDiffColorDataVector.resize (pointDiffColorDataVector_size);
            compressedColorDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                               pointDiffColorDataVector);
          }

        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::writeFrameHeader (std::ostream& compressedTreeDataOut_arg)
      {

        // encode header identifier
        compressedTreeDataOut_arg.write ((const char*)frameHeaderIdentifier_, strlen(frameHeaderIdentifier_));

        // encode point cloud header id
        compressedTreeDataOut_arg.write ((const char*)&frameID_, sizeof(frameID_));

        // encode frame type (I/P-frame)
        compressedTreeDataOut_arg.write ((const char*)&iFrame_, sizeof(iFrame_));
        if (iFrame_)
        {
          double minX, minY, minZ, maxX, maxY, maxZ;
          double octreeResolution;
          unsigned char colorBitDepth;
          double pointResolution;

          // get current configuration
          octreeResolution = this->getResolution();
          colorBitDepth  = colorCoder_.getBitDepth();
          pointResolution= pointCoder_.getPrecision();
          this->getBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);

          // encode amount of points
          if (doVoxelGridEnDecoding_) {
            pointCount_ = this->leafCount_;
          } else
          {
            pointCount_ = this->objectCount_;
          }

          // encode coding configuration
          compressedTreeDataOut_arg.write ((const char*)&doVoxelGridEnDecoding_, sizeof(doVoxelGridEnDecoding_));
          compressedTreeDataOut_arg.write ((const char*)&cloudWithColor_, sizeof(cloudWithColor_));
          compressedTreeDataOut_arg.write ((const char*)&pointCount_, sizeof(pointCount_));
          compressedTreeDataOut_arg.write ((const char*)&octreeResolution, sizeof(octreeResolution));
          compressedTreeDataOut_arg.write ((const char*)&colorBitDepth, sizeof(colorBitDepth));
          compressedTreeDataOut_arg.write ((const char*)&pointResolution, sizeof(pointResolution));

          // encode octree bounding box
          compressedTreeDataOut_arg.write ((const char*)&minX, sizeof(minX));
          compressedTreeDataOut_arg.write ((const char*)&minY, sizeof(minY));
          compressedTreeDataOut_arg.write ((const char*)&minZ, sizeof(minZ));
          compressedTreeDataOut_arg.write ((const char*)&maxX, sizeof(maxX));
          compressedTreeDataOut_arg.write ((const char*)&maxY, sizeof(maxY));
          compressedTreeDataOut_arg.write ((const char*)&maxZ, sizeof(maxZ));

        }
      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::readFrameHeader ( std::istream& compressedTreeDataIn_arg)
      {

      // sync to frame header
        unsigned int headerIdPos = 0;
        while (headerIdPos < strlen (frameHeaderIdentifier_))
        {
          char readChar;
          compressedTreeDataIn_arg.read ((char*)&readChar, sizeof(readChar));
          if (readChar != frameHeaderIdentifier_[headerIdPos++])
          {
            headerIdPos = (frameHeaderIdentifier_[0]==readChar)?1:0;
          }
        }

        // read header
        compressedTreeDataIn_arg.read ((char*)&frameID_, sizeof(frameID_));

        compressedTreeDataIn_arg.read ((char*)&iFrame_, sizeof(iFrame_));
        if (iFrame_)
        {
          double minX, minY, minZ, maxX, maxY, maxZ;
          double octreeResolution;
          unsigned char colorBitDepth;
          double pointResolution;

          // read coder configuration
          compressedTreeDataIn_arg.read ((char*)&doVoxelGridEnDecoding_, sizeof(doVoxelGridEnDecoding_));
          compressedTreeDataIn_arg.read ((char*)&dataWithColor_, sizeof(dataWithColor_));
          compressedTreeDataIn_arg.read ((char*)&pointCount_, sizeof(pointCount_));
          compressedTreeDataIn_arg.read ((char*)&octreeResolution, sizeof(octreeResolution));
          compressedTreeDataIn_arg.read ((char*)&colorBitDepth, sizeof(colorBitDepth));
          compressedTreeDataIn_arg.read ((char*)&pointResolution, sizeof(pointResolution));

          // read octree bounding box
          compressedTreeDataIn_arg.read ((char*)&minX, sizeof(minX));
          compressedTreeDataIn_arg.read ((char*)&minY, sizeof(minY));
          compressedTreeDataIn_arg.read ((char*)&minZ, sizeof(minZ));
          compressedTreeDataIn_arg.read ((char*)&maxX, sizeof(maxX));
          compressedTreeDataIn_arg.read ((char*)&maxY, sizeof(maxY));
          compressedTreeDataIn_arg.read ((char*)&maxZ, sizeof(maxZ));

          // reset octree and assign new bounding box & resolution
          this->deleteTree ();
          this->setResolution (octreeResolution);
          this->defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);

          // configure color & point coding
          colorCoder_.setBitDepth (colorBitDepth);
          pointCoder_.setPrecision ((float) pointResolution);

        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT>
      void
      PointCloudCompression<PointT, LeafT, OctreeT>::serializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg)
      {

      // reference to point indices vector stored within octree leaf
      const std::vector<int>& leafIdx = leaf_arg.getIdxVector ();

        if (!doVoxelGridEnDecoding_)
        {
          double lowerVoxelCorner[3];

          // encode amount of points within voxel
          pointCountDataVector_.push_back ((int)leafIdx.size ());

          // calculate lower voxel corner based on octree key
          lowerVoxelCorner[0] = ((double)key_arg.x) * this->resolution_ + this->minX_;
          lowerVoxelCorner[1] = ((double)key_arg.y) * this->resolution_ + this->minY_;
          lowerVoxelCorner[2] = ((double)key_arg.z) * this->resolution_ + this->minZ_;

          // differentially encode points to lower voxel corner
          pointCoder_.encodePoints (leafIdx, lowerVoxelCorner, this->input_);

          if (cloudWithColor_)
          {
            // encode color of points
            colorCoder_.encodePoints (leafIdx, pointColorOffset_, this->input_);
          }

        }
        else
        {
          if (cloudWithColor_)
          {
            // encode average color of all points within voxel
            colorCoder_.encodeAverageOfPoints (leafIdx, pointColorOffset_, this->input_);
          }
        }

      }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, OctreeT>::deserializeLeafCallback (OctreeLeaf& leaf_arg, const OctreeKey& key_arg)
    {
      // Silence compiler warnings
      (void)leaf_arg;

      double lowerVoxelCorner[3];
      std::size_t pointCount, i, cloudSize;
      PointT newPoint;

      pointCount = 1;

      if (!doVoxelGridEnDecoding_)
      {
        // get current cloud size
        cloudSize = this->output_->points.size ();

        // get amount of point to be decoded
        pointCount = *pointCountDataVectorIterator_;
        pointCountDataVectorIterator_++;

        // increase point cloud by amount of voxel points
        for (i = 0; i < pointCount; i++)
        {
          this->output_->points.push_back (newPoint);
        }

        // calculcate position of lower voxel corner
        lowerVoxelCorner[0] = ((double)key_arg.x) * this->resolution_ + this->minX_;
        lowerVoxelCorner[1] = ((double)key_arg.y) * this->resolution_ + this->minY_;
        lowerVoxelCorner[2] = ((double)key_arg.z) * this->resolution_ + this->minZ_;

        // decode differentially encoded points
        pointCoder_.decodePoints (this->output_, lowerVoxelCorner, cloudSize, cloudSize + pointCount);

      }
      else
      {
        // calculcate center of lower voxel corner
        newPoint.x = ((double)key_arg.x + 0.5) * this->resolution_ + this->minX_;
        newPoint.y = ((double)key_arg.y + 0.5) * this->resolution_ + this->minY_;
        newPoint.z = ((double)key_arg.z + 0.5) * this->resolution_ + this->minZ_;

        // add point to point cloud
        this->output_->points.push_back (newPoint);

      }

      if (cloudWithColor_)
      {
        if (dataWithColor_)
        {
          // decode color information
          colorCoder_.decodePoints (this->output_, this->output_->points.size () - pointCount,
                                    this->output_->points.size (), pointColorOffset_);
        }
        else
        {
          // set default color information
          colorCoder_.setDefaultColor (this->output_, this->output_->points.size () - pointCount,
                                       this->output_->points.size (), pointColorOffset_);
        }
      }

    }
  }
}

#define PCL_INSTANTIATE_PointCloudCompression(T) template class PCL_EXPORTS pcl::octree::PointCloudCompression<T>;

#endif

