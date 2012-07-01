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

#ifndef OCTREE_COMPRESSION_HPP
#define OCTREE_COMPRESSION_HPP

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/compression/entropy_range_coder.h>

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
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void PointCloudCompression<
        PointT, LeafT, BranchT, OctreeT>::encodePointCloud (
        const PointCloudConstPtr &cloud_arg,
        std::ostream& compressedTreeDataOut_arg)
    {
      unsigned char recentTreeDepth =
          static_cast<unsigned char> (this->getTreeDepth ());

      // initialize octree
      this->setInputCloud (cloud_arg);

      // add point to octree
      this->addPointsFromInputCloud ();

      // make sure cloud contains points
      if (this->leafCount_>0) {


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
          pointColorOffset_ = static_cast<unsigned char> (fields[rgba_index].offset);
          cloudWithColor_ = true;
        }

        // apply encoding configuration
        cloudWithColor_ &= doColorEncoding_;


        // if octree depth changed, we enforce I-frame encoding
        iFrame_ |= (recentTreeDepth != this->getTreeDepth ());// | !(iFrameCounter%10);

        // enable I-frame rate
        if (iFrameCounter_++==iFrameRate_)
        {
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
        colorCoder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
        colorCoder_.setVoxelCount (static_cast<unsigned int> (this->leafCount_));

        // initialize point encoding
        pointCoder_.initializeEncoding ();
        pointCoder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));

        // serialize octree
        if (iFrame_)
          // i-frame encoding - encode tree structure without referencing previous buffer
          this->serializeTree (binaryTreeDataVector_, false);
        else
          // p-frame encoding - XOR encoded tree structure
          this->serializeTree (binaryTreeDataVector_, true);

        // write frame header information to stream
        this->writeFrameHeader (compressedTreeDataOut_arg);

        // apply entropy coding to the content of all data vectors and send data to output stream
        this->entropyEncoding (compressedTreeDataOut_arg);

        // prepare for next frame
        this->switchBuffers ();
        iFrame_ = false;

        if (bShowStatistics)
        {
          float bytesPerXYZ = static_cast<float> (compressedPointDataLen_) / static_cast<float> (pointCount_);
          float bytesPerColor = static_cast<float> (compressedColorDataLen_) / static_cast<float> (pointCount_);

          PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
          PCL_INFO ("Frame ID: %d\n", frameID_);
          if (iFrame_)
            PCL_INFO ("Encoding Frame: Intra frame\n");
          else
            PCL_INFO ("Encoding Frame: Prediction frame\n");
          PCL_INFO ("Number of encoded points: %ld\n", pointCount_);
          PCL_INFO ("XYZ compression percentage: %f%%\n", bytesPerXYZ / (3.0f * sizeof(float)) * 100.0f);
          PCL_INFO ("XYZ bytes per point: %f bytes\n", bytesPerXYZ);
          PCL_INFO ("Color compression percentage: %f%%\n", bytesPerColor / (sizeof (int)) * 100.0f);
          PCL_INFO ("Color bytes per point: %f bytes\n", bytesPerColor);
          PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (pointCount_) * (sizeof (int) + 3.0f  * sizeof (float)) / 1024);
          PCL_INFO ("Size of compressed point cloud: %d kBytes\n", (compressedPointDataLen_ + compressedColorDataLen_) / (1024));
          PCL_INFO ("Total bytes per point: %f\n", bytesPerXYZ + bytesPerColor);
          PCL_INFO ("Total compression percentage: %f\n", (bytesPerXYZ + bytesPerColor) / (sizeof (int) + 3.0f * sizeof(float)) * 100.0f);
          PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytesPerXYZ + bytesPerColor));
        }
      } else {
        if (bShowStatistics)
        PCL_INFO ("Info: Dropping empty point cloud\n");
        this->deleteTree();
        iFrameCounter_ = 0;
        iFrame_ = true;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::decodePointCloud (
        std::istream& compressedTreeDataIn_arg,
        PointCloudPtr &cloud_arg)
    {

      // synchronize to frame header
      syncToHeader(compressedTreeDataIn_arg);

      // initialize octree
      this->switchBuffers ();
      this->setOutputCloud (cloud_arg);

      // color field analysis
      cloudWithColor_ = false;
      std::vector<sensor_msgs::PointField> fields;
      int rgba_index = -1;
      rgba_index = pcl::getFieldIndex (*output_, "rgb", fields);
      if (rgba_index == -1)
        rgba_index = pcl::getFieldIndex (*output_, "rgba", fields);
      if (rgba_index >= 0)
      {
        pointColorOffset_ = static_cast<unsigned char> (fields[rgba_index].offset);
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
      output_->points.reserve (static_cast<std::size_t> (pointCount_));

      if (iFrame_)
        // i-frame decoding - decode tree structure without referencing previous buffer
        this->deserializeTree (binaryTreeDataVector_, false);
      else
        // p-frame decoding - decode XOR encoded tree structure
        this->deserializeTree (binaryTreeDataVector_, true);

      // assign point cloud properties
      output_->height = 1;
      output_->width = static_cast<uint32_t> (cloud_arg->points.size ());
      output_->is_dense = false;

      if (bShowStatistics)
      {
        float bytesPerXYZ = static_cast<float> (compressedPointDataLen_) / static_cast<float> (pointCount_);
        float bytesPerColor = static_cast<float> (compressedColorDataLen_) / static_cast<float> (pointCount_);

        PCL_INFO ("*** POINTCLOUD DECODING ***\n");
        PCL_INFO ("Frame ID: %d\n", frameID_);
        if (iFrame_)
          PCL_INFO ("Encoding Frame: Intra frame\n");
        else
          PCL_INFO ("Encoding Frame: Prediction frame\n");
        PCL_INFO ("Number of encoded points: %ld\n", pointCount_);
        PCL_INFO ("XYZ compression percentage: %f%%\n", bytesPerXYZ / (3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("XYZ bytes per point: %f bytes\n", bytesPerXYZ);
        PCL_INFO ("Color compression percentage: %f%%\n", bytesPerColor / (sizeof (int)) * 100.0f);
        PCL_INFO ("Color bytes per point: %f bytes\n", bytesPerColor);
        PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (pointCount_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
        PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressedPointDataLen_ + compressedColorDataLen_) / 1024.0f);
        PCL_INFO ("Total bytes per point: %d bytes\n", static_cast<int> (bytesPerXYZ + bytesPerColor));
        PCL_INFO ("Total compression percentage: %f%%\n", (bytesPerXYZ + bytesPerColor) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytesPerXYZ + bytesPerColor));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::entropyEncoding (std::ostream& compressedTreeDataOut_arg)
    {
      uint64_t binaryTreeDataVector_size;
      uint64_t pointAvgColorDataVector_size;

      compressedPointDataLen_ = 0;
      compressedColorDataLen_ = 0;

      // encode binary octree structure
      binaryTreeDataVector_size = binaryTreeDataVector_.size ();
      compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&binaryTreeDataVector_size), sizeof (binaryTreeDataVector_size));
      compressedPointDataLen_ += entropyCoder_.encodeCharVectorToStream (binaryTreeDataVector_,
                                                                         compressedTreeDataOut_arg);

      if (cloudWithColor_)
      {
        // encode averaged voxel color information
        std::vector<char>& pointAvgColorDataVector = colorCoder_.getAverageDataVector ();
        pointAvgColorDataVector_size = pointAvgColorDataVector.size ();
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&pointAvgColorDataVector_size),
                                         sizeof (pointAvgColorDataVector_size));
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
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&pointCountDataVector_size), sizeof (pointCountDataVector_size));
        compressedPointDataLen_ += entropyCoder_.encodeIntVectorToStream (pointCountDataVector_,
                                                                          compressedTreeDataOut_arg);

        // encode differential point information
        std::vector<char>& pointDiffDataVector = pointCoder_.getDifferentialDataVector ();
        pointDiffDataVector_size = pointDiffDataVector.size ();
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&pointDiffDataVector_size), sizeof (pointDiffDataVector_size));
        compressedPointDataLen_ += entropyCoder_.encodeCharVectorToStream (pointDiffDataVector,
                                                                           compressedTreeDataOut_arg);
        if (cloudWithColor_)
        {
          // encode differential color information
          std::vector<char>& pointDiffColorDataVector = colorCoder_.getDifferentialDataVector ();
          pointDiffColorDataVector_size = pointDiffColorDataVector.size ();
          compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&pointDiffColorDataVector_size),
                                           sizeof (pointDiffColorDataVector_size));
          compressedColorDataLen_ += entropyCoder_.encodeCharVectorToStream (pointDiffColorDataVector,
                                                                             compressedTreeDataOut_arg);
        }
      }
      // flush output stream
      compressedTreeDataOut_arg.flush ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::entropyDecoding (std::istream& compressedTreeDataIn_arg)
    {
      uint64_t binaryTreeDataVector_size;
      uint64_t pointAvgColorDataVector_size;

      compressedPointDataLen_ = 0;
      compressedColorDataLen_ = 0;

      // decode binary octree structure
      compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&binaryTreeDataVector_size), sizeof (binaryTreeDataVector_size));
      binaryTreeDataVector_.resize (static_cast<std::size_t> (binaryTreeDataVector_size));
      compressedPointDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                         binaryTreeDataVector_);

      if (dataWithColor_)
      {
        // decode averaged voxel color information
        std::vector<char>& pointAvgColorDataVector = colorCoder_.getAverageDataVector ();
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&pointAvgColorDataVector_size), sizeof (pointAvgColorDataVector_size));
        pointAvgColorDataVector.resize (static_cast<std::size_t> (pointAvgColorDataVector_size));
        compressedColorDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                           pointAvgColorDataVector);
      }

      if (!doVoxelGridEnDecoding_)
      {
        uint64_t pointCountDataVector_size;
        uint64_t pointDiffDataVector_size;
        uint64_t pointDiffColorDataVector_size;

        // decode amount of points per voxel
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&pointCountDataVector_size), sizeof (pointCountDataVector_size));
        pointCountDataVector_.resize (static_cast<std::size_t> (pointCountDataVector_size));
        compressedPointDataLen_ += entropyCoder_.decodeStreamToIntVector (compressedTreeDataIn_arg, pointCountDataVector_);
        pointCountDataVectorIterator_ = pointCountDataVector_.begin ();

        // decode differential point information
        std::vector<char>& pointDiffDataVector = pointCoder_.getDifferentialDataVector ();
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&pointDiffDataVector_size), sizeof (pointDiffDataVector_size));
        pointDiffDataVector.resize (static_cast<std::size_t> (pointDiffDataVector_size));
        compressedPointDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                           pointDiffDataVector);

        if (dataWithColor_)
        {
          // decode differential color information
          std::vector<char>& pointDiffColorDataVector = colorCoder_.getDifferentialDataVector ();
          compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&pointDiffColorDataVector_size), sizeof (pointDiffColorDataVector_size));
          pointDiffColorDataVector.resize (static_cast<std::size_t> (pointDiffColorDataVector_size));
          compressedColorDataLen_ += entropyCoder_.decodeStreamToCharVector (compressedTreeDataIn_arg,
                                                                             pointDiffColorDataVector);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader (std::ostream& compressedTreeDataOut_arg)
    {
      // encode header identifier
      compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (frameHeaderIdentifier_), strlen (frameHeaderIdentifier_));
      // encode point cloud header id
      compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&frameID_), sizeof (frameID_));
      // encode frame type (I/P-frame)
      compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&iFrame_), sizeof (iFrame_));
      if (iFrame_)
      {
        double minX, minY, minZ, maxX, maxY, maxZ;
        double octreeResolution;
        unsigned char colorBitDepth;
        double pointResolution;

        // get current configuration
        octreeResolution = this->getResolution ();
        colorBitDepth  = colorCoder_.getBitDepth ();
        pointResolution= pointCoder_.getPrecision ();
        this->getBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);

        // encode amount of points
        if (doVoxelGridEnDecoding_)
          pointCount_ = this->leafCount_;
        else
          pointCount_ = this->objectCount_;

        // encode coding configuration
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&doVoxelGridEnDecoding_), sizeof (doVoxelGridEnDecoding_));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&cloudWithColor_), sizeof (cloudWithColor_));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&pointCount_), sizeof (pointCount_));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&octreeResolution), sizeof (octreeResolution));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&colorBitDepth), sizeof (colorBitDepth));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&pointResolution), sizeof (pointResolution));

        // encode octree bounding box
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&minX), sizeof (minX));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&minY), sizeof (minY));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&minZ), sizeof (minZ));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&maxX), sizeof (maxX));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&maxY), sizeof (maxY));
        compressedTreeDataOut_arg.write (reinterpret_cast<const char*> (&maxZ), sizeof (maxZ));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::syncToHeader ( std::istream& compressedTreeDataIn_arg)
    {
      // sync to frame header
      unsigned int headerIdPos = 0;
      while (headerIdPos < strlen (frameHeaderIdentifier_))
      {
        char readChar;
        compressedTreeDataIn_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
        if (readChar != frameHeaderIdentifier_[headerIdPos++])
          headerIdPos = (frameHeaderIdentifier_[0]==readChar)?1:0;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::readFrameHeader ( std::istream& compressedTreeDataIn_arg)
    {
      // read header
      compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&frameID_), sizeof (frameID_));
      compressedTreeDataIn_arg.read (reinterpret_cast<char*>(&iFrame_), sizeof (iFrame_));
      if (iFrame_)
      {
        double minX, minY, minZ, maxX, maxY, maxZ;
        double octreeResolution;
        unsigned char colorBitDepth;
        double pointResolution;

        // read coder configuration
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&doVoxelGridEnDecoding_), sizeof (doVoxelGridEnDecoding_));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&dataWithColor_), sizeof (dataWithColor_));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&pointCount_), sizeof (pointCount_));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&octreeResolution), sizeof (octreeResolution));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&colorBitDepth), sizeof (colorBitDepth));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&pointResolution), sizeof (pointResolution));

        // read octree bounding box
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&minX), sizeof (minX));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&minY), sizeof (minY));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&minZ), sizeof (minZ));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&maxX), sizeof (maxX));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&maxY), sizeof (maxY));
        compressedTreeDataIn_arg.read (reinterpret_cast<char*> (&maxZ), sizeof (maxZ));

        // reset octree and assign new bounding box & resolution
        this->deleteTree ();
        this->setResolution (octreeResolution);
        this->defineBoundingBox (minX, minY, minZ, maxX, maxY, maxZ);

        // configure color & point coding
        colorCoder_.setBitDepth (colorBitDepth);
        pointCoder_.setPrecision (static_cast<float> (pointResolution));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::serializeTreeCallback (
        LeafNode &leaf_arg, const OctreeKey & key_arg)
    {
      // reference to point indices vector stored within octree leaf
      const std::vector<int>& leafIdx = leaf_arg.getDataTVector ();

      if (!doVoxelGridEnDecoding_)
      {
        double lowerVoxelCorner[3];

        // encode amount of points within voxel
        pointCountDataVector_.push_back (static_cast<int> (leafIdx.size ()));

        // calculate lower voxel corner based on octree key
        lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * this->resolution_ + this->minX_;
        lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * this->resolution_ + this->minY_;
        lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * this->resolution_ + this->minZ_;

        // differentially encode points to lower voxel corner
        pointCoder_.encodePoints (leafIdx, lowerVoxelCorner, this->input_);

        if (cloudWithColor_)
          // encode color of points
          colorCoder_.encodePoints (leafIdx, pointColorOffset_, this->input_);
      }
      else
      {
        if (cloudWithColor_)
          // encode average color of all points within voxel
          colorCoder_.encodeAverageOfPoints (leafIdx, pointColorOffset_, this->input_);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    PointCloudCompression<PointT, LeafT, BranchT, OctreeT>::deserializeTreeCallback (LeafNode&,
        const OctreeKey& key_arg)
    {
      double lowerVoxelCorner[3];
      std::size_t pointCount, i, cloudSize;
      PointT newPoint;

      pointCount = 1;

      if (!doVoxelGridEnDecoding_)
      {
        // get current cloud size
        cloudSize = output_->points.size ();

        // get amount of point to be decoded
        pointCount = *pointCountDataVectorIterator_;
        pointCountDataVectorIterator_++;

        // increase point cloud by amount of voxel points
        for (i = 0; i < pointCount; i++)
          output_->points.push_back (newPoint);

        // calculcate position of lower voxel corner
        lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * this->resolution_ + this->minX_;
        lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * this->resolution_ + this->minY_;
        lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * this->resolution_ + this->minZ_;

        // decode differentially encoded points
        pointCoder_.decodePoints (output_, lowerVoxelCorner, cloudSize, cloudSize + pointCount);
      }
      else
      {
        // calculate center of lower voxel corner
        newPoint.x = static_cast<float> ((static_cast<double> (key_arg.x) + 0.5) * this->resolution_ + this->minX_);
        newPoint.y = static_cast<float> ((static_cast<double> (key_arg.y) + 0.5) * this->resolution_ + this->minY_);
        newPoint.z = static_cast<float> ((static_cast<double> (key_arg.z) + 0.5) * this->resolution_ + this->minZ_);

        // add point to point cloud
        output_->points.push_back (newPoint);
      }

      if (cloudWithColor_)
      {
        if (dataWithColor_)
          // decode color information
          colorCoder_.decodePoints (output_, output_->points.size () - pointCount,
                                    output_->points.size (), pointColorOffset_);
        else
          // set default color information
          colorCoder_.setDefaultColor (output_, output_->points.size () - pointCount,
                                       output_->points.size (), pointColorOffset_);
      }
    }
  }
}

#define PCL_INSTANTIATE_PointCloudCompression(T) template class PCL_EXPORTS pcl::octree::PointCloudCompression<T, pcl::octree::OctreeContainerDataTVector<int>, pcl::octree::OctreeContainerEmpty<int>, pcl::octree::Octree2BufBase<int, pcl::octree::OctreeContainerDataTVector<int>, pcl::octree::OctreeContainerEmpty<int> > >;

#endif

