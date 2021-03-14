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

#include <pcl/common/io.h> // for getFieldIndex
#include <pcl/compression/entropy_range_coder.h>

#include <iostream>
#include <vector>
#include <cstring>

namespace pcl
{
  namespace io
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void OctreePointCloudCompression<
        PointT, LeafT, BranchT, OctreeT>::encodePointCloud (
        const PointCloudConstPtr &cloud_arg,
        std::ostream& compressed_tree_data_out_arg)
    {
      unsigned char recent_tree_depth =
          static_cast<unsigned char> (this->getTreeDepth ());

      // initialize octree
      this->setInputCloud (cloud_arg);

      // add point to octree
      this->addPointsFromInputCloud ();

      // make sure cloud contains points
      if (this->leaf_count_>0) {


        // color field analysis
        cloud_with_color_ = false;
        std::vector<pcl::PCLPointField> fields;
        int rgba_index = -1;
        rgba_index = pcl::getFieldIndex<PointT> ("rgb", fields);
        if (rgba_index == -1)
        {
          rgba_index = pcl::getFieldIndex<PointT> ("rgba", fields);
        }
        if (rgba_index >= 0)
        {
          point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
          cloud_with_color_ = true;
        }

        // apply encoding configuration
        cloud_with_color_ &= do_color_encoding_;


        // if octree depth changed, we enforce I-frame encoding
        i_frame_ |= (recent_tree_depth != this->getTreeDepth ());// | !(iFrameCounter%10);

        // enable I-frame rate
        if (i_frame_counter_++==i_frame_rate_)
        {
          i_frame_counter_ =0;
          i_frame_ = true;
        }

        // increase frameID
        frame_ID_++;

        // do octree encoding
        if (!do_voxel_grid_enDecoding_)
        {
          point_count_data_vector_.clear ();
          point_count_data_vector_.reserve (cloud_arg->size ());
        }

        // initialize color encoding
        color_coder_.initializeEncoding ();
        color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->size ()));
        color_coder_.setVoxelCount (static_cast<unsigned int> (this->leaf_count_));

        // initialize point encoding
        point_coder_.initializeEncoding ();
        point_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->size ()));

        // serialize octree
        if (i_frame_)
          // i-frame encoding - encode tree structure without referencing previous buffer
          this->serializeTree (binary_tree_data_vector_, false);
        else
          // p-frame encoding - XOR encoded tree structure
          this->serializeTree (binary_tree_data_vector_, true);


        // write frame header information to stream
        this->writeFrameHeader (compressed_tree_data_out_arg);

        // apply entropy coding to the content of all data vectors and send data to output stream
        this->entropyEncoding (compressed_tree_data_out_arg);

        // prepare for next frame
        this->switchBuffers ();

        // reset object count
        object_count_ = 0;

        if (b_show_statistics_)
        {
          float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
          float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);

          PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
          PCL_INFO ("Frame ID: %d\n", frame_ID_);
          if (i_frame_)
            PCL_INFO ("Encoding Frame: Intra frame\n");
          else
            PCL_INFO ("Encoding Frame: Prediction frame\n");
          PCL_INFO ("Number of encoded points: %ld\n", point_count_);
          PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
          PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
          PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
          PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
          PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
          PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_color_data_len_) / 1024.0f);
          PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ + bytes_per_color);
          PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
          PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
        }
        
        i_frame_ = false;
      } else {
        if (b_show_statistics_)
        PCL_INFO ("Info: Dropping empty point cloud\n");
        this->deleteTree();
        i_frame_counter_ = 0;
        i_frame_ = true;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::decodePointCloud (
        std::istream& compressed_tree_data_in_arg,
        PointCloudPtr &cloud_arg)
    {

      // synchronize to frame header
      syncToHeader(compressed_tree_data_in_arg);

      // initialize octree
      this->switchBuffers ();
      this->setOutputCloud (cloud_arg);

      // color field analysis
      cloud_with_color_ = false;
      std::vector<pcl::PCLPointField> fields;
      int rgba_index = -1;
      rgba_index = pcl::getFieldIndex<PointT> ("rgb", fields);
      if (rgba_index == -1)
        rgba_index = pcl::getFieldIndex<PointT> ("rgba", fields);
      if (rgba_index >= 0)
      {
        point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
        cloud_with_color_ = true;
      }

      // read header from input stream
      this->readFrameHeader (compressed_tree_data_in_arg);

      // decode data vectors from stream
      this->entropyDecoding (compressed_tree_data_in_arg);

      // initialize color and point encoding
      color_coder_.initializeDecoding ();
      point_coder_.initializeDecoding ();

      // initialize output cloud
      output_->points.clear ();
      output_->points.reserve (static_cast<std::size_t> (point_count_));

      if (i_frame_)
        // i-frame decoding - decode tree structure without referencing previous buffer
        this->deserializeTree (binary_tree_data_vector_, false);
      else
        // p-frame decoding - decode XOR encoded tree structure
        this->deserializeTree (binary_tree_data_vector_, true);

      // assign point cloud properties
      output_->height = 1;
      output_->width = cloud_arg->size ();
      output_->is_dense = false;

      if (b_show_statistics_)
      {
        float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
        float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);

        PCL_INFO ("*** POINTCLOUD DECODING ***\n");
        PCL_INFO ("Frame ID: %d\n", frame_ID_);
        if (i_frame_)
          PCL_INFO ("Decoding Frame: Intra frame\n");
        else
          PCL_INFO ("Decoding Frame: Prediction frame\n");
        PCL_INFO ("Number of decoded points: %ld\n", point_count_);
        PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
        PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
        PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
        PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
        PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_color_data_len_) / 1024.0f);
        PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ + bytes_per_color);
        PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
        PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::entropyEncoding (std::ostream& compressed_tree_data_out_arg)
    {
      std::uint64_t binary_tree_data_vector_size;
      std::uint64_t point_avg_color_data_vector_size;

      compressed_point_data_len_ = 0;
      compressed_color_data_len_ = 0;

      // encode binary octree structure
      binary_tree_data_vector_size = binary_tree_data_vector_.size ();
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
      compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream (binary_tree_data_vector_,
                                                                             compressed_tree_data_out_arg);

      if (cloud_with_color_)
      {
        // encode averaged voxel color information
        std::vector<char>& pointAvgColorDataVector = color_coder_.getAverageDataVector ();
        point_avg_color_data_vector_size = pointAvgColorDataVector.size ();
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_avg_color_data_vector_size),
                                            sizeof (point_avg_color_data_vector_size));
        compressed_color_data_len_ += entropy_coder_.encodeCharVectorToStream (pointAvgColorDataVector,
                                                                               compressed_tree_data_out_arg);
      }

      if (!do_voxel_grid_enDecoding_)
      {
        std::uint64_t pointCountDataVector_size;
        std::uint64_t point_diff_data_vector_size;
        std::uint64_t point_diff_color_data_vector_size;

        // encode amount of points per voxel
        pointCountDataVector_size = point_count_data_vector_.size ();
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&pointCountDataVector_size), sizeof (pointCountDataVector_size));
        compressed_point_data_len_ += entropy_coder_.encodeIntVectorToStream (point_count_data_vector_,
                                                                          compressed_tree_data_out_arg);

        // encode differential point information
        std::vector<char>& point_diff_data_vector = point_coder_.getDifferentialDataVector ();
        point_diff_data_vector_size = point_diff_data_vector.size ();
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_diff_data_vector_size), sizeof (point_diff_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream (point_diff_data_vector,
                                                                               compressed_tree_data_out_arg);
        if (cloud_with_color_)
        {
          // encode differential color information
          std::vector<char>& point_diff_color_data_vector = color_coder_.getDifferentialDataVector ();
          point_diff_color_data_vector_size = point_diff_color_data_vector.size ();
          compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_diff_color_data_vector_size),
                                           sizeof (point_diff_color_data_vector_size));
          compressed_color_data_len_ += entropy_coder_.encodeCharVectorToStream (point_diff_color_data_vector,
                                                                                 compressed_tree_data_out_arg);
        }
      }
      // flush output stream
      compressed_tree_data_out_arg.flush ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::entropyDecoding (std::istream& compressed_tree_data_in_arg)
    {
      std::uint64_t binary_tree_data_vector_size;
      std::uint64_t point_avg_color_data_vector_size;

      compressed_point_data_len_ = 0;
      compressed_color_data_len_ = 0;

      // decode binary octree structure
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
      binary_tree_data_vector_.resize (static_cast<std::size_t> (binary_tree_data_vector_size));
      compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg,
                                                                         binary_tree_data_vector_);

      if (data_with_color_)
      {
        // decode averaged voxel color information
        std::vector<char>& point_avg_color_data_vector = color_coder_.getAverageDataVector ();
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_avg_color_data_vector_size), sizeof (point_avg_color_data_vector_size));
        point_avg_color_data_vector.resize (static_cast<std::size_t> (point_avg_color_data_vector_size));
        compressed_color_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg,
                                                                           point_avg_color_data_vector);
      }

      if (!do_voxel_grid_enDecoding_)
      {
        std::uint64_t point_count_data_vector_size;
        std::uint64_t point_diff_data_vector_size;
        std::uint64_t point_diff_color_data_vector_size;

        // decode amount of points per voxel
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_count_data_vector_size), sizeof (point_count_data_vector_size));
        point_count_data_vector_.resize (static_cast<std::size_t> (point_count_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.decodeStreamToIntVector (compressed_tree_data_in_arg, point_count_data_vector_);
        point_count_data_vector_iterator_ = point_count_data_vector_.begin ();

        // decode differential point information
        std::vector<char>& pointDiffDataVector = point_coder_.getDifferentialDataVector ();
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_diff_data_vector_size), sizeof (point_diff_data_vector_size));
        pointDiffDataVector.resize (static_cast<std::size_t> (point_diff_data_vector_size));
        compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg,
                                                                           pointDiffDataVector);

        if (data_with_color_)
        {
          // decode differential color information
          std::vector<char>& pointDiffColorDataVector = color_coder_.getDifferentialDataVector ();
          compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_diff_color_data_vector_size), sizeof (point_diff_color_data_vector_size));
          pointDiffColorDataVector.resize (static_cast<std::size_t> (point_diff_color_data_vector_size));
          compressed_color_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg,
                                                                             pointDiffColorDataVector);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::writeFrameHeader (std::ostream& compressed_tree_data_out_arg)
    {
      // encode header identifier
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier_), strlen (frame_header_identifier_));
      // encode point cloud header id
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&frame_ID_), sizeof (frame_ID_));
      // encode frame type (I/P-frame)
      compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&i_frame_), sizeof (i_frame_));
      if (i_frame_)
      {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        double octree_resolution;
        unsigned char color_bit_depth;
        double point_resolution;

        // get current configuration
        octree_resolution = this->getResolution ();
        color_bit_depth  = color_coder_.getBitDepth ();
        point_resolution= point_coder_.getPrecision ();
        this->getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

        // encode amount of points
        if (do_voxel_grid_enDecoding_)
          point_count_ = this->leaf_count_;
        else
          point_count_ = this->object_count_;

        // encode coding configuration
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&do_voxel_grid_enDecoding_), sizeof (do_voxel_grid_enDecoding_));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&cloud_with_color_), sizeof (cloud_with_color_));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_), sizeof (point_count_));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&octree_resolution), sizeof (octree_resolution));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&color_bit_depth), sizeof (color_bit_depth));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_resolution), sizeof (point_resolution));

        // encode octree bounding box
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x), sizeof (min_x));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y), sizeof (min_y));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z), sizeof (min_z));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x), sizeof (max_x));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y), sizeof (max_y));
        compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z), sizeof (max_z));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::syncToHeader ( std::istream& compressed_tree_data_in_arg)
    {
      // sync to frame header
      unsigned int header_id_pos = 0;
      while (header_id_pos < strlen (frame_header_identifier_))
      {
        char readChar;
        compressed_tree_data_in_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
        if (readChar != frame_header_identifier_[header_id_pos++])
          header_id_pos = (frame_header_identifier_[0]==readChar)?1:0;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::readFrameHeader ( std::istream& compressed_tree_data_in_arg)
    {
      // read header
      compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&frame_ID_), sizeof (frame_ID_));
      compressed_tree_data_in_arg.read (reinterpret_cast<char*>(&i_frame_), sizeof (i_frame_));
      if (i_frame_)
      {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        double octree_resolution;
        unsigned char color_bit_depth;
        double point_resolution;

        // read coder configuration
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&do_voxel_grid_enDecoding_), sizeof (do_voxel_grid_enDecoding_));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&data_with_color_), sizeof (data_with_color_));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_count_), sizeof (point_count_));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&octree_resolution), sizeof (octree_resolution));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&color_bit_depth), sizeof (color_bit_depth));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_resolution), sizeof (point_resolution));

        // read octree bounding box
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_x), sizeof (min_x));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_y), sizeof (min_y));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_z), sizeof (min_z));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_x), sizeof (max_x));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_y), sizeof (max_y));
        compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_z), sizeof (max_z));

        // reset octree and assign new bounding box & resolution
        this->deleteTree ();
        this->setResolution (octree_resolution);
        this->defineBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

        // configure color & point coding
        color_coder_.setBitDepth (color_bit_depth);
        point_coder_.setPrecision (static_cast<float> (point_resolution));
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::serializeTreeCallback (
        LeafT &leaf_arg, const OctreeKey & key_arg)
    {
      // reference to point indices vector stored within octree leaf
      const auto& leafIdx = leaf_arg.getPointIndicesVector();

      if (!do_voxel_grid_enDecoding_)
      {
        double lowerVoxelCorner[3];

        // encode amount of points within voxel
        point_count_data_vector_.push_back (static_cast<int> (leafIdx.size ()));

        // calculate lower voxel corner based on octree key
        lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * this->resolution_ + this->min_x_;
        lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * this->resolution_ + this->min_y_;
        lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * this->resolution_ + this->min_z_;

        // differentially encode points to lower voxel corner
        point_coder_.encodePoints (leafIdx, lowerVoxelCorner, this->input_);

        if (cloud_with_color_)
          // encode color of points
          color_coder_.encodePoints (leafIdx, point_color_offset_, this->input_);
      }
      else
      {
        if (cloud_with_color_)
          // encode average color of all points within voxel
          color_coder_.encodeAverageOfPoints (leafIdx, point_color_offset_, this->input_);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT> void
    OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::deserializeTreeCallback (LeafT&,
        const OctreeKey& key_arg)
    {
      PointT newPoint;

      std::size_t pointCount = 1;

      if (!do_voxel_grid_enDecoding_)
      {
        // get current cloud size
        const auto cloudSize = output_->size ();

        // get amount of point to be decoded
        pointCount = *point_count_data_vector_iterator_;
        point_count_data_vector_iterator_++;

        // increase point cloud by amount of voxel points
        for (std::size_t i = 0; i < pointCount; i++)
          output_->points.push_back (newPoint);

        // calculate position of lower voxel corner
        double lowerVoxelCorner[3];
        lowerVoxelCorner[0] = static_cast<double> (key_arg.x) * this->resolution_ + this->min_x_;
        lowerVoxelCorner[1] = static_cast<double> (key_arg.y) * this->resolution_ + this->min_y_;
        lowerVoxelCorner[2] = static_cast<double> (key_arg.z) * this->resolution_ + this->min_z_;

        // decode differentially encoded points
        point_coder_.decodePoints (output_, lowerVoxelCorner, cloudSize, cloudSize + pointCount);
      }
      else
      {
        // calculate center of lower voxel corner
        newPoint.x = static_cast<float> ((static_cast<double> (key_arg.x) + 0.5) * this->resolution_ + this->min_x_);
        newPoint.y = static_cast<float> ((static_cast<double> (key_arg.y) + 0.5) * this->resolution_ + this->min_y_);
        newPoint.z = static_cast<float> ((static_cast<double> (key_arg.z) + 0.5) * this->resolution_ + this->min_z_);

        // add point to point cloud
        output_->points.push_back (newPoint);
      }

      if (cloud_with_color_)
      {
        if (data_with_color_)
          // decode color information
          color_coder_.decodePoints (output_, output_->size () - pointCount,
                                    output_->size (), point_color_offset_);
        else
          // set default color information
          color_coder_.setDefaultColor (output_, output_->size () - pointCount,
                                       output_->size (), point_color_offset_);
      }
    }
  }
}

#endif

