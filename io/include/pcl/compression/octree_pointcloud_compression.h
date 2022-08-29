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

#pragma once

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_pointcloud.h>
#include "entropy_range_coder.h"
#include "color_coding.h"
#include "point_coding.h"

#include "compression_profiles.h"

#include <iostream>
#include <vector>

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
    template<typename PointT, typename LeafT = OctreeContainerPointIndices,
        typename BranchT = OctreeContainerEmpty,
        typename OctreeT = Octree2BufBase<LeafT, BranchT> >
    class OctreePointCloudCompression : public OctreePointCloud<PointT, LeafT,
        BranchT, OctreeT>
    {
      public:
        // public typedefs
        using PointCloud = typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud;
        using PointCloudPtr = typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr;
        using PointCloudConstPtr = typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr;

        // Boost shared pointers
        using Ptr = shared_ptr<OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT> >;
        using ConstPtr = shared_ptr<const OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT> >;

        using LeafNode = typename OctreeT::LeafNode;
        using BranchNode = typename OctreeT::BranchNode;

        using RealTimeStreamCompression = OctreePointCloudCompression<PointT, LeafT, BranchT, Octree2BufBase<LeafT, BranchT> >;
        using SinglePointCloudCompressionLowMemory = OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeBase<LeafT, BranchT> >;


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
          color_coder_ (),
          point_coder_ (),
          do_voxel_grid_enDecoding_ (doVoxelGridDownDownSampling_arg), i_frame_rate_ (iFrameRate_arg),
          i_frame_counter_ (0), frame_ID_ (0), point_count_ (0), i_frame_ (true),
          do_color_encoding_ (doColorEncoding_arg), cloud_with_color_ (false), data_with_color_ (false),
          point_color_offset_ (0), b_show_statistics_ (showStatistics_arg), 
          compressed_point_data_len_ (), compressed_color_data_len_ (), selected_profile_(compressionProfile_arg),
          point_resolution_(pointResolution_arg), octree_resolution_(octreeResolution_arg),
          color_bit_resolution_(colorBitResolution_arg),
          object_count_(0)
        {
          initialization();
        }

        /** \brief Empty deconstructor. */
        
        ~OctreePointCloudCompression () override = default;

        /** \brief Initialize globals */
        void initialization () {
          if (selected_profile_ != MANUAL_CONFIGURATION)
          {
            // apply selected compression profile

            // retrieve profile settings
            const configurationProfile_t selectedProfile = compressionProfiles_[selected_profile_];

            // apply profile settings
            i_frame_rate_ = selectedProfile.iFrameRate;
            do_voxel_grid_enDecoding_ = selectedProfile.doVoxelGridDownSampling;
            this->setResolution (selectedProfile.octreeResolution);
            point_coder_.setPrecision (static_cast<float> (selectedProfile.pointResolution));
            do_color_encoding_ = selectedProfile.doColorEncoding;
            color_coder_.setBitDepth (selectedProfile.colorBitResolution);

          }
          else 
          {
            // configure point & color coder
            point_coder_.setPrecision (static_cast<float> (point_resolution_));
            color_coder_.setBitDepth (color_bit_resolution_);
          }

          if (point_coder_.getPrecision () == this->getResolution ())
            //disable differential point colding
            do_voxel_grid_enDecoding_ = true;

        }

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] pointIdx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        void
        addPointIdx (const uindex_t pointIdx_arg) override
        {
          ++object_count_;
          OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointIdx(pointIdx_arg);
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
          * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
          */
        void
        encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);

        /** \brief Decode point cloud from input stream
          * \param compressed_tree_data_in_arg: binary input stream containing compressed data
          * \param cloud_arg: reference to decoded point cloud
          * \warning This function is blocking until there is data available from the input stream. If the stream never contains any data, this will hang forever!
          */
        void
        decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);

      protected:

        /** \brief Write frame information to output stream
          * \param compressed_tree_data_out_arg: binary output stream
          */
        void
        writeFrameHeader (std::ostream& compressed_tree_data_out_arg);

        /** \brief Read frame information to output stream
          * \param compressed_tree_data_in_arg: binary input stream
          */
        void
        readFrameHeader (std::istream& compressed_tree_data_in_arg);

        /** \brief Synchronize to frame header
          * \param compressed_tree_data_in_arg: binary input stream
          */
        void
        syncToHeader (std::istream& compressed_tree_data_in_arg);

        /** \brief Apply entropy encoding to encoded information and output to binary stream
          * \param compressed_tree_data_out_arg: binary output stream
          */
        void
        entropyEncoding (std::ostream& compressed_tree_data_out_arg);

        /** \brief Entropy decoding of input binary stream and output to information vectors
          * \param compressed_tree_data_in_arg: binary input stream
          */
        void
        entropyDecoding (std::istream& compressed_tree_data_in_arg);

        /** \brief Encode leaf node information during serialization
          * \param leaf_arg: reference to new leaf node
          * \param key_arg: octree key of new leaf node
         */
        void
        serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg) override;

        /** \brief Decode leaf nodes information during deserialization
         * \param key_arg octree key of new leaf node
         */
        // param leaf_arg reference to new leaf node
        void
        deserializeTreeCallback (LeafT&, const OctreeKey& key_arg) override;


        /** \brief Pointer to output point cloud dataset. */
        PointCloudPtr output_;

        /** \brief Vector for storing binary tree structure */
        std::vector<char> binary_tree_data_vector_;

        /** \brief Vector for storing points per voxel information  */
        std::vector<unsigned int> point_count_data_vector_;

        /** \brief Iterator on points per voxel vector */
        std::vector<unsigned int>::const_iterator point_count_data_vector_iterator_;

        /** \brief Color coding instance */
        ColorCoding<PointT> color_coder_;

        /** \brief Point coding instance */
        PointCoding<PointT> point_coder_;

        /** \brief Static range coder instance */
        StaticRangeCoder entropy_coder_;

        bool do_voxel_grid_enDecoding_;
        std::uint32_t i_frame_rate_;
        std::uint32_t i_frame_counter_;
        std::uint32_t frame_ID_;
        std::uint64_t point_count_;
        bool i_frame_;

        bool do_color_encoding_;
        bool cloud_with_color_;
        bool data_with_color_;
        unsigned char point_color_offset_;

        //bool activating statistics
        bool b_show_statistics_;
        std::uint64_t compressed_point_data_len_;
        std::uint64_t compressed_color_data_len_;

        // frame header identifier
        static const char* frame_header_identifier_;

        const compression_Profiles_e selected_profile_;
        const double point_resolution_;
        const double octree_resolution_;
        const unsigned char color_bit_resolution_;

        std::size_t object_count_;

      };

    // define frame identifier
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
      const char* OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::frame_header_identifier_ = "<PCL-OCT-COMPRESSED>";
  }

}
