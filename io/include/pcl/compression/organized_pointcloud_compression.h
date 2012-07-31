/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 * Authors: Julius Kammerl (julius@kammerl.de)
 */

#ifndef PCL_ORGANIZED_POINT_COMPRESSION_H_
#define PCL_ORGANIZED_POINT_COMPRESSION_H_

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

#include <pcl/common/boost.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <vector>

namespace pcl
{
  namespace io
  {
    template<typename PointT>
    class OrganizedPointCloudCompression
    {
      public:
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        /** \brief Empty Constructor. */
        OrganizedPointCloudCompression ()
        {
        }

        /** \brief Empty deconstructor. */
        virtual ~OrganizedPointCloudCompression ()
        {
        }

        /** \brief Encode point cloud to output stream
         * \param[in] cloud_arg:  point cloud to be compressed
         * \param[out] compressedDataOut_arg:  binary output stream containing compressed data
         * \param[in] doColorEncoding: encode color information (if available)
         * \param[in] depthQuantization_arg:  quantization parameter of inverse depth coding
         * \param[in] pngLevel_arg: png compression level (default compression: -1)
         * \param[in] bShowStatistics_arg:  show statistics
         */
        void encodePointCloud (const PointCloudConstPtr &cloud_arg,
                               std::ostream& compressedDataOut_arg,
                               bool doColorEncoding = false,
                               float depthQuantization_arg = 150.0f,
                               int pngLevel_arg = -1,
                               bool bShowStatistics_arg = true);

        /** \brief Decode point cloud from input stream
         * \param[in] compressedDataIn_arg: binary input stream containing compressed data
         * \param[out] cloud_arg: reference to decoded point cloud
         * \param[in] bShowStatistics_arg: show compression statistics during decoding
         */
        void decodePointCloud (std::istream& compressedDataIn_arg,
                               PointCloudPtr &cloud_arg,
                               bool bShowStatistics_arg = true);

      protected:
        /** \brief Analyze input point cloud and calculate the maximum depth and focal length
         * \param[in] cloud_arg: input point cloud
         * \param[out] maxDepth_arg: calculated maximum depth
         * \param[out] vocalLength_arg: estimated vocal length
         */
        void analyzeOrganizedCloud (PointCloudConstPtr cloud_arg,
                                    float& maxDepth_arg,
                                    float& vocalLength_arg) const;

      private:
        // frame header identifier
        static const char* frameHeaderIdentifier_;
    };

    // define frame identifier
    template<typename PointT>
    const char* OrganizedPointCloudCompression<PointT>::frameHeaderIdentifier_ = "<PCL-ORG-COMPRESSED>";
  }
}

#endif
