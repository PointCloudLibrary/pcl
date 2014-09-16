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
 */

#ifndef PCL_ORGANIZED_POINT_COMPRESSION_H_
#define PCL_ORGANIZED_POINT_COMPRESSION_H_

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

#include <pcl/common/boost.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <pcl/io/openni_camera/openni_shift_to_depth_conversion.h>

#include <vector>

namespace pcl
{
  namespace io
  {
    /** \author Julius Kammerl (julius@kammerl.de)
     */
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
         * \param[in] convertToMono: convert rgb to mono
         * \param[in] pngLevel_arg: png compression level (default compression: -1)
         * \param[in] bShowStatistics_arg:  show statistics
         */
        void encodePointCloud (const PointCloudConstPtr &cloud_arg,
                               std::ostream& compressedDataOut_arg,
                               bool doColorEncoding = false,
                               bool convertToMono = false,
                               bool bShowStatistics_arg = true,
                               int pngLevel_arg = -1);

        /** \brief Encode raw disparity map and color image.
         * \note Default values are configured according to the kinect/asus device specifications
         * \param[in] disparityMap_arg:  pointer to raw 16-bit disparity map
         * \param[in] colorImage_arg:  pointer to raw 8-bit rgb color image
         * \param[in] width_arg:  width of disparity map/color image
         * \param[in] height_arg:  height of disparity map/color image
         * \param[out] compressedDataOut_arg:  binary output stream containing compressed data
         * \param[in] doColorEncoding: encode color information (if available)
         * \param[in] convertToMono: convert rgb to mono
         * \param[in] pngLevel_arg: png compression level (default compression: -1)
         * \param[in] bShowStatistics_arg:  show statistics
         * \param[in] focalLength_arg focal length
         * \param[in] disparityShift_arg disparity shift
         * \param[in] disparityScale_arg disparity scaling
         */
        void encodeRawDisparityMapWithColorImage ( std::vector<uint16_t>& disparityMap_arg,
                                                   std::vector<uint8_t>& colorImage_arg,
                                                   uint32_t width_arg,
                                                   uint32_t height_arg,
                                                   std::ostream& compressedDataOut_arg,
                                                   bool doColorEncoding = false,
                                                   bool convertToMono = false,
                                                   bool bShowStatistics_arg = true,
                                                   int pngLevel_arg = -1,
                                                   float focalLength_arg = 525.0f,
                                                   float disparityShift_arg = 174.825f,
                                                   float disparityScale_arg = -0.161175f);

        /** \brief Decode point cloud from input stream
         * \param[in] compressedDataIn_arg: binary input stream containing compressed data
         * \param[out] cloud_arg: reference to decoded point cloud
         * \param[in] bShowStatistics_arg: show compression statistics during decoding
         * \return false if an I/O error occured.
         */
        bool decodePointCloud (std::istream& compressedDataIn_arg,
                               PointCloudPtr &cloud_arg,
                               bool bShowStatistics_arg = true);

      protected:
        /** \brief Analyze input point cloud and calculate the maximum depth and focal length
         * \param[in] cloud_arg: input point cloud
         * \param[out] maxDepth_arg: calculated maximum depth
         * \param[out] focalLength_arg: estimated focal length
         */
        void analyzeOrganizedCloud (PointCloudConstPtr cloud_arg,
                                    float& maxDepth_arg,
                                    float& focalLength_arg) const;

      private:
        // frame header identifier
        static const char* frameHeaderIdentifier_;

        //
        openni_wrapper::ShiftToDepthConverter sd_converter_;
    };

    // define frame identifier
    template<typename PointT>
    const char* OrganizedPointCloudCompression<PointT>::frameHeaderIdentifier_ = "<PCL-ORG-COMPRESSED>";
  }
}

#endif
