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

#ifndef ORGANIZED_COMPRESSION_HPP
#define ORGANIZED_COMPRESSION_HPP

#include <pcl/compression/organized_pointcloud_compression.h>

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>

#include <pcl/common/boost.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <pcl/compression/libpng_wrapper.h>
#include <pcl/compression/organized_pointcloud_conversion.h>

#include <string>
#include <vector>
#include <limits>
#include <assert.h>

namespace pcl
{
  namespace io
  {
    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> void
    OrganizedPointCloudCompression<PointT>::encodePointCloud (const PointCloudConstPtr &cloud_arg,
                                                              std::ostream& compressedDataOut_arg,
                                                              bool doColorEncoding,
                                                              bool convertToMono,
                                                              bool bShowStatistics_arg,
                                                              int pngLevel_arg)
    {
      uint32_t cloud_width = cloud_arg->width;
      uint32_t cloud_height = cloud_arg->height;

      float maxDepth, focalLength, disparityShift, disparityScale;

      // no disparity scaling/shifting required during decoding
      disparityScale = 1.0f;
      disparityShift = 0.0f;

      analyzeOrganizedCloud (cloud_arg, maxDepth, focalLength);

      // encode header identifier
      compressedDataOut_arg.write (reinterpret_cast<const char*> (frameHeaderIdentifier_), strlen (frameHeaderIdentifier_));
      // encode point cloud width
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&cloud_width), sizeof (cloud_width));
      // encode frame type height
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&cloud_height), sizeof (cloud_height));
      // encode frame max depth
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&maxDepth), sizeof (maxDepth));
      // encode frame focal lenght
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&focalLength), sizeof (focalLength));
      // encode frame disparity scale
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&disparityScale), sizeof (disparityScale));
      // encode frame disparity shift
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&disparityShift), sizeof (disparityShift));

      // disparity and rgb image data
      std::vector<uint16_t> disparityData;
      std::vector<uint8_t> colorData;

      // compressed disparity and rgb image data
      std::vector<uint8_t> compressedDisparity;
      std::vector<uint8_t> compressedColor;

      uint32_t compressedDisparitySize = 0;
      uint32_t compressedColorSize = 0;

      // Convert point cloud to disparity and rgb image
      OrganizedConversion<PointT>::convert (*cloud_arg, focalLength, disparityShift, disparityScale, convertToMono,  disparityData, colorData);

      // Compress disparity information
      encodeMonoImageToPNG (disparityData, cloud_width, cloud_height, compressedDisparity, pngLevel_arg);

      compressedDisparitySize = static_cast<uint32_t>(compressedDisparity.size());
      // Encode size of compressed disparity image data
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedDisparitySize), sizeof (compressedDisparitySize));
      // Output compressed disparity to ostream
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedDisparity[0]), compressedDisparity.size () * sizeof(uint8_t));

      // Compress color information
      if (CompressionPointTraits<PointT>::hasColor && doColorEncoding)
      {
        if (convertToMono)
        {
          encodeMonoImageToPNG (colorData, cloud_width, cloud_height, compressedColor, 1 /*Z_BEST_SPEED*/);
        } else
        {
          encodeRGBImageToPNG (colorData, cloud_width, cloud_height, compressedColor, 1 /*Z_BEST_SPEED*/);
        }
      }

      compressedColorSize = static_cast<uint32_t>(compressedColor.size ());
      // Encode size of compressed Color image data
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedColorSize), sizeof (compressedColorSize));
      // Output compressed disparity to ostream
      compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedColor[0]), compressedColor.size () * sizeof(uint8_t));

      if (bShowStatistics_arg)
      {
        uint64_t pointCount = cloud_width * cloud_height;
        float bytesPerPoint = static_cast<float> (compressedDisparitySize+compressedColorSize) / static_cast<float> (pointCount);

        PCL_INFO("*** POINTCLOUD ENCODING ***\n");
        PCL_INFO("Number of encoded points: %ld\n", pointCount);
        PCL_INFO("Size of uncompressed point cloud: %.2f kBytes\n", (static_cast<float> (pointCount) * CompressionPointTraits<PointT>::bytesPerPoint) / 1024.0f);
        PCL_INFO("Size of compressed point cloud: %.2f kBytes\n", static_cast<float> (compressedDisparitySize+compressedColorSize) / 1024.0f);
        PCL_INFO("Total bytes per point: %.4f bytes\n", static_cast<float> (bytesPerPoint));
        PCL_INFO("Total compression percentage: %.4f%%\n", (bytesPerPoint) / (CompressionPointTraits<PointT>::bytesPerPoint) * 100.0f);
        PCL_INFO("Compression ratio: %.2f\n\n", static_cast<float> (CompressionPointTraits<PointT>::bytesPerPoint) / bytesPerPoint);
      }

      // flush output stream
      compressedDataOut_arg.flush();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> void
    OrganizedPointCloudCompression<PointT>::encodeRawDisparityMapWithColorImage ( std::vector<uint16_t>& disparityMap_arg,
                                                                                  std::vector<uint8_t>& colorImage_arg,
                                                                                  uint32_t width_arg,
                                                                                  uint32_t height_arg,
                                                                                  std::ostream& compressedDataOut_arg,
                                                                                  bool doColorEncoding,
                                                                                  bool convertToMono,
                                                                                  bool bShowStatistics_arg,
                                                                                  int pngLevel_arg,
                                                                                  float focalLength_arg,
                                                                                  float disparityShift_arg,
                                                                                  float disparityScale_arg)
    {
       float maxDepth = -1;

       size_t cloud_size = width_arg*height_arg;
       assert (disparityMap_arg.size()==cloud_size);
       if (colorImage_arg.size())
       {
         assert (colorImage_arg.size()==cloud_size*3);
       }

       // encode header identifier
       compressedDataOut_arg.write (reinterpret_cast<const char*> (frameHeaderIdentifier_), strlen (frameHeaderIdentifier_));
       // encode point cloud width
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&width_arg), sizeof (width_arg));
       // encode frame type height
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&height_arg), sizeof (height_arg));
       // encode frame max depth
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&maxDepth), sizeof (maxDepth));
       // encode frame focal lenght
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&focalLength_arg), sizeof (focalLength_arg));
       // encode frame disparity scale
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&disparityScale_arg), sizeof (disparityScale_arg));
       // encode frame disparity shift
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&disparityShift_arg), sizeof (disparityShift_arg));

       // compressed disparity and rgb image data
       std::vector<uint8_t> compressedDisparity;
       std::vector<uint8_t> compressedColor;

       uint32_t compressedDisparitySize = 0;
       uint32_t compressedColorSize = 0;

       // Remove color information of invalid points
       uint16_t* depth_ptr = &disparityMap_arg[0];
       uint8_t* color_ptr = &colorImage_arg[0];

       size_t i;
       for (i=0; i<cloud_size; ++i, ++depth_ptr, color_ptr+=sizeof(uint8_t)*3)
       {
         if (!(*depth_ptr) || (*depth_ptr==0x7FF))
           memset(color_ptr, 0, sizeof(uint8_t)*3);
       }

       // Compress disparity information
       encodeMonoImageToPNG (disparityMap_arg, width_arg, height_arg, compressedDisparity, pngLevel_arg);

       compressedDisparitySize = static_cast<uint32_t>(compressedDisparity.size());
       // Encode size of compressed disparity image data
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedDisparitySize), sizeof (compressedDisparitySize));
       // Output compressed disparity to ostream
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedDisparity[0]), compressedDisparity.size () * sizeof(uint8_t));

       // Compress color information
       if (colorImage_arg.size() && doColorEncoding)
       {
         if (convertToMono)
         {
           size_t i, size;
           vector<uint8_t> monoImage;
           size = width_arg*height_arg;

           monoImage.reserve(size);

           // grayscale conversion
           for (i=0; i<size; ++i)
           {
             uint8_t grayvalue = static_cast<uint8_t>(0.2989 * static_cast<float>(colorImage_arg[i*3+0]) +
                                                      0.5870 * static_cast<float>(colorImage_arg[i*3+1]) +
                                                      0.1140 * static_cast<float>(colorImage_arg[i*3+2]));
             monoImage.push_back(grayvalue);
           }
           encodeMonoImageToPNG (monoImage, width_arg, height_arg, compressedColor, 1 /*Z_BEST_SPEED*/);

         } else
         {
           encodeRGBImageToPNG (colorImage_arg, width_arg, height_arg, compressedColor, 1 /*Z_BEST_SPEED*/);
         }

       }

       compressedColorSize = static_cast<uint32_t>(compressedColor.size ());
       // Encode size of compressed Color image data
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedColorSize), sizeof (compressedColorSize));
       // Output compressed disparity to ostream
       compressedDataOut_arg.write (reinterpret_cast<const char*> (&compressedColor[0]), compressedColor.size () * sizeof(uint8_t));

       if (bShowStatistics_arg)
       {
         uint64_t pointCount = width_arg * height_arg;
         float bytesPerPoint = static_cast<float> (compressedDisparitySize+compressedColorSize) / static_cast<float> (pointCount);

         PCL_INFO("*** POINTCLOUD ENCODING ***\n");
         PCL_INFO("Number of encoded points: %ld\n", pointCount);
         PCL_INFO("Size of uncompressed disparity map+color image: %.2f kBytes\n", (static_cast<float> (pointCount) * (sizeof(uint8_t)*3+sizeof(uint16_t))) / 1024.0f);
         PCL_INFO("Size of compressed point cloud: %.2f kBytes\n", static_cast<float> (compressedDisparitySize+compressedColorSize) / 1024.0f);
         PCL_INFO("Total bytes per point: %.4f bytes\n", static_cast<float> (bytesPerPoint));
         PCL_INFO("Total compression percentage: %.4f%%\n", (bytesPerPoint) / (sizeof(uint8_t)*3+sizeof(uint16_t)) * 100.0f);
         PCL_INFO("Compression ratio: %.2f\n\n", static_cast<float> (CompressionPointTraits<PointT>::bytesPerPoint) / bytesPerPoint);
       }

       // flush output stream
       compressedDataOut_arg.flush();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> bool
    OrganizedPointCloudCompression<PointT>::decodePointCloud (std::istream& compressedDataIn_arg,
                                                              PointCloudPtr &cloud_arg,
                                                              bool bShowStatistics_arg)
    {
      uint32_t cloud_width;
      uint32_t cloud_height;
      float maxDepth;
      float focalLength;
      float disparityShift = 0.0f;
      float disparityScale;

      // disparity and rgb image data
      std::vector<uint16_t> disparityData;
      std::vector<uint8_t> colorData;

      // compressed disparity and rgb image data
      std::vector<uint8_t> compressedDisparity;
      std::vector<uint8_t> compressedColor;

      uint32_t compressedDisparitySize;
      uint32_t compressedColorSize;

      // PNG decoded parameters
      size_t png_width = 0;
      size_t png_height = 0;
      unsigned int png_channels = 1;

      // sync to frame header
      unsigned int headerIdPos = 0;
      bool valid_stream = true;
      while (valid_stream && (headerIdPos < strlen (frameHeaderIdentifier_)))
      {
        char readChar;
        compressedDataIn_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
        if (compressedDataIn_arg.gcount()!= sizeof (readChar))
          valid_stream = false;
        if (readChar != frameHeaderIdentifier_[headerIdPos++])
          headerIdPos = (frameHeaderIdentifier_[0] == readChar) ? 1 : 0;

        valid_stream &= compressedDataIn_arg.good ();
      }

      if (valid_stream) {

        //////////////
        // reading frame header
        compressedDataIn_arg.read (reinterpret_cast<char*> (&cloud_width), sizeof (cloud_width));
        compressedDataIn_arg.read (reinterpret_cast<char*> (&cloud_height), sizeof (cloud_height));
        compressedDataIn_arg.read (reinterpret_cast<char*> (&maxDepth), sizeof (maxDepth));
        compressedDataIn_arg.read (reinterpret_cast<char*> (&focalLength), sizeof (focalLength));
        compressedDataIn_arg.read (reinterpret_cast<char*> (&disparityScale), sizeof (disparityScale));
        compressedDataIn_arg.read (reinterpret_cast<char*> (&disparityShift), sizeof (disparityShift));

        // reading compressed disparity data
        compressedDataIn_arg.read (reinterpret_cast<char*> (&compressedDisparitySize), sizeof (compressedDisparitySize));
        compressedDisparity.resize (compressedDisparitySize);
        compressedDataIn_arg.read (reinterpret_cast<char*> (&compressedDisparity[0]), compressedDisparitySize * sizeof(uint8_t));

        // reading compressed rgb data
        compressedDataIn_arg.read (reinterpret_cast<char*> (&compressedColorSize), sizeof (compressedColorSize));
        compressedColor.resize (compressedColorSize);
        compressedDataIn_arg.read (reinterpret_cast<char*> (&compressedColor[0]), compressedColorSize * sizeof(uint8_t));

        // decode PNG compressed disparity data
        decodePNGToImage (compressedDisparity, disparityData, png_width, png_height, png_channels);

        // decode PNG compressed rgb data
        decodePNGToImage (compressedColor, colorData, png_width, png_height, png_channels);
      }

      if (disparityShift==0.0f)
      {
        // reconstruct point cloud
        OrganizedConversion<PointT>::convert (disparityData,
                                              colorData,
                                              static_cast<bool>(png_channels==1),
                                              cloud_width,
                                              cloud_height,
                                              focalLength,
                                              disparityShift,
                                              disparityScale,
                                              *cloud_arg);
      } else
      {

        // we need to decode a raw shift image
        std::size_t size = disparityData.size();
        std::vector<float> depthData;
        depthData.resize(size);

        // initialize shift-to-depth converter
        if (!sd_converter_.isInitialized())
          sd_converter_.generateLookupTable();

        // convert shift to depth image
        for (std::size_t i=0; i<size; ++i)
          depthData[i] = sd_converter_.shiftToDepth(disparityData[i]);

        // reconstruct point cloud
        OrganizedConversion<PointT>::convert (depthData,
                                              colorData,
                                              static_cast<bool>(png_channels==1),
                                              cloud_width,
                                              cloud_height,
                                              focalLength,
                                              *cloud_arg);
      }

      if (bShowStatistics_arg)
      {
        uint64_t pointCount = cloud_width * cloud_height;
        float bytesPerPoint = static_cast<float> (compressedDisparitySize+compressedColorSize) / static_cast<float> (pointCount);

        PCL_INFO("*** POINTCLOUD DECODING ***\n");
        PCL_INFO("Number of encoded points: %ld\n", pointCount);
        PCL_INFO("Size of uncompressed point cloud: %.2f kBytes\n", (static_cast<float> (pointCount) * CompressionPointTraits<PointT>::bytesPerPoint) / 1024.0f);
        PCL_INFO("Size of compressed point cloud: %.2f kBytes\n", static_cast<float> (compressedDisparitySize+compressedColorSize) / 1024.0f);
        PCL_INFO("Total bytes per point: %.4f bytes\n", static_cast<float> (bytesPerPoint));
        PCL_INFO("Total compression percentage: %.4f%%\n", (bytesPerPoint) / (CompressionPointTraits<PointT>::bytesPerPoint) * 100.0f);
        PCL_INFO("Compression ratio: %.2f\n\n", static_cast<float> (CompressionPointTraits<PointT>::bytesPerPoint) / bytesPerPoint);
      }

      return valid_stream;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> void
    OrganizedPointCloudCompression<PointT>::analyzeOrganizedCloud (PointCloudConstPtr cloud_arg,
                                                                   float& maxDepth_arg,
                                                                   float& focalLength_arg) const
    {
      size_t width, height, it;
      int centerX, centerY;
      int x, y;
      float maxDepth;
      float focalLength;

      width = cloud_arg->width;
      height = cloud_arg->height;

      // Center of organized point cloud
      centerX = static_cast<int> (width / 2);
      centerY = static_cast<int> (height / 2);

      // Ensure we have an organized point cloud
      assert((width>1) && (height>1));
      assert(width*height == cloud_arg->points.size());

      maxDepth = 0;
      focalLength = 0;

      it = 0;
      for (y = -centerY; y < +centerY; ++y)
        for (x = -centerX; x < +centerX; ++x)
        {
          const PointT& point = cloud_arg->points[it++];

          if (pcl::isFinite (point))
          {
            if (maxDepth < point.z)
            {
              // Update maximum depth
              maxDepth = point.z;

              // Calculate focal length
              focalLength = 2.0f / (point.x / (static_cast<float> (x) * point.z) + point.y / (static_cast<float> (y) * point.z));
            }
          }
        }

      // Update return values
      maxDepth_arg = maxDepth;
      focalLength_arg = focalLength;
    }

  }
}

#endif

