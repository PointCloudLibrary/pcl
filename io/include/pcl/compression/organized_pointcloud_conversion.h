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

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/common/point_tests.h> // for pcl::isFinite

#include <vector>
#include <limits>
#include <cassert>

namespace pcl
{
namespace io
{

template<typename PointT> struct CompressionPointTraits
{
    static const bool hasColor = false;
    static const unsigned int channels = 1;
    static const std::size_t bytesPerPoint = 3 * sizeof(float);
};

template<>
struct CompressionPointTraits<PointXYZRGB>
{
    static const bool hasColor = true;
    static const unsigned int channels = 4;
    static const std::size_t bytesPerPoint = 3 * sizeof(float) + 3 * sizeof(std::uint8_t);
};

template<>
struct CompressionPointTraits<PointXYZRGBA>
{
    static const bool hasColor = true;
    static const unsigned int channels = 4;
    static const std::size_t bytesPerPoint = 3 * sizeof(float) + 3 * sizeof(std::uint8_t);
};

template <typename PointT, bool enableColor = CompressionPointTraits<PointT>::hasColor >
struct OrganizedConversion;

// Uncolored point cloud specialization
template<typename PointT>
struct OrganizedConversion<PointT, false>
{
  /** \brief Convert point cloud to disparity image
    * \param[in] cloud_arg input point cloud
    * \param[in] focalLength_arg focal length
    * \param[in] disparityShift_arg disparity shift
    * \param[in] disparityScale_arg disparity scaling
    * \param[out] disparityData_arg output disparity image
    * \ingroup io
    */
  static void convert(const pcl::PointCloud<PointT>& cloud_arg,
                      float focalLength_arg,
                      float disparityShift_arg,
                      float disparityScale_arg,
                      bool ,
                      typename std::vector<std::uint16_t>& disparityData_arg,
                      typename std::vector<std::uint8_t>&)
  {
    const auto cloud_size = cloud_arg.size ();

    // Clear image data
    disparityData_arg.clear ();

    disparityData_arg.reserve (cloud_size);

    for (std::size_t i = 0; i < cloud_size; ++i)
    {
      // Get point from cloud
      const PointT& point = cloud_arg[i];

      if (pcl::isFinite (point))
      {
        // Inverse depth quantization
        auto disparity = static_cast<std::uint16_t> ( focalLength_arg / (disparityScale_arg * point.z) + disparityShift_arg / disparityScale_arg);
        disparityData_arg.push_back (disparity);
      }
      else
      {
        // Non-valid points are encoded with zeros
        disparityData_arg.push_back (0);
      }
    }
  }

  /** \brief Convert disparity image to point cloud
    * \param[in] disparityData_arg input depth image
    * \param[in] width_arg width of disparity image
    * \param[in] height_arg height of disparity image
    * \param[in] focalLength_arg focal length
    * \param[in] disparityShift_arg disparity shift
    * \param[in] disparityScale_arg disparity scaling
    * \param[out] cloud_arg output point cloud
    * \ingroup io
    */
  static void convert(typename std::vector<std::uint16_t>& disparityData_arg,
                      typename std::vector<std::uint8_t>&,
                      bool,
                      std::size_t width_arg,
                      std::size_t height_arg,
                      float focalLength_arg,
                      float disparityShift_arg,
                      float disparityScale_arg,
                      pcl::PointCloud<PointT>& cloud_arg)
  {
    std::size_t cloud_size = width_arg * height_arg;

    assert(disparityData_arg.size()==cloud_size);

    // Reset point cloud
    cloud_arg.clear ();
    cloud_arg.reserve (cloud_size);

    // Define point cloud parameters
    cloud_arg.width = static_cast<std::uint32_t> (width_arg);
    cloud_arg.height = static_cast<std::uint32_t> (height_arg);
    cloud_arg.is_dense = false;

    // Calculate center of disparity image
    int centerX = static_cast<int> (width_arg / 2);
    int centerY = static_cast<int> (height_arg / 2);

    const float fl_const = 1.0f / focalLength_arg;
    static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

    std::size_t i = 0;
    for (int y = -centerY; y < centerY; ++y )
      for (int x = -centerX; x < centerX; ++x )
      {
        PointT newPoint;
        const std::uint16_t& pixel_disparity = disparityData_arg[i];
        ++i;

        if (pixel_disparity)
        {
          // Inverse depth decoding
          float depth = focalLength_arg / (static_cast<float> (pixel_disparity) * disparityScale_arg + disparityShift_arg);

          // Generate new points
          newPoint.x = static_cast<float> (x) * depth * fl_const;
          newPoint.y = static_cast<float> (y) * depth * fl_const;
          newPoint.z = depth;

        }
        else
        {
          // Generate bad point
          newPoint.x = newPoint.y = newPoint.z = bad_point;
        }

        cloud_arg.push_back (newPoint);
      }
  }

  /** \brief Convert disparity image to point cloud
    * \param[in] depthData_arg input depth image
    * \param[in] width_arg width of disparity image
    * \param[in] height_arg height of disparity image
    * \param[in] focalLength_arg focal length
    * \param[out] cloud_arg output point cloud
    * \ingroup io
    */
  static void convert(typename std::vector<float>& depthData_arg,
                      typename std::vector<std::uint8_t>&,
                      bool,
                      std::size_t width_arg,
                      std::size_t height_arg,
                      float focalLength_arg,
                      pcl::PointCloud<PointT>& cloud_arg)
  {
    std::size_t cloud_size = width_arg * height_arg;

    assert(depthData_arg.size()==cloud_size);

    // Reset point cloud
    cloud_arg.clear ();
    cloud_arg.reserve (cloud_size);

    // Define point cloud parameters
    cloud_arg.width = static_cast<std::uint32_t> (width_arg);
    cloud_arg.height = static_cast<std::uint32_t> (height_arg);
    cloud_arg.is_dense = false;

    // Calculate center of disparity image
    int centerX = static_cast<int> (width_arg / 2);
    int centerY = static_cast<int> (height_arg / 2);

    const float fl_const = 1.0f / focalLength_arg;
    static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

    std::size_t i = 0;
    for (int y = -centerY; y < centerY; ++y )
      for (int x = -centerX; x < centerX; ++x )
      {
        PointT newPoint;
        const float& pixel_depth = depthData_arg[i];
        ++i;

        if (pixel_depth)
        {
          // Generate new points
          newPoint.x = static_cast<float> (x) * pixel_depth * fl_const;
          newPoint.y = static_cast<float> (y) * pixel_depth * fl_const;
          newPoint.z = pixel_depth;

        }
        else
        {
          // Generate bad point
          newPoint.x = newPoint.y = newPoint.z = bad_point;
        }

        cloud_arg.push_back (newPoint);
      }
  }
};

// Colored point cloud specialization
template <typename PointT>
struct OrganizedConversion<PointT, true>
{
  /** \brief Convert point cloud to disparity image and rgb image
    * \param[in] cloud_arg input point cloud
    * \param[in] focalLength_arg focal length
    * \param[in] disparityShift_arg disparity shift
    * \param[in] disparityScale_arg disparity scaling
    * \param[in] convertToMono convert color to mono/grayscale
    * \param[out] disparityData_arg output disparity image
    * \param[out] rgbData_arg output rgb image
    * \ingroup io
    */
  static void convert(const pcl::PointCloud<PointT>& cloud_arg,
                      float focalLength_arg,
                      float disparityShift_arg,
                      float disparityScale_arg,
                      bool convertToMono,
                      typename std::vector<std::uint16_t>& disparityData_arg,
                      typename std::vector<std::uint8_t>& rgbData_arg)
  {
    const auto cloud_size = cloud_arg.size ();

    // Reset output vectors
    disparityData_arg.clear ();
    rgbData_arg.clear ();

    // Allocate memory
    disparityData_arg.reserve (cloud_size);
    if (convertToMono)
    {
      rgbData_arg.reserve (cloud_size);
    } else
    {
      rgbData_arg.reserve (cloud_size * 3);
    }

    for (std::size_t i = 0; i < cloud_size; ++i)
    {
      const PointT& point = cloud_arg[i];

      if (pcl::isFinite (point))
      {
        if (convertToMono)
        {
          // Encode point color
          auto grayvalue = static_cast<std::uint8_t>(0.2989 * point.r
                                                    + 0.5870 * point.g
                                                    + 0.1140 * point.b);

          rgbData_arg.push_back (grayvalue);
        } else
        {
          // Encode point color
          rgbData_arg.push_back (point.r);
          rgbData_arg.push_back (point.g);
          rgbData_arg.push_back (point.b);
        }

        // Inverse depth quantization
        auto disparity = static_cast<std::uint16_t> (focalLength_arg / (disparityScale_arg * point.z) + disparityShift_arg / disparityScale_arg);

        // Encode disparity
        disparityData_arg.push_back (disparity);
      }
      else
      {
        // Encode black point
        if (convertToMono)
        {
          rgbData_arg.push_back (0);
        } else
        {
          rgbData_arg.push_back (0);
          rgbData_arg.push_back (0);
          rgbData_arg.push_back (0);
        }

        // Encode bad point
        disparityData_arg.push_back (0);
      }
    }
  }

  /** \brief Convert disparity image to point cloud
    * \param[in] disparityData_arg output disparity image
    * \param[in] rgbData_arg output rgb image
    * \param[in] monoImage_arg input image is a single-channel mono image
    * \param[in] width_arg width of disparity image
    * \param[in] height_arg height of disparity image
    * \param[in] focalLength_arg focal length
    * \param[in] disparityShift_arg disparity shift
    * \param[in] disparityScale_arg disparity scaling
    * \param[out] cloud_arg output point cloud
    * \ingroup io
    */
  static void convert(typename std::vector<std::uint16_t>& disparityData_arg,
                      typename std::vector<std::uint8_t>& rgbData_arg,
                      bool monoImage_arg,
                      std::size_t width_arg,
                      std::size_t height_arg,
                      float focalLength_arg,
                      float disparityShift_arg,
                      float disparityScale_arg,
                      pcl::PointCloud<PointT>& cloud_arg)
  {
    std::size_t cloud_size = width_arg*height_arg;
    bool hasColor = (!rgbData_arg.empty ());

    // Check size of input data
    assert (disparityData_arg.size()==cloud_size);
    if (hasColor)
    {
      if (monoImage_arg)
      {
        assert (rgbData_arg.size()==cloud_size);
      } else
      {
        assert (rgbData_arg.size()==cloud_size*3);
      }
    }

    // Reset point cloud
    cloud_arg.clear();
    cloud_arg.reserve(cloud_size);

    // Define point cloud parameters
    cloud_arg.width = static_cast<std::uint32_t>(width_arg);
    cloud_arg.height = static_cast<std::uint32_t>(height_arg);
    cloud_arg.is_dense = false;

    // Calculate center of disparity image
    int centerX = static_cast<int>(width_arg/2);
    int centerY = static_cast<int>(height_arg/2);

    const float fl_const = 1.0f/focalLength_arg;
    static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

    std::size_t i = 0;
    for (int y = -centerY; y < centerY; ++y )
      for (int x = -centerX; x < centerX; ++x )
      {
        PointT newPoint;

        const std::uint16_t& pixel_disparity = disparityData_arg[i];

        if (pixel_disparity && (pixel_disparity!=0x7FF))
        {
          float depth = focalLength_arg / (static_cast<float> (pixel_disparity) * disparityScale_arg + disparityShift_arg);

          // Define point location
          newPoint.z = depth;
          newPoint.x = static_cast<float> (x) * depth * fl_const;
          newPoint.y = static_cast<float> (y) * depth * fl_const;

          if (hasColor)
          {
            if (monoImage_arg)
            {
              // Define point color
              newPoint.r = rgbData_arg[i];
              newPoint.g = rgbData_arg[i];
              newPoint.b = rgbData_arg[i];
            } else
            {
              // Define point color
              newPoint.r = rgbData_arg[i*3+0];
              newPoint.g = rgbData_arg[i*3+1];
              newPoint.b = rgbData_arg[i*3+2];
            }

          } else
          {
            // Set white point color
            newPoint.rgba = 0xffffffffu;
          }
        } else
        {
          // Define bad point
          newPoint.x = newPoint.y = newPoint.z = bad_point;
          newPoint.rgb = 0.0f;
        }

        // Add point to cloud
        cloud_arg.push_back(newPoint);
        // Increment point iterator
        ++i;
    }
  }

  /** \brief Convert disparity image to point cloud
    * \param[in] depthData_arg output disparity image
    * \param[in] rgbData_arg output rgb image
    * \param[in] monoImage_arg input image is a single-channel mono image
    * \param[in] width_arg width of disparity image
    * \param[in] height_arg height of disparity image
    * \param[in] focalLength_arg focal length
    * \param[out] cloud_arg output point cloud
    * \ingroup io
    */
  static void convert(typename std::vector<float>& depthData_arg,
                      typename std::vector<std::uint8_t>& rgbData_arg,
                      bool monoImage_arg,
                      std::size_t width_arg,
                      std::size_t height_arg,
                      float focalLength_arg,
                      pcl::PointCloud<PointT>& cloud_arg)
  {
    std::size_t cloud_size = width_arg*height_arg;
    bool hasColor = (!rgbData_arg.empty ());

    // Check size of input data
    assert (depthData_arg.size()==cloud_size);
    if (hasColor)
    {
      if (monoImage_arg)
      {
        assert (rgbData_arg.size()==cloud_size);
      } else
      {
        assert (rgbData_arg.size()==cloud_size*3);
      }
    }

    // Reset point cloud
    cloud_arg.clear();
    cloud_arg.reserve(cloud_size);

    // Define point cloud parameters
    cloud_arg.width = static_cast<std::uint32_t>(width_arg);
    cloud_arg.height = static_cast<std::uint32_t>(height_arg);
    cloud_arg.is_dense = false;

    // Calculate center of disparity image
    int centerX = static_cast<int>(width_arg/2);
    int centerY = static_cast<int>(height_arg/2);

    const float fl_const = 1.0f/focalLength_arg;
    static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

    std::size_t i = 0;
    for (int y = -centerY; y < centerY; ++y )
      for (int x = -centerX; x < centerX; ++x )
      {
        PointT newPoint;

        const float& pixel_depth = depthData_arg[i];

        if (pixel_depth)
        {
          // Define point location
          newPoint.z = pixel_depth;
          newPoint.x = static_cast<float> (x) * pixel_depth * fl_const;
          newPoint.y = static_cast<float> (y) * pixel_depth * fl_const;

          if (hasColor)
          {
            if (monoImage_arg)
            {
              // Define point color
              newPoint.r = rgbData_arg[i];
              newPoint.g = rgbData_arg[i];
              newPoint.b = rgbData_arg[i];
            } else
            {
              // Define point color
              newPoint.r = rgbData_arg[i*3+0];
              newPoint.g = rgbData_arg[i*3+1];
              newPoint.b = rgbData_arg[i*3+2];
            }

          } else
          {
            // Set white point color
            newPoint.rgba = 0xffffffffu;
          }
        } else
        {
          // Define bad point
          newPoint.x = newPoint.y = newPoint.z = bad_point;
          newPoint.rgb = 0.0f;
        }

        // Add point to cloud
        cloud_arg.push_back(newPoint);
        // Increment point iterator
        ++i;
    }
  }
};

} // namespace io
} // namespace pcl

