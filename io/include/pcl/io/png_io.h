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
 * Authors: Anatoly Baksheev
 */

#ifndef PCL_IO_PNG_IO_H_
#define PCL_IO_PNG_IO_H_

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <string>
#include <vector>

namespace pcl
{
  namespace io
  {
    /** \brief Saves 8-bit encoded image to PNG file.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] mono_image image grayscale data
      * \param[in] width image width
      * \param[in] height image height
      * \param[in] channels number of channels
      * \ingroup io
      */
    PCL_EXPORTS void
    saveCharPNGFile (const std::string& file_name, const unsigned char *mono_image, int width, int height, int channels);

    /** \brief Saves 16-bit encoded image to PNG file.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] short_image image short data
      * \param[in] width image width
      * \param[in] height image height
      * \param[in] channels number of channels
      * \ingroup io
      */
    PCL_EXPORTS void
    saveShortPNGFile (const std::string& file_name, const unsigned short *short_image, int width, int height, int channels);

    /** \brief Saves 8-bit encoded RGB image to PNG file.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] rgb_image image rgb data
      * \param[in] width image width
      * \param[in] height image height
      * \ingroup io
      */
    PCL_EXPORTS void 
    saveRgbPNGFile (const std::string& file_name, const unsigned char *rgb_image, int width, int height)
    {
      saveCharPNGFile(file_name, rgb_image, width, height, 3);
    }

    /** \brief Saves 8-bit grayscale cloud as image to PNG file.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] cloud point cloud to save
      * \ingroup io
      */
    void
    savePNGFile (const std::string& file_name, const pcl::PointCloud<unsigned char>& cloud)
    {
      saveCharPNGFile(file_name, &cloud.points[0], cloud.width, cloud.height, 1);
    }

    /** \brief Saves 16-bit grayscale cloud as image to PNG file.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] cloud point cloud to save
      * \ingroup io
      */

    void
    savePNGFile (const std::string& file_name, const pcl::PointCloud<unsigned short>& cloud)
    {
      saveShortPNGFile(file_name, &cloud.points[0], cloud.width, cloud.height, 1);
    }

    /** \brief Saves RGB fields of cloud as image to PNG file. 
      * \param[in] file_name the name of the file to write to disk
      * \param[in] cloud point cloud to save
      * \ingroup io
      */
    template <typename T> void
    savePNGFile (const std::string& file_name, const pcl::PointCloud<T>& cloud)
    {
      std::vector<unsigned char> data(cloud.width * cloud.height * 3);

      for (size_t i = 0; i < cloud.points.size (); ++i)
      {
        data[i*3 + 0] = cloud.points[i].r;
        data[i*3 + 1] = cloud.points[i].g;
        data[i*3 + 2] = cloud.points[i].b;        
      }
      saveRgbPNGFile(file_name, &data[0], cloud.width, cloud.height);
    }        
  }
}

#endif  //#ifndef PCL_IO_PNG_IO_H_
