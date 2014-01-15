/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_LZF_IMAGE_IO_HPP_
#define PCL_LZF_IMAGE_IO_HPP_

#include <pcl/console/print.h>
#include <pcl/io/debayer.h>

#define CLIP_CHAR(c) static_cast<unsigned char> ((c)>255?255:(c)<0?0:(c))

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFDepth16ImageReader::read (
    const std::string &filename, pcl::PointCloud<PointT> &cloud)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFDepth16ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight () * 2)
  {
    PCL_DEBUG ("[pcl::io::LZFDepth16ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFDepth16ImageReader::read] Are you sure %s is a 16-bit depth PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight () * 2, filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFDepth16ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Copy to PointT
  cloud.width    = getWidth ();
  cloud.height   = getHeight ();
  cloud.is_dense = true;
  cloud.resize (getWidth () * getHeight ());
  register int depth_idx = 0, point_idx = 0;
  double constant_x = 1.0 / parameters_.focal_length_x,
         constant_y = 1.0 / parameters_.focal_length_y;
  for (uint32_t v = 0; v < cloud.height; ++v)
  {
    for (register uint32_t u = 0; u < cloud.width; ++u, ++point_idx, depth_idx += 2)
    {
      PointT &pt = cloud.points[point_idx];
      unsigned short val;
      memcpy (&val, &uncompressed_data[depth_idx], sizeof (unsigned short));
      if (val == 0)
      {
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        cloud.is_dense = false;
        continue;
      }

      pt.z = static_cast<float> (val * z_multiplication_factor_);
      pt.x = (static_cast<float> (u) - static_cast<float> (parameters_.principal_point_x)) 
        * pt.z * static_cast<float> (constant_x);
      pt.y = (static_cast<float> (v) - static_cast<float> (parameters_.principal_point_y)) 
        * pt.z * static_cast<float> (constant_y);
    }
  }
  cloud.sensor_origin_.setZero ();
  cloud.sensor_orientation_.w () = 1.0f;
  cloud.sensor_orientation_.x () = 0.0f;
  cloud.sensor_orientation_.y () = 0.0f;
  cloud.sensor_orientation_.z () = 0.0f;
  return (true);
}
        
///////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFDepth16ImageReader::readOMP (const std::string &filename, 
                                         pcl::PointCloud<PointT> &cloud, 
                                         unsigned int num_threads)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFDepth16ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight () * 2)
  {
    PCL_DEBUG ("[pcl::io::LZFDepth16ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFDepth16ImageReader::read] Are you sure %s is a 16-bit depth PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight () * 2, filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFDepth16ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Copy to PointT
  cloud.width    = getWidth ();
  cloud.height   = getHeight ();
  cloud.is_dense = true;
  cloud.resize (getWidth () * getHeight ());
  double constant_x = 1.0 / parameters_.focal_length_x,
         constant_y = 1.0 / parameters_.focal_length_y;
#ifdef _OPENMP
#pragma omp parallel for num_threads (num_threads)
#endif
  for (int i = 0; i < static_cast< int> (cloud.size ()); ++i)
  {
    int u = i % cloud.width;
    int v = i / cloud.width;
    PointT &pt = cloud.points[i];
    int depth_idx = 2*i;
    unsigned short val;
    memcpy (&val, &uncompressed_data[depth_idx], sizeof (unsigned short));
    if (val == 0)
    {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
      if (cloud.is_dense)
      {
#ifdef _OPENMP
#pragma omp critical
#endif
      {
      if (cloud.is_dense)
        cloud.is_dense = false;
      }
      }
      continue;
    }

    pt.z = static_cast<float> (val * z_multiplication_factor_);
    pt.x = (static_cast<float> (u) - static_cast<float> (parameters_.principal_point_x)) 
      * pt.z * static_cast<float> (constant_x);
    pt.y = (static_cast<float> (v) - static_cast<float> (parameters_.principal_point_y)) 
      * pt.z * static_cast<float> (constant_y);
    
  }
  cloud.sensor_origin_.setZero ();
  cloud.sensor_orientation_.w () = 1.0f;
  cloud.sensor_orientation_.x () = 0.0f;
  cloud.sensor_orientation_.y () = 0.0f;
  cloud.sensor_orientation_.z () = 0.0f;
  return (true);

}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFRGB24ImageReader::read (
    const std::string &filename, pcl::PointCloud<PointT> &cloud)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFRGB24ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight () * 3)
  {
    PCL_DEBUG ("[pcl::io::LZFRGB24ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFRGB24ImageReader::read] Are you sure %s is a 24-bit RGB PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight () * 3, filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFRGB24ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Copy to PointT
  cloud.width  = getWidth ();
  cloud.height = getHeight ();
  cloud.resize (getWidth () * getHeight ());

  register int rgb_idx = 0;
  unsigned char *color_r = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);
  unsigned char *color_g = reinterpret_cast<unsigned char*> (&uncompressed_data[getWidth () * getHeight ()]);
  unsigned char *color_b = reinterpret_cast<unsigned char*> (&uncompressed_data[2 * getWidth () * getHeight ()]);

  for (size_t i = 0; i < cloud.size (); ++i, ++rgb_idx)
  {
    PointT &pt = cloud.points[i];

    pt.b = color_b[rgb_idx];
    pt.g = color_g[rgb_idx];
    pt.r = color_r[rgb_idx];
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFRGB24ImageReader::readOMP (
    const std::string &filename, pcl::PointCloud<PointT> &cloud, unsigned int num_threads)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFRGB24ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight () * 3)
  {
    PCL_DEBUG ("[pcl::io::LZFRGB24ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFRGB24ImageReader::read] Are you sure %s is a 24-bit RGB PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight () * 3, filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFRGB24ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Copy to PointT
  cloud.width  = getWidth ();
  cloud.height = getHeight ();
  cloud.resize (getWidth () * getHeight ());

  unsigned char *color_r = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);
  unsigned char *color_g = reinterpret_cast<unsigned char*> (&uncompressed_data[getWidth () * getHeight ()]);
  unsigned char *color_b = reinterpret_cast<unsigned char*> (&uncompressed_data[2 * getWidth () * getHeight ()]);

#ifdef _OPENMP
#pragma omp parallel for num_threads (num_threads)
#endif//_OPENMP
  for (long int i = 0; i < cloud.size (); ++i)
  {
    PointT &pt = cloud.points[i];

    pt.b = color_b[i];
    pt.g = color_g[i];
    pt.r = color_r[i];
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFYUV422ImageReader::read (
    const std::string &filename, pcl::PointCloud<PointT> &cloud)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFYUV422ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight () * 2)
  {
    PCL_DEBUG ("[pcl::io::LZFYUV422ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFYUV422ImageReader::read] Are you sure %s is a 16-bit YUV422 PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight (), filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFYUV422ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Convert YUV422 to RGB24 and copy to PointT
  cloud.width  = getWidth ();
  cloud.height = getHeight ();
  cloud.resize (getWidth () * getHeight ());

  int wh2 = getWidth () * getHeight () / 2;
  unsigned char *color_u = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);
  unsigned char *color_y = reinterpret_cast<unsigned char*> (&uncompressed_data[wh2]);
  unsigned char *color_v = reinterpret_cast<unsigned char*> (&uncompressed_data[wh2 + getWidth () * getHeight ()]);
  
  register int y_idx = 0;
  for (int i = 0; i < wh2; ++i, y_idx += 2)
  {
    int v = color_v[i] - 128;
    int u = color_u[i] - 128;

    PointT &pt1 = cloud.points[y_idx + 0];
    pt1.r =  CLIP_CHAR (color_y[y_idx + 0] + ((v * 18678 + 8192 ) >> 14));
    pt1.g =  CLIP_CHAR (color_y[y_idx + 0] + ((v * -9519 - u * 6472 + 8192) >> 14));
    pt1.b =  CLIP_CHAR (color_y[y_idx + 0] + ((u * 33292 + 8192 ) >> 14));

    PointT &pt2 = cloud.points[y_idx + 1];
    pt2.r =  CLIP_CHAR (color_y[y_idx + 1] + ((v * 18678 + 8192 ) >> 14));
    pt2.g =  CLIP_CHAR (color_y[y_idx + 1] + ((v * -9519 - u * 6472 + 8192) >> 14));
    pt2.b =  CLIP_CHAR (color_y[y_idx + 1] + ((u * 33292 + 8192 ) >> 14));
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFYUV422ImageReader::readOMP (
    const std::string &filename, pcl::PointCloud<PointT> &cloud, unsigned int num_threads)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFYUV422ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight () * 2)
  {
    PCL_DEBUG ("[pcl::io::LZFYUV422ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFYUV422ImageReader::read] Are you sure %s is a 16-bit YUV422 PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight (), filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFYUV422ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Convert YUV422 to RGB24 and copy to PointT
  cloud.width  = getWidth ();
  cloud.height = getHeight ();
  cloud.resize (getWidth () * getHeight ());

  int wh2 = getWidth () * getHeight () / 2;
  unsigned char *color_u = reinterpret_cast<unsigned char*> (&uncompressed_data[0]);
  unsigned char *color_y = reinterpret_cast<unsigned char*> (&uncompressed_data[wh2]);
  unsigned char *color_v = reinterpret_cast<unsigned char*> (&uncompressed_data[wh2 + getWidth () * getHeight ()]);
  
#ifdef _OPENMP
#pragma omp parallel for num_threads (num_threads)
#endif//_OPENMP
  for (int i = 0; i < wh2; ++i)
  {
    int y_idx = 2*i;
    int v = color_v[i] - 128;
    int u = color_u[i] - 128;

    PointT &pt1 = cloud.points[y_idx + 0];
    pt1.r =  CLIP_CHAR (color_y[y_idx + 0] + ((v * 18678 + 8192 ) >> 14));
    pt1.g =  CLIP_CHAR (color_y[y_idx + 0] + ((v * -9519 - u * 6472 + 8192) >> 14));
    pt1.b =  CLIP_CHAR (color_y[y_idx + 0] + ((u * 33292 + 8192 ) >> 14));

    PointT &pt2 = cloud.points[y_idx + 1];
    pt2.r =  CLIP_CHAR (color_y[y_idx + 1] + ((v * 18678 + 8192 ) >> 14));
    pt2.g =  CLIP_CHAR (color_y[y_idx + 1] + ((v * -9519 - u * 6472 + 8192) >> 14));
    pt2.b =  CLIP_CHAR (color_y[y_idx + 1] + ((u * 33292 + 8192 ) >> 14));
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFBayer8ImageReader::read (
    const std::string &filename, pcl::PointCloud<PointT> &cloud)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFBayer8ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight ())
  {
    PCL_DEBUG ("[pcl::io::LZFBayer8ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFBayer8ImageReader::read] Are you sure %s is a 8-bit Bayer PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight (), filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFBayer8ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Convert Bayer8 to RGB24
  std::vector<unsigned char> rgb_buffer (getWidth () * getHeight () * 3);
  pcl::io::DeBayer i;
  i.debayerEdgeAware (reinterpret_cast<unsigned char*> (&uncompressed_data[0]), 
                     static_cast<unsigned char*> (&rgb_buffer[0]), 
                     getWidth (), getHeight ());
  // Copy to PointT
  cloud.width  = getWidth ();
  cloud.height = getHeight ();
  cloud.resize (getWidth () * getHeight ());
  register int rgb_idx = 0;
  for (size_t i = 0; i < cloud.size (); ++i, rgb_idx += 3)
  {
    PointT &pt = cloud.points[i];

    pt.b = rgb_buffer[rgb_idx + 2];
    pt.g = rgb_buffer[rgb_idx + 1];
    pt.r = rgb_buffer[rgb_idx + 0];
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::io::LZFBayer8ImageReader::readOMP (
    const std::string &filename, pcl::PointCloud<PointT> &cloud, unsigned int num_threads)
{
  uint32_t uncompressed_size;
  std::vector<char> compressed_data;
  if (!loadImageBlob (filename, compressed_data, uncompressed_size))
  {
    PCL_ERROR ("[pcl::io::LZFBayer8ImageReader::read] Unable to read image data from %s.\n", filename.c_str ());
    return (false);
  }

  if (uncompressed_size != getWidth () * getHeight ())
  {
    PCL_DEBUG ("[pcl::io::LZFBayer8ImageReader::read] Uncompressed data has wrong size (%u), while in fact it should be %u bytes. \n[pcl::io::LZFBayer8ImageReader::read] Are you sure %s is a 8-bit Bayer PCLZF file? Identifier says: %s\n", uncompressed_size, getWidth () * getHeight (), filename.c_str (), getImageType ().c_str ());
    return (false);
  }

  std::vector<char> uncompressed_data (uncompressed_size);
  decompress (compressed_data, uncompressed_data);

  if (uncompressed_data.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFBayer8ImageReader::read] Error uncompressing data stored in %s!\n", filename.c_str ());
    return (false);
  }

  // Convert Bayer8 to RGB24
  std::vector<unsigned char> rgb_buffer (getWidth () * getHeight () * 3);
  pcl::io::DeBayer i;
  i.debayerEdgeAware (reinterpret_cast<unsigned char*> (&uncompressed_data[0]), 
                     static_cast<unsigned char*> (&rgb_buffer[0]), 
                     getWidth (), getHeight ());
  // Copy to PointT
  cloud.width  = getWidth ();
  cloud.height = getHeight ();
  cloud.resize (getWidth () * getHeight ());
#ifdef _OPENMP
#pragma omp parallel for num_threads (num_threads)
#endif//_OPENMP
  for (long int i = 0; i < cloud.size (); ++i)
  {
    PointT &pt = cloud.points[i];
    long int rgb_idx = 3*i;
    pt.b = rgb_buffer[rgb_idx + 2];
    pt.g = rgb_buffer[rgb_idx + 1];
    pt.r = rgb_buffer[rgb_idx + 0];
  }
  return (true);
}

#endif  //#ifndef PCL_LZF_IMAGE_IO_HPP_

