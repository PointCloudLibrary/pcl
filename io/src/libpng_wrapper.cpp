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

#include <pcl/compression/libpng_wrapper.h>

#include <png.h>
#include <zlib.h>

#include <vector>
#include <cstdlib>
#include <cassert>
#include <cstring> // for memcpy
#include <iterator> // for back_inserter


// user defined I/O callback methods for libPNG
namespace 
{
  using std::uint8_t;
  /////////////////////////////////////////////////////////////////////////////////////////
  void 
  user_read_data (png_structp png_ptr, png_bytep data, png_size_t length)
  {
    auto** input_pointer = reinterpret_cast<std::uint8_t**>(png_get_io_ptr (png_ptr));

    memcpy (data, *input_pointer, sizeof (std::uint8_t) * length);
    (*input_pointer) += length;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void 
  user_write_data (png_structp png_ptr,  png_bytep data, png_size_t length)
  {
    auto* pngVec = reinterpret_cast<std::vector<std::uint8_t>*>(png_get_io_ptr (png_ptr));
    std::copy (data, data + length, std::back_inserter (*pngVec));
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void 
  user_flush_data (png_structp)
  {
  }
}

namespace pcl
{
  namespace io
  {
    ///////////////////////////////////////////////////////////////////////////////////////////
    template<typename T> void
    encodeImageToPNG (typename std::vector<T>& image_arg,
                      std::size_t width_arg,
                      std::size_t height_arg,
                      int image_format_arg,
                      typename std::vector<std::uint8_t>& pngData_arg,
                      int png_level_arg)
    {
      png_structp png_ptr;
      png_infop info_ptr;
      volatile int channels;

      if (image_arg.empty ())
        return;

      // Get amount of channels
      switch (image_format_arg)
       {
         case PNG_COLOR_TYPE_GRAY:
           channels = 1;
           break;
         case PNG_COLOR_TYPE_GRAY_ALPHA:
           channels = 2;
           break;
         case PNG_COLOR_TYPE_RGB:
           channels = 3;
           break;
         case PNG_COLOR_TYPE_RGB_ALPHA:
           channels = 4;
           break;
         default:
           channels = 0;
           break;
       }

      // Ensure valid input array
      assert (image_arg.size () == width_arg*height_arg*channels);

      // Initialize write structure
      png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
      assert (png_ptr && "creating png_create_write_structpng_create_write_struct failed");

      // Initialize info structure
      info_ptr = png_create_info_struct (png_ptr);
      assert (info_ptr && "Could not allocate info struct");

      // Setup Exception handling
      setjmp(png_jmpbuf(png_ptr));

      // reserve memory for output data (300kB)
      pngData_arg.clear ();
      pngData_arg.reserve (300 * 1024);

      // Define I/O methods
      png_set_write_fn (png_ptr, reinterpret_cast<void*> (&pngData_arg), 
                        user_write_data, user_flush_data);

      // Define zlib compression level
      if (png_level_arg >= 0)
      {
        png_set_compression_level (png_ptr, png_level_arg);
      }
      else
      {
        png_set_compression_level (png_ptr, Z_DEFAULT_COMPRESSION);
      }

      // Write header
      png_set_IHDR (png_ptr, info_ptr,
                    static_cast<png_uint_32> (width_arg), static_cast<png_uint_32> (height_arg),
                    sizeof(T) * 8,
                    image_format_arg, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                    PNG_FILTER_TYPE_DEFAULT);

      png_write_info (png_ptr, info_ptr);

      // Write image data
      for (std::size_t y = 0; y < height_arg; y++)
      {
        png_write_row (png_ptr, reinterpret_cast<png_bytep> (&image_arg[y * width_arg * channels]));
      }

      // End write
      png_write_end (png_ptr, nullptr);

      if (info_ptr)
        png_free_data (png_ptr, info_ptr, PNG_FREE_ALL, -1);
      if (png_ptr)
        png_destroy_write_struct (&png_ptr, nullptr);
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    template<typename T> void
    decodePNGImage (typename std::vector<std::uint8_t>& pngData_arg,
                    typename std::vector<T>& imageData_arg,
                    std::size_t& width_arg,
                    std::size_t& height_arg,
                    unsigned int& channels_arg)
    {
      png_structp png_ptr;
      png_infop info_ptr;
      png_uint_32 png_width;
      png_uint_32 png_height;
      int png_bit_depth, png_color_type, png_interlace_type;

      png_bytep * row_pointers;

      if (pngData_arg.empty ())
        return;

      png_ptr = png_create_read_struct (PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
      assert (png_ptr && "creating png_create_write_structpng_create_write_struct failed");

      // Initialize info structure
      info_ptr = png_create_info_struct (png_ptr);
      assert(info_ptr && "Could not allocate info struct");

      // Setup Exception handling
      setjmp (png_jmpbuf(png_ptr));

      std::uint8_t* input_pointer = &pngData_arg[0];
      png_set_read_fn (png_ptr, reinterpret_cast<void*> (&input_pointer), user_read_data);

      png_read_info (png_ptr, info_ptr);

      png_get_IHDR (png_ptr, info_ptr, &png_width, &png_height, &png_bit_depth,
          &png_color_type, &png_interlace_type, nullptr, nullptr);

      // ensure a color bit depth of 8
      assert(png_bit_depth==sizeof(T)*8);

      unsigned int png_channels;
      switch (png_color_type)
      {
        case PNG_COLOR_TYPE_GRAY:
          png_channels = 1;
          break;
        case PNG_COLOR_TYPE_GRAY_ALPHA:
          png_channels = 2;
          break;
        case PNG_COLOR_TYPE_RGB:
          png_channels = 3;
          break;
        case PNG_COLOR_TYPE_RGB_ALPHA:
          png_channels = 4;
          break;
        default:
          png_channels = 0;
          break;
      }

      width_arg = png_width;
      height_arg = png_height;
      channels_arg = png_channels;

      imageData_arg.clear ();
      imageData_arg.resize (png_height * png_width * png_channels);

      row_pointers = reinterpret_cast<png_bytep*> (malloc (sizeof(png_bytep) * png_height));

      for (unsigned long y = 0; y < png_height; y++)
        row_pointers[y] = reinterpret_cast<png_byte*> (&imageData_arg[y * png_width * png_channels]);

      png_read_image (png_ptr, row_pointers);

      if (info_ptr)
        png_free_data (png_ptr, info_ptr, PNG_FREE_ALL, -1);
      if (png_ptr)
        png_destroy_read_struct (&png_ptr, nullptr, nullptr);
      if (row_pointers)
        free (row_pointers);
    }
  } // namespace io
} // namespace pcl


///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::encodeMonoImageToPNG (std::vector<std::uint8_t>& image_arg,
                               std::size_t width_arg,
                               std::size_t height_arg,
                               std::vector<std::uint8_t>& pngData_arg,
                               int png_level_arg)
{
  encodeImageToPNG<std::uint8_t> (image_arg, 
                             static_cast<png_uint_32> (width_arg), static_cast<png_uint_32> (height_arg),
                             PNG_COLOR_TYPE_GRAY, pngData_arg, png_level_arg);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::encodeMonoImageToPNG (std::vector<std::uint16_t>& image_arg,
                               std::size_t width_arg,
                               std::size_t height_arg,
                               std::vector<std::uint8_t>& pngData_arg,
                               int png_level_arg)
{
  encodeImageToPNG<std::uint16_t> (image_arg,
                              static_cast<png_uint_32> (width_arg), static_cast<png_uint_32> (height_arg),
                              PNG_COLOR_TYPE_GRAY, pngData_arg, png_level_arg);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::encodeRGBImageToPNG (std::vector<std::uint8_t>& image_arg,
                              std::size_t width_arg,
                              std::size_t height_arg,
                              std::vector<std::uint8_t>& pngData_arg,
                              int png_level_arg)
{
  encodeImageToPNG<std::uint8_t>(image_arg,
                            static_cast<png_uint_32> (width_arg), static_cast<png_uint_32> (height_arg),
                            PNG_COLOR_TYPE_RGB, pngData_arg, png_level_arg);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::encodeRGBImageToPNG (std::vector<std::uint16_t>& image_arg,
                              std::size_t width_arg,
                              std::size_t height_arg,
                              std::vector<std::uint8_t>& pngData_arg,
                              int png_level_arg)
{
  encodeImageToPNG<std::uint16_t>(image_arg,
                             static_cast<png_uint_32> (width_arg), static_cast<png_uint_32> (height_arg),
                             PNG_COLOR_TYPE_RGB, pngData_arg, png_level_arg);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::decodePNGToImage (std::vector<std::uint8_t>& pngData_arg,
                           std::vector<std::uint8_t>& imageData_arg,
                           std::size_t& width_arg,
                           std::size_t& height_arg,
                           unsigned int& channels_arg)
{
  decodePNGImage<std::uint8_t> (pngData_arg, imageData_arg, width_arg, height_arg, channels_arg);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::io::decodePNGToImage (std::vector<std::uint8_t>& pngData_arg,
                           std::vector<std::uint16_t>& imageData_arg,
                           std::size_t& width_arg,
                           std::size_t& height_arg,
                           unsigned int& channels_arg)
{
  decodePNGImage<std::uint16_t> (pngData_arg, imageData_arg, width_arg, height_arg, channels_arg);
}

