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
#include <pcl/console/print.h>

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

  // PNG input data with its size and the current read position
  struct PNGInputBuffer
  {
    const std::uint8_t* data;
    std::size_t size;
    std::size_t offset;
  };

  /////////////////////////////////////////////////////////////////////////////////////////
  void 
  user_read_data (png_structp png_ptr, png_bytep data, png_size_t length)
  {
    auto* input = reinterpret_cast<PNGInputBuffer*>(png_get_io_ptr (png_ptr));

    if (length > input->size - input->offset)
      png_error (png_ptr, "Read past the end of the PNG data");

    memcpy (data, input->data + input->offset, sizeof (std::uint8_t) * length);
    input->offset += length;
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

  /////////////////////////////////////////////////////////////////////////////////////////
  void
  user_error_fn (png_structp png_ptr, png_const_charp error_msg)
  {
    PCL_ERROR ("[pcl::io::libpng_wrapper] %s\n", error_msg);
    // Return to the caller's setjmp, libpng's default handler would print the message again
    png_longjmp (png_ptr, 1);
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void
  user_warning_fn (png_structp, png_const_charp warning_msg)
  {
    PCL_WARN ("[pcl::io::libpng_wrapper] %s\n", warning_msg);
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
      png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, nullptr,
                                         user_error_fn, user_warning_fn);
      if (!png_ptr)
      {
        PCL_ERROR ("[pcl::io::libpng_wrapper] Could not create the PNG write struct\n");
        pngData_arg.clear ();
        return;
      }

      // Initialize info structure
      info_ptr = png_create_info_struct (png_ptr);
      if (!info_ptr)
      {
        PCL_ERROR ("[pcl::io::libpng_wrapper] Could not allocate the PNG info struct\n");
        png_destroy_write_struct (&png_ptr, nullptr);
        pngData_arg.clear ();
        return;
      }

      // Setup Exception handling
      if (setjmp (png_jmpbuf (png_ptr)))
      {
        // libpng reported an error
        png_destroy_write_struct (&png_ptr, &info_ptr);
        pngData_arg.clear ();
        return;
      }

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

      png_destroy_write_struct (&png_ptr, &info_ptr);
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

      png_bytep * volatile row_pointers = nullptr;

      if (pngData_arg.empty ())
        return;

      const auto clear_outputs = [&] ()
      {
        imageData_arg.clear ();
        width_arg = 0;
        height_arg = 0;
        channels_arg = 0;
      };

      png_ptr = png_create_read_struct (PNG_LIBPNG_VER_STRING, nullptr,
                                        user_error_fn, user_warning_fn);
      if (!png_ptr)
      {
        PCL_ERROR ("[pcl::io::libpng_wrapper] Could not create the PNG read struct\n");
        clear_outputs ();
        return;
      }

      // Initialize info structure
      info_ptr = png_create_info_struct (png_ptr);
      if (!info_ptr)
      {
        PCL_ERROR ("[pcl::io::libpng_wrapper] Could not allocate the PNG info struct\n");
        png_destroy_read_struct (&png_ptr, nullptr, nullptr);
        clear_outputs ();
        return;
      }

      // Setup Exception handling
      if (setjmp (png_jmpbuf (png_ptr)))
      {
        // libpng reported an error, e.g. truncated or corrupt PNG data
        png_destroy_read_struct (&png_ptr, &info_ptr, nullptr);
        free (row_pointers);
        clear_outputs ();
        return;
      }

      PNGInputBuffer input {pngData_arg.data (), pngData_arg.size (), 0};
      png_set_read_fn (png_ptr, reinterpret_cast<void*> (&input), user_read_data);

      png_read_info (png_ptr, info_ptr);

      png_get_IHDR (png_ptr, info_ptr, &png_width, &png_height, &png_bit_depth,
          &png_color_type, &png_interlace_type, nullptr, nullptr);

      // ensure the color bit depth matches the output type
      if (png_bit_depth != static_cast<int> (sizeof (T) * 8))
        png_error (png_ptr, "PNG bit depth does not match the output image type");

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
          png_error (png_ptr, "Unsupported PNG color type");
      }

      const std::size_t row_size = static_cast<std::size_t> (png_width) * png_channels;
      if (png_height > imageData_arg.max_size () / row_size)
        png_error (png_ptr, "PNG image is too large");

      width_arg = png_width;
      height_arg = png_height;
      channels_arg = png_channels;

      imageData_arg.clear ();
      imageData_arg.resize (png_height * row_size);

      row_pointers = reinterpret_cast<png_bytep*> (malloc (sizeof(png_bytep) * png_height));
      if (!row_pointers)
        png_error (png_ptr, "Could not allocate row pointers");

      for (std::size_t y = 0; y < png_height; y++)
        row_pointers[y] = reinterpret_cast<png_byte*> (&imageData_arg[y * row_size]);

      png_read_image (png_ptr, row_pointers);

      png_destroy_read_struct (&png_ptr, &info_ptr, nullptr);
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

