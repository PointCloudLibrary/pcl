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
#include <pcl/console/time.h>
#include <pcl/io/lzf_image_io.h>
#include <pcl/io/lzf.h>
#include <pcl/console/print.h>
#include <fcntl.h>
#include <string.h>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
#endif

#define LZF_HEADER_SIZE 37

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFImageWriter::saveImageBlob (const char* data, 
                                        size_t data_size, 
                                        const std::string &filename)
{
#ifdef _WIN32
  HANDLE h_native_file = CreateFile (filename.c_str (), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
  if (h_native_file == INVALID_HANDLE_VALUE)
    return (false);
  HANDLE fm = CreateFileMapping (h_native_file, NULL, PAGE_READWRITE, 0, data_size, NULL);
  char *map = static_cast<char*> (MapViewOfFile (fm, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, data_size));
  CloseHandle (fm);
  memcpy (&map[0], data, data_size);
  UnmapViewOfFile (map);
  CloseHandle (h_native_file);
#else
  int fd = pcl_open (filename.c_str (), O_RDWR | O_CREAT | O_TRUNC, static_cast<mode_t> (0600));
  if (fd < 0)
    return (false);
  // Stretch the file size to the size of the data
  off_t result = pcl_lseek (fd, data_size - 1, SEEK_SET);
  if (result < 0)
  {
    pcl_close (fd);
    return (false);
  }
  // Write a bogus entry so that the new file size comes in effect
  result = static_cast<int> (::write (fd, "", 1));
  if (result != 1)
  {
    pcl_close (fd);
    return (false);
  }

  char *map = static_cast<char*> (mmap (0, data_size, PROT_WRITE, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    return (false);
  }

  // Copy the data
  memcpy (&map[0], data, data_size);

  if (munmap (map, (data_size)) == -1)
  {
    pcl_close (fd);
    return (false);
  }
  pcl_close (fd);
#endif
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
pcl::uint32_t
pcl::io::LZFImageWriter::compress (const char* input, 
                                   uint32_t uncompressed_size, 
                                   uint32_t width,
                                   uint32_t height,
                                   const std::string &image_type,
                                   char *output)
{
  static const int header_size = LZF_HEADER_SIZE;
  float finput_size = static_cast<float> (uncompressed_size);
  unsigned int compressed_size = pcl::lzfCompress (input,
                                                   uncompressed_size,
                                                   &output[header_size],
                                                   uint32_t (finput_size * 1.5f));

  uint32_t compressed_final_size = 0;
  if (compressed_size)
  {
    // Copy the header first
    const char header[] = "PCLZF";
    memcpy (&output[0],  &header[0], 5);
    memcpy (&output[5],  &width, sizeof (uint32_t));
    memcpy (&output[9],  &height, sizeof (uint32_t));
    std::string itype = image_type;
    // Cut or pad the string
    if (itype.size () > 16)
    {
      PCL_WARN ("[pcl::io::LZFImageWriter::compress] Image type should be a string of maximum 16 characters! Cutting %s to %s.\n", image_type.c_str (), image_type.substr (0, 15).c_str ());
      itype = itype.substr (0, 15);
    }
    if (itype.size () < 16)
      itype.insert (itype.end (), 16 - itype.size (), ' ');

    memcpy (&output[13], &itype[0], 16);
    memcpy (&output[29], &compressed_size, sizeof (uint32_t));
    memcpy (&output[33], &uncompressed_size, sizeof (uint32_t));
    compressed_final_size = uint32_t (compressed_size + header_size);
  }

  return (compressed_final_size);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFDepth16ImageWriter::write (const char* data,
                                       uint32_t width, uint32_t height,
                                       const std::string &filename)
{
  // Prepare the compressed depth buffer
  unsigned int depth_size = width * height * 2;
  char* compressed_depth = static_cast<char*> (malloc (size_t (float (depth_size) * 1.5f + float (LZF_HEADER_SIZE))));

  size_t compressed_size = compress (data,
                                     depth_size,
                                     width, height,
                                     "depth16",
                                     compressed_depth);
  if (compressed_size == 0)
  {
    free (compressed_depth);
    return (false);
  }

  // Save the actual image
  saveImageBlob (compressed_depth, compressed_size, filename);
  free (compressed_depth);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFImageWriter::writeParameter (const double &parameter,
                                         const std::string &tag,
                                         const std::string &filename)
{
  boost::property_tree::ptree pt;
  try
  {
    boost::property_tree::xml_parser::read_xml (filename, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (std::exception& e)
  {}

  boost::property_tree::xml_writer_settings<char> settings ('\t', 1);
  pt.put (tag, parameter);
  write_xml (filename, pt, std::locale (), settings);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFDepth16ImageWriter::writeParameters (const pcl::io::CameraParameters &parameters,
                                                 const std::string &filename)
{
  boost::property_tree::ptree pt;
  try
  {
    boost::property_tree::xml_parser::read_xml (filename, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (std::exception& e)
  {}

  boost::property_tree::xml_writer_settings<char> settings ('\t', 1);
  pt.put ("depth.focal_length_x", parameters.focal_length_x);
  pt.put ("depth.focal_length_y", parameters.focal_length_y);
  pt.put ("depth.principal_point_x", parameters.principal_point_x);
  pt.put ("depth.principal_point_y", parameters.principal_point_y);
  pt.put ("depth.z_multiplication_factor", z_multiplication_factor_);
  write_xml (filename, pt, std::locale (), settings);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFRGB24ImageWriter::write (const char *data, 
                                     uint32_t width, uint32_t height,
                                     const std::string &filename)
{
  // Transform RGBRGB into RRGGBB for better compression
  std::vector<char> rrggbb (width * height * 3);
  int ptr1 = 0,
      ptr2 = width * height,
      ptr3 = 2 * width * height;
  for (uint32_t i = 0; i < width * height; ++i, ++ptr1, ++ptr2, ++ptr3)
  {
    rrggbb[ptr1] = data[i * 3 + 0];
    rrggbb[ptr2] = data[i * 3 + 1];
    rrggbb[ptr3] = data[i * 3 + 2];
  }

  char* compressed_rgb = static_cast<char*> (malloc (size_t (float (rrggbb.size ()) * 1.5f + float (LZF_HEADER_SIZE))));
  size_t compressed_size = compress (reinterpret_cast<const char*> (&rrggbb[0]), 
                                     uint32_t (rrggbb.size ()),
                                     width, height,
                                     "rgb24",
                                     compressed_rgb);

  if (compressed_size == 0)
  {
    free (compressed_rgb);
    return (false);
  }

  // Save the actual image
  saveImageBlob (compressed_rgb, compressed_size, filename);
  free (compressed_rgb);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFRGB24ImageWriter::writeParameters (const pcl::io::CameraParameters &parameters,
                                              const std::string &filename)
{
  boost::property_tree::ptree pt;
  try
  {
    boost::property_tree::xml_parser::read_xml (filename, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (std::exception& e)
  {}

  boost::property_tree::xml_writer_settings<char> settings ('\t', 1);
  pt.put ("rgb.focal_length_x", parameters.focal_length_x);
  pt.put ("rgb.focal_length_y", parameters.focal_length_y);
  pt.put ("rgb.principal_point_x", parameters.principal_point_x);
  pt.put ("rgb.principal_point_y", parameters.principal_point_y);
  write_xml (filename, pt, std::locale (), settings);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFYUV422ImageWriter::write (const char *data, 
                                      uint32_t width, uint32_t height,
                                      const std::string &filename)
{
  // Transform YUV422 into UUUYYYYYYVVV for better compression
  std::vector<char> uuyyvv (width * height * 2);
  int wh2 = width * height / 2,
      ptr1 = 0,                        // u
      ptr2 = wh2,                      // y
      ptr3 = wh2 + width * height;     // v
  for (int i = 0; i < wh2; ++i, ++ptr1, ptr2 += 2, ++ptr3)
  {
    uuyyvv[ptr1] = data[i * 4 + 0];       // u
    uuyyvv[ptr2 + 0] = data[i * 4 + 1];   // y
    uuyyvv[ptr2 + 1] = data[i * 4 + 3];   // y
    uuyyvv[ptr3] = data[i * 4 + 2];       // v
  }

  char* compressed_yuv = static_cast<char*> (malloc (size_t (float (uuyyvv.size ()) * 1.5f + float (LZF_HEADER_SIZE))));
  size_t compressed_size = compress (reinterpret_cast<const char*> (&uuyyvv[0]), 
                                     uint32_t (uuyyvv.size ()),
                                     width, height,
                                     "yuv422",
                                     compressed_yuv);

  if (compressed_size == 0)
  {
    free (compressed_yuv);
    return (false);
  }

  // Save the actual image
  saveImageBlob (compressed_yuv, compressed_size, filename);
  free (compressed_yuv);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFBayer8ImageWriter::write (const char *data, 
                                      uint32_t width, uint32_t height,
                                      const std::string &filename)
{
  unsigned int bayer_size = width * height;
  char* compressed_bayer = static_cast<char*> (malloc (size_t (float (bayer_size) * 1.5f + float (LZF_HEADER_SIZE))));
  size_t compressed_size = compress (data,
                                     bayer_size,
                                     width, height,
                                     "bayer8",
                                     compressed_bayer);

  if (compressed_size == 0)
  {
    free (compressed_bayer);
    return (false);
  }

  // Save the actual image
  saveImageBlob (compressed_bayer, compressed_size, filename);
  free (compressed_bayer);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
pcl::io::LZFImageReader::LZFImageReader ()
  : width_ ()
  , height_ ()
  , image_type_identifier_ ()
  , parameters_ ()
{
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFImageReader::loadImageBlob (const std::string &filename,
                                        std::vector<char> &data,
                                        uint32_t &uncompressed_size)
{
  if (filename == "" || !boost::filesystem::exists (filename))
  {
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Could not find file '%s'.\n", filename.c_str ());
    return (false);
  }
  // Open for reading
  int fd = pcl_open (filename.c_str (), O_RDONLY);
  if (fd == -1)
  {
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Failure to open file %s\n", filename.c_str () );
    return (false);
  }

  // Seek to the end of file to get the filesize
  off_t data_size = pcl_lseek (fd, 0, SEEK_END);
  if (data_size < 0)
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] lseek errno: %d strerror: %s\n", errno, strerror (errno));
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Error during lseek ()!\n");
    return (false);
  }
  pcl_lseek (fd, 0, SEEK_SET);

#ifdef _WIN32
  // As we don't know the real size of data (compressed or not), 
  // we set dwMaximumSizeHigh = dwMaximumSizeLow = 0 so as to map the whole file
  HANDLE fm = CreateFileMapping ((HANDLE) _get_osfhandle (fd), NULL, PAGE_READONLY, 0, 0, NULL);
  // As we don't know the real size of data (compressed or not), 
  // we set dwNumberOfBytesToMap = 0 so as to map the whole file
  char *map = static_cast<char*>(MapViewOfFile (fm, FILE_MAP_READ, 0, 0, 0));
  if (map == NULL)
  {
    CloseHandle (fm);
    pcl_close (fd);
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Error mapping view of file, %s\n", filename.c_str ());
    return (false);
  }
#else
  char *map = static_cast<char*> (mmap (0, data_size, PROT_READ, MAP_SHARED, fd, 0));
  if (map == reinterpret_cast<char*> (-1))    // MAP_FAILED
  {
    pcl_close (fd);
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Error preparing mmap for PCLZF file.\n");
    return (false);
  }
#endif

  // Check the header identifier here
  char header_string[5];
  memcpy (&header_string,    &map[0], 5);        // PCLZF
  if (std::string (header_string).substr (0, 5) != "PCLZF")
  {
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Wrong signature header! Should be 'P'C'L'Z'F'.\n");
#ifdef _WIN32
  UnmapViewOfFile (map);
  CloseHandle (fm);
#else
    munmap (map, data_size);
#endif
    return (false);
  }
  memcpy (&width_,            &map[5], sizeof (uint32_t));
  memcpy (&height_,           &map[9], sizeof (uint32_t));
  char imgtype_string[16];
  memcpy (&imgtype_string,    &map[13], 16);       // BAYER8, RGB24_, YUV422_, ...
  image_type_identifier_ = std::string (imgtype_string).substr (0, 15);
  image_type_identifier_.insert (image_type_identifier_.end (), 1, '\0');

  static const int header_size = LZF_HEADER_SIZE;
  uint32_t compressed_size;
  memcpy (&compressed_size,   &map[29], sizeof (uint32_t));

  if (compressed_size + header_size != data_size)
  {
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Number of bytes to decompress written in file (%u) differs from what it should be (%u)!\n", compressed_size, data_size - header_size);
#ifdef _WIN32
  UnmapViewOfFile (map);
  CloseHandle (fm);
#else
    munmap (map, data_size);
#endif
    return (false);
  }

  memcpy (&uncompressed_size, &map[33], sizeof (uint32_t));

  data.resize (compressed_size);
  memcpy (&data[0], &map[header_size], compressed_size);
 
#ifdef _WIN32
  UnmapViewOfFile (map);
  CloseHandle (fm);
#else
  if (munmap (map, data_size) == -1)
    PCL_ERROR ("[pcl::io::LZFImageReader::loadImage] Munmap failure\n");
#endif
  pcl_close (fd);

  data_size = off_t (compressed_size);      // We only care about this from here on
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFImageReader::decompress (const std::vector<char> &input,
                                     std::vector<char> &output)
{
  if (output.empty ())
  {
    PCL_ERROR ("[pcl::io::LZFImageReader::decompress] Output array needs to be preallocated! The correct uncompressed array value should have been stored during the compression.\n");
    return (false);
  }
  unsigned int tmp_size = pcl::lzfDecompress (static_cast<const char*>(&input[0]), 
                                              uint32_t (input.size ()), 
                                              static_cast<char*>(&output[0]), 
                                              uint32_t (output.size ()));

  if (tmp_size != output.size ())
  {
    PCL_WARN ("[pcl::io::LZFImageReader::decompress] Size of decompressed lzf data (%u) does not match the uncompressed size value (%u). Errno: %d\n", tmp_size, output.size (), errno);
    return (false);
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFImageReader::readParameters (const std::string &filename)
{
  std::filebuf fb;
  std::filebuf *f = fb.open (filename.c_str (), std::ios::in);
  if (f == NULL)
    return (false);
  std::istream is (&fb);
  bool res = readParameters (is);
  fb.close ();
  return (res);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFRGB24ImageReader::readParameters (std::istream& is)
{
  boost::property_tree::ptree pt;
  read_xml (is, pt, boost::property_tree::xml_parser::trim_whitespace);

  boost::optional<boost::property_tree::ptree&> tree = pt.get_child_optional ("rgb");
  if (!tree)
    return (false);

  parameters_.focal_length_x = tree.get ().get<double>("focal_length_x");
  parameters_.focal_length_y = tree.get ().get<double>("focal_length_y");
  parameters_.principal_point_x = tree.get ().get<double>("principal_point_x");
  parameters_.principal_point_y = tree.get ().get<double>("principal_point_y");
  PCL_DEBUG ("[pcl::io::LZFRGB24ImageReader::readParameters] Read camera parameters (fx,fy,cx,cy): %g,%g,%g,%g.\n", 
      parameters_.focal_length_x, parameters_.focal_length_y, 
      parameters_.principal_point_x, parameters_.principal_point_y);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////
bool
pcl::io::LZFDepth16ImageReader::readParameters (std::istream& is)
{
  boost::property_tree::ptree pt;
  read_xml (is, pt, boost::property_tree::xml_parser::trim_whitespace);

  boost::optional<boost::property_tree::ptree&> tree = pt.get_child_optional ("depth");
  if (!tree)
    return (false);

  parameters_.focal_length_x = tree.get ().get<double>("focal_length_x");
  parameters_.focal_length_y = tree.get ().get<double>("focal_length_y");
  parameters_.principal_point_x = tree.get ().get<double>("principal_point_x");
  parameters_.principal_point_y = tree.get ().get<double>("principal_point_y");
  z_multiplication_factor_ = tree.get ().get<double>("z_multiplication_factor");
  PCL_DEBUG ("[pcl::io::LZFDepth16ImageReader::readParameters] Read camera parameters (fx,fy,cx,cy): %g,%g,%g,%g.\n", 
      parameters_.focal_length_x, parameters_.focal_length_y, 
      parameters_.principal_point_x, parameters_.principal_point_y);
  PCL_DEBUG ("[pcl::io::LZFDepth16ImageReader::readParameters] Multiplication factor: %g.\n", z_multiplication_factor_);
  return (true);
}

