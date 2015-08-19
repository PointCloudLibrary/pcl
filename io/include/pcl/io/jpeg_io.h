/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014 Centrum Wiskunde Informatica.
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
#ifndef JPEG_IO_H
#define JPEG_IO_H

#include <pcl/PCLImage.h>

/**!
\brief jpeg reading and writing for PCL based on jpeg turbo
\author Rufael Mekuria rufael.mekuria@cwi.nl
*/

namespace pcl{
  
  namespace io{
  
    /** \brief Class for writing jpeg buffers and files */
    class PCL_EXPORTS JPEGWriter
    {
      public:
        /** \brief function for writing an image as a compressed JPEG buffer
        *  \param[in] im_in PCLImage input image
        *  \param[out] jpeg_out  vector<uint8_t> containing compressed jpeg data
        *  \param[in] quality quality value of jpeg to use (100 is best)
        *  \return  returns true when succesfull, false otherwise
        */
        static bool
        writeJPEG(const PCLImage &im_in, std::vector<uint8_t> &cdat, int quality = 75);

        /** \brief function for writing an image as a compressed JPEG file
        *  \param[in] im_in PCLImage input image
        *  \param[out] jpeg_out string  containing compressed jpeg filename
        *  \param[in] quality quality value of jpeg to use (100 is best)
        *  \return  returns true when succesfull, false otherwise
        */
        static bool
        writeJPEG(const PCLImage &im_in, const std::string &file_name, int quality = 75);
    
      private:
        /** \brief generic function compressed JPEG buffer/file write
        *  \param[in] im_in PCLImage input image
        *  \param[out] cdat  vector<uint8_t>
        *  \param[in] file_name  string file to write to the output
        *  \param[in] quality quality value of jpeg to use (100 is best)
        *  \param[in] bool write file
        *  \return  returns true when succesfull
        */
        static bool
        writeJPEG(const PCLImage &im_in, std::vector<uint8_t> &cdat, const std::string &file_name, int quality, bool write_file);
    };

    /** \brief Class for reading jpeg buffers and files */
    class PCL_EXPORTS JPEGReader
    {
      public:
        /** \brief function for reading a compressed JPEG buffer as a PCL image
        *  \param[in] jpeg_in_dat  vector<uint8_t> containing compressed jpeg
        *  \param[out] im_out PCLImage output image
        *  \return  returns true when succesfull, false otherwise
        */
        static bool
        readJPEG(const std::vector<uint8_t> &jpeg_in_dat, PCLImage &im_out);

        /** \brief function for reading a compressed JPEG file as a PCL image
        *   \param[in] jpeg_in_file  string filename with compressed jpeg image
        *   \param[out] im_out PCLImage output image
        *   \return  returns true when succesfull, false otherwise
        */
        static bool
        readJPEG(const std::string &jpeg_in_file, PCLImage &im_out);

      private:
        /** \brief helper function for reading a compressed JPEG buffer/file as a PCL image
        *   \param[in] jpeg_in_dat   character membuffer  with compressed jpeg image
        *   \param[in] jpeg_in_file  string filename with compressed jpeg image
        *   \param[in] read_file bool  read file or from memmory buffer instead
        *   \param[out] im_out PCLImage output image
        *   \return  returns true when succesfull, false otherwise
        */
        static bool
        readJPEG(const std::vector<uint8_t> &jpeg_in_dat, const std::string &jpeg_in_file, bool read_file, PCLImage &im_out);
    };
  }
}
#endif // JPEG_IO_H
