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

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <vector>

namespace pcl
{
  namespace io
  {
    /** \brief Basic camera parameters placeholder. */
    struct CameraParameters
    {
      /** fx */
      double focal_length_x;
      /** fy */
      double focal_length_y;
      /** cx */
      double principal_point_x;
      /** cy */
      double principal_point_y;
    };

    /** \brief PCL-LZF image format reader.
      * The PCL-LZF image format is nothing else but a LZF-modified compression over
      * an existing file type (e.g., PNG). However, in certain situations, like RGB data for 
      * example, an [RGBRGB...RGB] array will be first reordered into [RR...RGG...GBB...B]
      * in order to ensure better compression.
      *
      * The current list of compressors/decompressors include:
      *  * LZF compressed 24-bit [RR...RGG...GBB...B] data
      *  * LZF compressed 8-bit Bayer data
      *  * LZF compressed 16-bit YUV422 data
      *  * LZF compressed 16-bit depth data
      *
      * Please note that files found using the above mentioned extensions will be treated
      * as such. Inherit from this class and overwrite the I/O methods if you plan to change 
      * this behavior.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFImageReader
    {
      public:
        /** Empty constructor */
        LZFImageReader ();
        /** Empty destructor */
        virtual ~LZFImageReader () = default;

        /** \brief Read camera parameters from a given file and store them internally.
          * \return true if operation successful, false otherwise
          */ 
        bool
        readParameters (const std::string &filename);

        /** \brief Read the parameters from a struct instead
         *  \param[in] parameters Camera parameters to use */
        inline void
        setParameters (const CameraParameters &parameters)
        {
          parameters_ = parameters;
        }

        /** \brief Get the camera parameters currently being used
         *  returns a CameraParameters struct */
        inline CameraParameters
        getParameters () const
        {
          return parameters_;
        }

        /** \brief Get the image width as read from disk. */
        inline std::uint32_t
        getWidth () const
        {
          return (width_);
        }

        /** \brief Get the image height as read from disk. */
        inline std::uint32_t
        getHeight () const
        {
          return (height_);
        }

        /** \brief Get the type of the image read from disk. */
        inline std::string
        getImageType () const
        {
          return (image_type_identifier_);
        }

      protected:
        /** \brief Read camera parameters from a given stream and store them internally.
          * \return true if operation successful, false otherwise
          */ 
        virtual bool
        readParameters (std::istream&) { return (false); }

        /** \brief Load a compressed image array from disk
          * \param[in] filename the file name to load the data from
          * \param[out] data the size of the data
          * \param uncompressed_size
          * \return an array filled with the data loaded from disk, NULL if error
          */
        bool
        loadImageBlob (const std::string &filename,
                       std::vector<char> &data,
                       std::uint32_t &uncompressed_size);

        /** \brief Realtime LZF decompression.
          * \param[in] input the array to decompress
          * \param[out] output the decompressed array
          * \return true if operation successful, false otherwise
          */
        bool
        decompress (const std::vector<char> &input, 
                    std::vector<char> &output); 

        /** \brief The image width, as read from the file. */
        std::uint32_t width_;

        /** \brief The image height, as read from the file. */
        std::uint32_t height_;

        /** \brief The image type string, as read from the file. */
        std::string image_type_identifier_;

        /** \brief Internal set of camera parameters. */
        CameraParameters parameters_;
    };

    /** \brief PCL-LZF 16-bit depth image format reader.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFDepth16ImageReader : public LZFImageReader
    {
      public:
        using LZFImageReader::readParameters;

        /** Empty constructor */
        LZFDepth16ImageReader () 
          : z_multiplication_factor_ (0.001)      // Set default multiplication factor
        {}

        /** Empty destructor */
        ~LZFDepth16ImageReader () override = default;

        /** \brief Read the data stored in a PCLZF depth file and convert it to a pcl::PointCloud type.
          * \param[in] filename the file name to read the data from
          * \param[out] cloud the resultant output point cloud
          */
        template <typename PointT> bool
        read (const std::string &filename, pcl::PointCloud<PointT> &cloud);
        
        /** \brief Read the data stored in a PCLZF depth file and convert it to a pcl::PointCloud type.
          * \param[in] filename the file name to read the data from
          * \param[in] num_threads The number of threads to use. 0 indicates OpenMP is free to choose.
          * \param[out] cloud the resultant output point cloud
          */
        template <typename PointT> bool
        readOMP (const std::string &filename, pcl::PointCloud<PointT> &cloud, 
                 unsigned int num_threads=0);

        /** \brief Read camera parameters from a given stream and store them internally.
          * The parameters will be read from the \<depth\> ... \</depth\> tag.
          * \return true if operation successful, false otherwise
          */ 
        bool
        readParameters (std::istream& is) override;

      protected:
        /** \brief Z-value depth multiplication factor 
          * (i.e., if raw data is in [mm] and we want [m], we need to multiply with 0.001)
          */
        double z_multiplication_factor_;
    };

    /** \brief PCL-LZF 24-bit RGB image format reader.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFRGB24ImageReader : public LZFImageReader
    {
      public:
        using LZFImageReader::readParameters;

        /** Empty constructor */
        LZFRGB24ImageReader () = default;
        /** Empty destructor */
        ~LZFRGB24ImageReader () override = default;

        /** \brief Read the data stored in a PCLZF RGB file and convert it to a pcl::PointCloud type.
          * \param[in] filename the file name to read the data from
          * \param[out] cloud the resultant output point cloud
          */
        template<typename PointT> bool
        read (const std::string &filename, pcl::PointCloud<PointT> &cloud);
        
        /** \brief Read the data stored in a PCLZF RGB file and convert it to a pcl::PointCloud type.
          * Note that, unless massively multithreaded, this will likely not result in a significant speedup and may even slow performance.
          * \param[in] filename the file name to read the data from
          * \param[in] num_threads The number of threads to use
          * \param[out] cloud the resultant output point cloud
          */
        template <typename PointT> bool
        readOMP (const std::string &filename, pcl::PointCloud<PointT> &cloud, 
                 unsigned int num_threads=0);

        /** \brief Read camera parameters from a given stream and store them internally.
          * The parameters will be read from the \<rgb\> ... \</rgb\> tag.
          * \return true if operation successful, false otherwise
          */ 
        bool
        readParameters (std::istream& is) override;

      protected:
    };

    /** \brief PCL-LZF 8-bit Bayer image format reader.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFYUV422ImageReader : public LZFRGB24ImageReader
    {
      public:
        using LZFRGB24ImageReader::readParameters;

        /** Empty constructor */
        LZFYUV422ImageReader () = default;
        /** Empty destructor */
        ~LZFYUV422ImageReader () override = default;

        /** \brief Read the data stored in a PCLZF YUV422 16bit file and convert it to a pcl::PointCloud type.
          * \param[in] filename the file name to read the data from
          * \param[out] cloud the resultant output point cloud
          */
        template<typename PointT> bool
        read (const std::string &filename, pcl::PointCloud<PointT> &cloud);
        
        /** \brief Read the data stored in a PCLZF YUV422 file and convert it to a pcl::PointCloud type.
          * Note that, unless massively multithreaded, this will likely not result in a significant speedup
          * \param[in] filename the file name to read the data from
          * \param[in] num_threads The number of threads to use
          * \param[out] cloud the resultant output point cloud
          */
        template <typename PointT> bool
        readOMP (const std::string &filename, pcl::PointCloud<PointT> &cloud, 
                 unsigned int num_threads=0);
    };

    /** \brief PCL-LZF 8-bit Bayer image format reader.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFBayer8ImageReader : public LZFRGB24ImageReader
    {
      public:
        using LZFRGB24ImageReader::readParameters;

        /** Empty constructor */
        LZFBayer8ImageReader () = default;
        /** Empty destructor */
        ~LZFBayer8ImageReader () override = default;

        /** \brief Read the data stored in a PCLZF Bayer 8bit file and convert it to a pcl::PointCloud type.
          * \param[in] filename the file name to read the data from
          * \param[out] cloud the resultant output point cloud
          */
        template<typename PointT> bool
        read (const std::string &filename, pcl::PointCloud<PointT> &cloud);

        /** \brief Read the data stored in a PCLZF Bayer 8bit file and convert it to a pcl::PointCloud type.
          * Note that, unless massively multithreaded, this will likely not result in a significant speedup and may even slow performance.
          * \param[in] filename the file name to read the data from
          * \param[in] num_threads The number of threads to use
          * \param[out] cloud the resultant output point cloud
          */
        template <typename PointT> bool
        readOMP (const std::string &filename, pcl::PointCloud<PointT> &cloud, 
                 unsigned int num_threads=0);
    };

    /** \brief PCL-LZF image format writer.
      * The PCL-LZF image format is nothing else but a LZF-modified compression over
      * an existing file type (e.g., PNG). However, in certain situations, like RGB data for 
      * example, an [RGBRGB...RGB] array will be first reordered into [RR...RGG...GBB...B]
      * in order to ensure better compression.
      *
      * The current list of compressors/decompressors include:
      *  * LZF compressed 24-bit [RR...RGG...GBB...B] data
      *  * LZF compressed 8-bit Bayer data
      *  * LZF compressed 16-bit YUV422 data
      *  * LZF compressed 16-bit depth data
      *
      * Please note that files found using the above mentioned extensions will be treated
      * as such. Inherit from this class and overwrite the I/O methods if you plan to change 
      * this behavior.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFImageWriter
    {
      public:
        /** Empty constructor */
        LZFImageWriter () = default;
        /** Empty destructor */
        virtual ~LZFImageWriter () = default;

        /** \brief Save an image into PCL-LZF format. Virtual.
          * \param[in] data the array holding the image
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] filename the file name to write
          * \return true if operation successful, false otherwise
          */
        virtual bool
        write (const char* data,
               std::uint32_t width, std::uint32_t height,
               const std::string &filename) = 0;

        /** \brief Write camera parameters to disk. Virtual.
          * \param[in] parameters the camera parameters
          * \param[in] filename the file name to write
          * \return true if operation successful, false otherwise
          */ 
        virtual bool
        writeParameters (const CameraParameters &parameters,
                         const std::string &filename) = 0;

        /** \brief Save an image and its camera parameters into PCL-LZF format.
          * \param[in] data the array holding the image
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] parameters the camera parameters
          * \param[in] filename_data the file name to write the data to
          * \param[in] filename_xml the file name to write the parameters to
          * \return true if operation successful, false otherwise
          */
        virtual bool
        write (const char* data,
               std::uint32_t width, std::uint32_t height,
               const CameraParameters &parameters,
               const std::string &filename_data,
               const std::string &filename_xml)
        {
          bool res1 = write (data, width, height, filename_data);
          bool res2 = writeParameters (parameters, filename_xml);
          return (res1 && res2);
        }

        /** \brief Write a single image/camera parameter to file, given an XML tag
          * \param[in] parameter the value of the parameter to write
          * \param[in] tag the value of the XML tag
          * \param[in] filename the file name to write
          * \return true if operation successful, false otherwise
          * Example:
          * \code
          * pcl::io::LZFDepthImageWriter w;
          * w.writeParameter (0.001, "depth.multiplication_factor", "parameters.xml");
          * \endcode
          */
        bool
        writeParameter (const double &parameter, const std::string &tag, 
                        const std::string &filename);
      protected:
        /** \brief Save a compressed image array to disk
          * \param[in] data the data to save
          * \param[in] data_size the size of the data
          * \param[in] filename the file name to write the data to
          * \return true if operation successful, false otherwise
          */
        bool
        saveImageBlob (const char* data, std::size_t data_size, 
                       const std::string &filename);

        /** \brief Realtime LZF compression.
          * \param[in] input the array to compress
          * \param[in] input_size the size of the array to compress
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] image_type the type of the image to save. This should be an up to 
          * 16 characters string describing the data type. Examples are: "bayer8", "rgb24",
          * "yuv422", "depth16".
          * \param[out] output the compressed output array (must be pre-allocated!)
          * \return the number of bytes in the output array
          */
        std::uint32_t
        compress (const char* input, std::uint32_t input_size, 
                  std::uint32_t width, std::uint32_t height,
                  const std::string &image_type,
                  char *output);
    };

    /** \brief PCL-LZF 16-bit depth image format writer.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFDepth16ImageWriter : public LZFImageWriter
    {
      public:
        /** Empty constructor */
        LZFDepth16ImageWriter () 
          : z_multiplication_factor_ (0.001)      // Set default multiplication factor
        {}

        /** Empty destructor */
        ~LZFDepth16ImageWriter () override = default;

        /** \brief Save a 16-bit depth image into PCL-LZF format.
          * \param[in] data the array holding the depth image
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] filename the file name to write (preferred extension: .pclzf)
          * \return true if operation successful, false otherwise
          */
        bool
        write (const char* data,
               std::uint32_t width, std::uint32_t height,
               const std::string &filename) override;

        /** \brief Write camera parameters to disk.
          * \param[in] parameters the camera parameters
          * \param[in] filename the file name to write
          * \return true if operation successful, false otherwise
          * This overwrites the following parameters in the xml file, under the 
          * \<depth> tag:
          *   \<focal_length_x\>...\</focal_length_x\>
          *   \<focal_length_y\>...\</focal_length_y\>
          *   \<principal_point_x\>...\</principal_point_x\>
          *   \<principal_point_y\>...\</principal_point_y\>
          *   \<z_multiplication_factor\>...\</z_multiplication_factor\>
          */ 
        bool
        writeParameters (const CameraParameters &parameters,
                         const std::string &filename) override;

      protected:
        /** \brief Z-value depth multiplication factor 
          * (i.e., if raw data is in [mm] and we want [m], we need to multiply with 0.001)
          */
        double z_multiplication_factor_;
    };

    /** \brief PCL-LZF 24-bit RGB image format writer.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFRGB24ImageWriter : public LZFImageWriter
    {
      public:
        /** Empty constructor */
        LZFRGB24ImageWriter () = default;
        /** Empty destructor */
        ~LZFRGB24ImageWriter () override = default;

        /** \brief Save a 24-bit RGB image into PCL-LZF format.
          * \param[in] data the array holding the RGB image (as [RGB..RGB] or [BGR..BGR])
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] filename the file name to write (preferred extension: .pclzf)
          * \return true if operation successful, false otherwise
          */
        bool
        write (const char *data, 
               std::uint32_t width, std::uint32_t height,
               const std::string &filename) override;

        /** \brief Write camera parameters to disk.
          * \param[in] parameters the camera parameters
          * \param[in] filename the file name to write
          * \return true if operation successful, false otherwise
          */ 
        bool
        writeParameters (const CameraParameters &parameters,
                         const std::string &filename) override;

      protected:
    };

    /** \brief PCL-LZF 16-bit YUV422 image format writer.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFYUV422ImageWriter : public LZFRGB24ImageWriter
    {
      public:
        /** Empty constructor */
        LZFYUV422ImageWriter () = default;
        /** Empty destructor */
        ~LZFYUV422ImageWriter () override = default;

        /** \brief Save a 16-bit YUV422 image into PCL-LZF format.
          * \param[in] data the array holding the YUV422 image (as [YUYV...YUYV])
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] filename the file name to write (preferred extension: .pclzf)
          * \return true if operation successful, false otherwise
          */
        bool
        write (const char *data, 
               std::uint32_t width, std::uint32_t height,
               const std::string &filename) override;
    };

    /** \brief PCL-LZF 8-bit Bayer image format writer.
      *
      * The main advantage of using the PCL-LZF image I/O routines is a very good file size 
      * versus I/O speed ratio. Tests performed using LZF, Snappy, ZIP, GZ2, BZIP2, as well 
      * as PNG, JPEG, and TIFF compression have shown that the internal PCL LZF methods 
      * provide the best score for the types of applications PCL is suited for.
      *
      * \author Radu B. Rusu
      * \ingroup io
      */
    class PCL_EXPORTS LZFBayer8ImageWriter : public LZFRGB24ImageWriter
    {
      public:
        /** Empty constructor */
        LZFBayer8ImageWriter () = default;
        /** Empty destructor */
        ~LZFBayer8ImageWriter () override = default;

        /** \brief Save a 8-bit Bayer image into PCL-LZF format.
          * \param[in] data the array holding the 8-bit Bayer array
          * \param[in] width the with of the data array
          * \param[in] height the height of the data array
          * \param[in] filename the file name to write (preferred extension: .pclzf)
          * \return true if operation successful, false otherwise
          */
        bool
        write (const char *data, 
               std::uint32_t width, std::uint32_t height,
               const std::string &filename) override;
    };
  }
}

#include <pcl/io/impl/lzf_image_io.hpp>
