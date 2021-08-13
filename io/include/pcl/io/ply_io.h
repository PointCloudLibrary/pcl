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
 * $Id$
 *
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/PolygonMesh.h>

#include <sstream>
#include <tuple>

namespace pcl
{
  /** \brief Point Cloud Data (PLY) file format reader.
    *
    * The PLY data format is organized in the following way:
    * lines beginning with "comment" are treated as comments
    *   - ply
    *   - format [ascii|binary_little_endian|binary_big_endian] 1.0
    *   - element vertex COUNT
    *   - property float x 
    *   - property float y 
    *   - [property float z] 
    *   - [property float normal_x] 
    *   - [property float normal_y] 
    *   - [property float normal_z] 
    *   - [property uchar red] 
    *   - [property uchar green] 
    *   - [property uchar blue] ...
    *   - ascii/binary point coordinates
    *   - [element camera 1]
    *   - [property float view_px] ...
    *   - [element range_grid COUNT]
    *   - [property list uchar int vertex_indices]
    *   - end header
    *
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS PLYReader : public FileReader
  {
    public:
      enum
      {
        PLY_V0 = 0,
        PLY_V1 = 1
      };
      
      PLYReader ()
        : origin_ (Eigen::Vector4f::Zero ())
        , orientation_ (Eigen::Matrix3f::Zero ())
        , cloud_ ()
        , vertex_count_ (0)
        , vertex_offset_before_ (0)
        , range_grid_ (nullptr)
        , rgb_offset_before_ (0)
        , do_resize_ (false)
        , polygons_ (nullptr)
        , r_(0), g_(0), b_(0)
        , a_(0), rgba_(0)
      {}

      PLYReader (const PLYReader &p)
        : origin_ (Eigen::Vector4f::Zero ())
        , orientation_ (Eigen::Matrix3f::Zero ())
        , cloud_ ()
        , vertex_count_ (0)
        , vertex_offset_before_ (0)
        , range_grid_ (nullptr)
        , rgb_offset_before_ (0)
        , do_resize_ (false)
        , polygons_ (nullptr)
        , r_(0), g_(0), b_(0)
        , a_(0), rgba_(0)
      {
        *this = p;
      }

      PLYReader&
      operator = (const PLYReader &p)
      {
        origin_ = p.origin_;
        orientation_ = p.orientation_;
        range_grid_ = p.range_grid_;
        polygons_ = p.polygons_;
        return (*this);
      }

      ~PLYReader () { delete range_grid_; }
      /** \brief Read a point cloud data header from a PLY file.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given PLY file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * Returns:
        *  * < 0 (-1) on error
        *  * > 0 on success
        * \param[in] file_name the name of the file to load
        * \param[out] cloud the resultant point cloud dataset (only the header will be filled)
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[out] ply_version the PLY version read from the file
        * \param[out] data_type the type of PLY data stored in the file
        * \param[out] data_idx the data index
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      int 
      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                  int &ply_version, int &data_type, unsigned int &data_idx, const int offset = 0) override;

      /** \brief Read a point cloud data from a PLY file and store it into a pcl/PCLPointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[out] ply_version the PLY version read from the file
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      int 
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int& ply_version, const int offset = 0) override;

      /** \brief Read a point cloud data from a PLY file and store it into a pcl/PCLPointCloud2.
        * \note This function is provided for backwards compatibility only
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      inline int 
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0)
      {
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int ply_version;
        return read (file_name, cloud, origin, orientation, ply_version, offset);
      }

      /** \brief Read a point cloud data from any PLY file, and convert it to the given template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      template<typename PointT> inline int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud, const int offset = 0)
      {
        pcl::PCLPointCloud2 blob;
        int ply_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                        ply_version, offset);

        // Exit in case of error
        if (res < 0)
          return (res);
        pcl::fromPCLPointCloud2 (blob, cloud);
        return (0);
      }
      
      /** \brief Read a point cloud data from a PLY file and store it into a pcl/PolygonMesh.
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] mesh the resultant PolygonMesh message read from disk
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[out] ply_version the PLY version read from the file
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      int
      read (const std::string &file_name, pcl::PolygonMesh &mesh,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
            int& ply_version, const int offset = 0);

      /** \brief Read a point cloud data from a PLY file and store it into a pcl/PolygonMesh.
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] mesh the resultant PolygonMesh message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      int
      read (const std::string &file_name, pcl::PolygonMesh &mesh, const int offset = 0);

    private:
      ::pcl::io::ply::ply_parser parser_;

      bool
      parse (const std::string& istream_filename);

      /** \brief Info callback function
        * \param[in] filename PLY file read
        * \param[in] line_number line triggering the callback
        * \param[in] message information message
        */
      void 
      infoCallback (const std::string& filename, std::size_t line_number, const std::string& message)
      {
        PCL_DEBUG ("[pcl::PLYReader] %s:%lu: %s\n", filename.c_str (), line_number, message.c_str ());
      }
      
      /** \brief Warning callback function
        * \param[in] filename PLY file read
        * \param[in] line_number line triggering the callback
        * \param[in] message warning message
        */
      void 
      warningCallback (const std::string& filename, std::size_t line_number, const std::string& message)
      {
        PCL_WARN ("[pcl::PLYReader] %s:%lu: %s\n", filename.c_str (), line_number, message.c_str ());
      }
      
      /** \brief Error callback function
        * \param[in] filename PLY file read
        * \param[in] line_number line triggering the callback
        * \param[in] message error message
        */
      void 
      errorCallback (const std::string& filename, std::size_t line_number, const std::string& message)
      {
        PCL_ERROR ("[pcl::PLYReader] %s:%lu: %s\n", filename.c_str (), line_number, message.c_str ());
      }
      
      /** \brief function called when the keyword element is parsed
        * \param[in] element_name element name
        * \param[in] count number of instances
        */
      std::tuple<std::function<void ()>, std::function<void ()> > 
      elementDefinitionCallback (const std::string& element_name, std::size_t count);
      
      bool
      endHeaderCallback ();

      /** \brief function called when a scalar property is parsed
        * \param[in] element_name element name to which the property belongs
        * \param[in] property_name property name
        */
      template <typename ScalarType> std::function<void (ScalarType)> 
      scalarPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name);

      /** \brief function called when a list property is parsed
        * \param[in] element_name element name to which the property belongs
        * \param[in] property_name list property name
        */
      template <typename SizeType, typename ScalarType>
      std::tuple<std::function<void (SizeType)>, std::function<void (ScalarType)>, std::function<void ()> >
      listPropertyDefinitionCallback (const std::string& element_name, const std::string& property_name);
      
      /** \brief function called at the beginning of a list property parsing.
        * \param[in] size number of elements in the list
        */
      template <typename SizeType> void
      vertexListPropertyBeginCallback (const std::string& property_name, SizeType size);

      /** \brief function called when a list element is parsed.
        * \param[in] value the list's element value
        */
      template <typename ContentType> void
      vertexListPropertyContentCallback (ContentType value);

      /** \brief function called at the end of a list property parsing */
      inline void
      vertexListPropertyEndCallback ();

      /** Callback function for an anonymous vertex scalar property.
        * Writes down a double value in cloud data.
        * param[in] value double value parsed
        */
      template<typename Scalar> void
      vertexScalarPropertyCallback (Scalar value);

      /** Callback function for vertex RGB color.
        * This callback is in charge of packing red green and blue in a single int
        * before writing it down in cloud data.
        * param[in] color_name color name in {red, green, blue}
        * param[in] color value of {red, green, blue} property
        */
      inline void
      vertexColorCallback (const std::string& color_name, pcl::io::ply::uint8 color);

      /** Callback function for vertex intensity.
        * converts intensity from int to float before writing it down in cloud data.
        * param[in] intensity
        */
      inline void
      vertexIntensityCallback (pcl::io::ply::uint8 intensity);

      /** Callback function for vertex alpha.
        * extracts RGB value, append alpha and put it back
        * param[in] alpha
        */
      inline void
      vertexAlphaCallback (pcl::io::ply::uint8 alpha);
      
      /** Callback function for origin x component.
        * param[in] value origin x value
        */
      inline void
      originXCallback (const float& value) { origin_[0] = value; }
      
      /** Callback function for origin y component.
        * param[in] value origin y value
        */
      inline void
      originYCallback (const float& value) { origin_[1] = value; }

      /** Callback function for origin z component.
        * param[in] value origin z value
        */      
      inline void
      originZCallback (const float& value) { origin_[2] = value; }
    
      /** Callback function for orientation x axis x component.
        * param[in] value orientation x axis x value
        */
      inline void
      orientationXaxisXCallback (const float& value) { orientation_ (0,0) = value; }
      
      /** Callback function for orientation x axis y component.
        * param[in] value orientation x axis y value
        */
      inline void
      orientationXaxisYCallback (const float& value) { orientation_ (0,1) = value; }
      
      /** Callback function for orientation x axis z component.
        * param[in] value orientation x axis z value
        */
      inline void
      orientationXaxisZCallback (const float& value) { orientation_ (0,2) = value; }
      
      /** Callback function for orientation y axis x component.
        * param[in] value orientation y axis x value
        */
      inline void
      orientationYaxisXCallback (const float& value) { orientation_ (1,0) = value; }
      
      /** Callback function for orientation y axis y component.
        * param[in] value orientation y axis y value
        */
      inline void
      orientationYaxisYCallback (const float& value) { orientation_ (1,1) = value; }

      /** Callback function for orientation y axis z component.
        * param[in] value orientation y axis z value
        */
      inline void
      orientationYaxisZCallback (const float& value) { orientation_ (1,2) = value; }
      
      /** Callback function for orientation z axis x component.
        * param[in] value orientation z axis x value
        */
      inline void
      orientationZaxisXCallback (const float& value) { orientation_ (2,0) = value; }
    
      /** Callback function for orientation z axis y component.
        * param[in] value orientation z axis y value
        */
      inline void
      orientationZaxisYCallback (const float& value) { orientation_ (2,1) = value; }
      
      /** Callback function for orientation z axis z component.
        * param[in] value orientation z axis z value
        */
      inline void
      orientationZaxisZCallback (const float& value) { orientation_ (2,2) = value; }
      
      /** Callback function to set the cloud height
        * param[in] height cloud height
        */
      inline void
      cloudHeightCallback (const int &height) { cloud_->height = height; }

      /** Callback function to set the cloud width
        * param[in] width cloud width
        */
      inline void
      cloudWidthCallback (const int &width) { cloud_->width = width; }
        
      /** Append a scalar property to the cloud fields.
        * param[in] name property name
        * param[in] count property count: 1 for scalar properties and higher for a
        * list property.
        */
      template<typename Scalar> void
      appendScalarProperty (const std::string& name, const std::size_t& count = 1);

      /** Amend property from cloud fields identified by \a old_name renaming
        * it \a new_name.
        * param[in] old_name property old name
        * param[in] new_name property new name
        */
      void
      amendProperty (const std::string& old_name, const std::string& new_name, std::uint8_t datatype = 0);

      /** Callback function for the begin of vertex line */
      void
      vertexBeginCallback ();

      /** Callback function for the end of vertex line */
      void
      vertexEndCallback ();

      /** Callback function for the begin of range_grid line */
      void
      rangeGridBeginCallback ();

      /** Callback function for the begin of range_grid vertex_indices property 
        * param[in] size vertex_indices list size  
        */
      void
      rangeGridVertexIndicesBeginCallback (pcl::io::ply::uint8 size);

      /** Callback function for each range_grid vertex_indices element
        * param[in] vertex_index index of the vertex in vertex_indices
        */      
      void
      rangeGridVertexIndicesElementCallback (pcl::io::ply::int32 vertex_index);

      /** Callback function for the end of a range_grid vertex_indices property */
      void
      rangeGridVertexIndicesEndCallback ();

      /** Callback function for the end of a range_grid element end */
      void
      rangeGridEndCallback ();

      /** Callback function for obj_info */
      void
      objInfoCallback (const std::string& line);

      /** Callback function for the begin of face line */
      void
      faceBeginCallback ();

      /** Callback function for the begin of face vertex_indices property
        * param[in] size vertex_indices list size
        */
      void
      faceVertexIndicesBeginCallback (pcl::io::ply::uint8 size);

      /** Callback function for each face vertex_indices element
        * param[in] vertex_index index of the vertex in vertex_indices
        */
      void
      faceVertexIndicesElementCallback (pcl::io::ply::int32 vertex_index);

      /** Callback function for the end of a face vertex_indices property */
      void
      faceVertexIndicesEndCallback ();

      /** Callback function for the end of a face element end */
      void
      faceEndCallback ();

      /// origin
      Eigen::Vector4f origin_;

      /// orientation
      Eigen::Matrix3f orientation_;

      //vertex element artifacts
      pcl::PCLPointCloud2 *cloud_;
      std::size_t vertex_count_;
      int vertex_offset_before_;
      //range element artifacts
      std::vector<std::vector <int> > *range_grid_;
      std::size_t rgb_offset_before_;
      bool do_resize_;
      //face element artifact
      std::vector<pcl::Vertices> *polygons_;
    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
      
    private:
      // RGB values stored by vertexColorCallback()
      std::int32_t r_, g_, b_;
      // Color values stored by vertexAlphaCallback()
      std::uint32_t a_, rgba_;
  };

  /** \brief Point Cloud Data (PLY) file format writer.
    * \author Nizar Sallem
    * \ingroup io
    */
  class PCL_EXPORTS PLYWriter : public FileWriter
  {
    public:
      ///Constructor
      PLYWriter () {};

      ///Destructor
      ~PLYWriter () {};

      /** \brief Generate the header of a PLY v.7 file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[in] valid_points number of valid points (finite ones for range_grid and
        * all of them for camer)
        * \param[in] use_camera if set to true then PLY file will use element camera else
        * element range_grid will be used.
        */
      inline std::string
      generateHeaderBinary (const pcl::PCLPointCloud2 &cloud,
                            const Eigen::Vector4f &origin, 
                            const Eigen::Quaternionf &orientation,
                            int valid_points,
                            bool use_camera = true)
      {
        return (generateHeader (cloud, origin, orientation, true, use_camera, valid_points));
      }
      
      /** \brief Generate the header of a PLY v.7 file format
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[in] valid_points number of valid points (finite ones for range_grid and
        * all of them for camer)
        * \param[in] use_camera if set to true then PLY file will use element camera else
        * element range_grid will be used.
        */
      inline std::string
      generateHeaderASCII (const pcl::PCLPointCloud2 &cloud,
                           const Eigen::Vector4f &origin, 
                           const Eigen::Quaternionf &orientation,
                           int valid_points,
                           bool use_camera = true)
      {
        return (generateHeader (cloud, origin, orientation, false, use_camera, valid_points));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points, in ASCII format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[in] precision the specified output numeric stream precision (default: 8)
        * \param[in] use_camera if set to true then PLY file will use element camera else
        * element range_grid will be used.
        */
      int 
      writeASCII (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                  const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                  const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                  int precision = 8,
                  bool use_camera = true);

      /** \brief Save point cloud data to a PLY file containing n-D points, in BINARY format
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor data acquisition origin (translation)
        * \param[in] orientation the sensor data acquisition origin (rotation)
        * \param[in] use_camera if set to true then PLY file will use element camera else
        * element range_grid will be used
        */
      int 
      writeBinary (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                   const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                   const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                   bool use_camera = true);

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        */
      inline int
      write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             const bool binary = false) override
      {
        if (binary)
          return (this->writeBinary (file_name, cloud, origin, orientation, true));
        return (this->writeASCII (file_name, cloud, origin, orientation, 8, true));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        * \param[in] use_camera set to true to use camera element and false to
        * use range_grid element
        */
      inline int
      write (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             bool binary = false,
             bool use_camera = true)
      {
        if (binary)
          return (this->writeBinary (file_name, cloud, origin, orientation, use_camera));
        return (this->writeASCII (file_name, cloud, origin, orientation, 8, use_camera));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the point cloud data message (boost shared pointer)
        * \param[in] origin the sensor acquisition origin
        * \param[in] orientation the sensor acquisition orientation
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        * \param[in] use_camera set to true to use camera element and false to
        * use range_grid element
        */
      inline int
      write (const std::string &file_name, const pcl::PCLPointCloud2::ConstPtr &cloud,
             const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
             const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
             bool binary = false,
             bool use_camera = true)
      {
        return (write (file_name, *cloud, origin, orientation, binary, use_camera));
      }

      /** \brief Save point cloud data to a PLY file containing n-D points
        * \param[in] file_name the output file name
        * \param[in] cloud the pcl::PointCloud data
        * \param[in] binary set to true if the file is to be written in a binary
        * PLY format, false (default) for ASCII
        * \param[in] use_camera set to true to use camera element and false to
        * use range_grid element
        */
      template<typename PointT> inline int
      write (const std::string &file_name, 
             const pcl::PointCloud<PointT> &cloud, 
             bool binary = false,
             bool use_camera = true)
      {
        Eigen::Vector4f origin = cloud.sensor_origin_;
        Eigen::Quaternionf orientation = cloud.sensor_orientation_;

        pcl::PCLPointCloud2 blob;
        pcl::toPCLPointCloud2 (cloud, blob);

        // Save the data
        return (this->write (file_name, blob, origin, orientation, binary, use_camera));
      }
      
    private:
      /** \brief Generate a PLY header.
        * \param[in] cloud the input point cloud
        * \param[in] binary whether the PLY file should be saved as binary data (true) or ascii (false)
        */
      std::string
      generateHeader (const pcl::PCLPointCloud2 &cloud,
                      const Eigen::Vector4f &origin, 
                      const Eigen::Quaternionf &orientation,
                      bool binary, 
                      bool use_camera,
                      int valid_points);
      
      void
      writeContentWithCameraASCII (int nr_points, 
                                   int point_size,
                                   const pcl::PCLPointCloud2 &cloud,
                                   const Eigen::Vector4f &origin, 
                                   const Eigen::Quaternionf &orientation,
                                   std::ofstream& fs);

      void
      writeContentWithRangeGridASCII (int nr_points, 
                                      int point_size,
                                      const pcl::PCLPointCloud2 &cloud,
                                      std::ostringstream& fs,
                                      int& nb_valid_points);
  };

  namespace io
  {
    /** \brief Load a PLY v.6 file into a templated PointCloud type.
      *
      * Any PLY files containing sensor data will generate a warning as a
      * pcl/PCLPointCloud2 message cannot hold the sensor origin.
      *
      * \param[in] file_name the name of the file to load
      * \param[in] cloud the resultant templated point cloud
      * \ingroup io
      */
    inline int
    loadPLYFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud)
    {
      pcl::PLYReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Load any PLY file into a templated PointCloud type.
      * \param[in] file_name the name of the file to load
      * \param[in] cloud the resultant templated point cloud
      * \param[in] origin the sensor acquisition origin (only for > PLY_V7 - null if not present)
      * \param[in] orientation the sensor acquisition orientation if available, 
      * identity if not present
      * \ingroup io
      */
    inline int
    loadPLYFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                 Eigen::Vector4f &origin, Eigen::Quaternionf &orientation)
    {
      pcl::PLYReader p;
      int ply_version;
      return (p.read (file_name, cloud, origin, orientation, ply_version));
    }

    /** \brief Load any PLY file into a templated PointCloud type
      * \param[in] file_name the name of the file to load
      * \param[in] cloud the resultant templated point cloud
      * \ingroup io
      */
    template<typename PointT> inline int
    loadPLYFile (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
    {
      pcl::PLYReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Load a PLY file into a PolygonMesh type.
      *
      * Any PLY files containing sensor data will generate a warning as a
      * pcl/PolygonMesh message cannot hold the sensor origin.
      *
      * \param[in] file_name the name of the file to load
      * \param[in] mesh the resultant polygon mesh
      * \ingroup io
      */
    inline int
    loadPLYFile (const std::string &file_name, pcl::PolygonMesh &mesh)
    {
      pcl::PLYReader p;
      return (p.read (file_name, mesh));
    }

    /** \brief Save point cloud data to a PLY file containing n-D points
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] origin the sensor data acquisition origin (translation)
      * \param[in] orientation the sensor data acquisition origin (rotation)
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      * \param[in] use_camera
      * \ingroup io
      */
    inline int 
    savePLYFile (const std::string &file_name, const pcl::PCLPointCloud2 &cloud,
                 const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
                 const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
                 bool binary_mode = false, bool use_camera = true)
    {
      PLYWriter w;
      return (w.write (file_name, cloud, origin, orientation, binary_mode, use_camera));
    }

    /** \brief Templated version for saving point cloud data to a PLY file
      * containing a specific given cloud format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      * \ingroup io
      */
    template<typename PointT> inline int
    savePLYFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud, bool binary_mode = false)
    {
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud, binary_mode));
    }

    /** \brief Templated version for saving point cloud data to a PLY file
      * containing a specific given cloud format.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePLYFileASCII (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud, false));
    }

    /** \brief Templated version for saving point cloud data to a PLY file containing a specific given cloud format.
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \ingroup io
      */
    template<typename PointT> inline int
    savePLYFileBinary (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
    {
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud, true));
    }

    /** \brief Templated version for saving point cloud data to a PLY file containing a specific given cloud format
      * \param[in] file_name the output file name
      * \param[in] cloud the point cloud data message
      * \param[in] indices the set of indices to save
      * \param[in] binary_mode true for binary mode, false (default) for ASCII
      * \ingroup io
      */
    template<typename PointT> int
    savePLYFile (const std::string &file_name, const pcl::PointCloud<PointT> &cloud,
                 const pcl::Indices &indices, bool binary_mode = false)
    {
      // Copy indices to a new point cloud
      pcl::PointCloud<PointT> cloud_out;
      copyPointCloud (cloud, indices, cloud_out);
      // Save the data
      PLYWriter w;
      return (w.write<PointT> (file_name, cloud_out, binary_mode));
    }

    /** \brief Saves a PolygonMesh in ascii PLY format.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] mesh the polygonal mesh to save
      * \param[in] precision the output ASCII precision default 5
      * \ingroup io
      */
    PCL_EXPORTS int
    savePLYFile (const std::string &file_name, const pcl::PolygonMesh &mesh, unsigned precision = 5);
    
    /** \brief Saves a PolygonMesh in binary PLY format.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] mesh the polygonal mesh to save
      * \ingroup io
      */
    PCL_EXPORTS int
    savePLYFileBinary (const std::string &file_name, const pcl::PolygonMesh &mesh);
  }
}
