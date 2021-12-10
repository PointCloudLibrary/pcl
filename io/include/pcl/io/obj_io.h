/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2013, Open Perception, Inc.
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

#include <pcl/memory.h>
#include <pcl/TextureMesh.h>
#include <pcl/io/file_io.h>

namespace pcl
{
  struct PolygonMesh;

  class PCL_EXPORTS MTLReader
  {
    public:
      /** \brief empty constructor */
      MTLReader ();

      /** \brief empty destructor */
      virtual ~MTLReader() {}

      /** \brief Read a MTL file given its full path.
        * \param[in] filename full path to MTL file
        * \return 0 on success < 0 else.
        */
      int
      read (const std::string& filename);

      /** \brief Read a MTL file given an OBJ file full path and the MTL file name.
        * \param[in] obj_file_name full path to OBJ file
        * \param[in] mtl_file_name MTL file name
        * \return 0 on success < 0 else.
        */
      int
      read (const std::string& obj_file_name, const std::string& mtl_file_name);

      std::vector<pcl::TexMaterial>::const_iterator
      getMaterial (const std::string& material_name) const;

      /// materials array
      std::vector<pcl::TexMaterial> materials_;

    private:
      /// converts CIE XYZ to RGB
      inline void
      cie2rgb (const Eigen::Vector3f& xyz, pcl::TexMaterial::RGB& rgb) const;
      /// fill a pcl::TexMaterial::RGB from a split line containing CIE x y z values
      int
      fillRGBfromXYZ (const std::vector<std::string>& split_line, pcl::TexMaterial::RGB& rgb);
      /// fill a pcl::TexMaterial::RGB from a split line containing r g b values
      int
      fillRGBfromRGB (const std::vector<std::string>& split_line, pcl::TexMaterial::RGB& rgb);
      /// matrix to convert CIE to RGB
      Eigen::Matrix3f xyz_to_rgb_matrix_;

      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };

  class PCL_EXPORTS OBJReader : public FileReader
  {
    public:
      /** \brief empty constructor */
      OBJReader() {}
      /** \brief empty destructor */
      ~OBJReader() {}
      /** \brief Read a point cloud data header from a FILE file.
        *
        * Load only the meta information (number of points, their types, etc),
        * and not the points themselves, from a given FILE file. Useful for fast
        * evaluation of the underlying data structure.
        *
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[out] origin the sensor acquisition origin always null
        * \param[out] orientation the sensor acquisition orientation always identity
        * \param[out] file_version always 0
        * \param data_type
        * \param data_idx
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        *
        * \return 0 on success.
        */
      int
      readHeader (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                  Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                  int &file_version, int &data_type, unsigned int &data_idx,
                  const int offset) override;

      /** \brief Read a point cloud data from a FILE file and store it into a
        * pcl/PCLPointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[out] origin the sensor acquisition origin always null
        * \param[out] orientation the sensor acquisition orientation always identity
        * \param[out] file_version always 0
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        *
        * \return 0 on success.
        */
      int
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
            int &file_version, const int offset = 0) override;


      /** \brief Read a point cloud data from a FILE file and store it into a
        * pcl/PCLPointCloud2.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        *
        * \return 0 on success.
        */
      int
      read (const std::string &file_name, pcl::PCLPointCloud2 &cloud, const int offset = 0);

      /** \brief Read a point cloud data from a FILE file and store it into a
        * pcl/TextureMesh.
        * \param[in] file_name the name of the file containing data
        * \param[out] mesh the resultant TextureMesh read from disk
        * \param[out] origin the sensor origin always null
        * \param[out] orientation the sensor orientation always identity
        * \param[out] file_version always 0
        * \param[in] offset the offset in the file where to expect the true
        * header to begin.
        *
        * \return 0 on success.
        */
      int
      read (const std::string &file_name, pcl::TextureMesh &mesh,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
            int &file_version, const int offset = 0);

      /** \brief Read a point cloud data from a FILE file and store it into a
        * pcl/TextureMesh.
        * \param[in] file_name the name of the file containing data
        * \param[out] mesh the resultant TextureMesh read from disk
        * \param[in] offset the offset in the file where to expect the true
        * header to begin.
        *
        * \return 0 on success.
        */
      int
      read (const std::string &file_name, pcl::TextureMesh &mesh, const int offset = 0);

      /** \brief Read a point cloud data from a FILE file and store it into a
        * pcl/PolygonMesh.
        * \param[in] file_name the name of the file containing data
        * \param[out] mesh the resultant PolygonMesh read from disk
        * \param[out] origin the sensor origin always null
        * \param[out] orientation the sensor orientation always identity
        * \param[out] file_version always 0
        * \param[in] offset the offset in the file where to expect the true
        * header to begin.
        *
        * \return 0 on success.
        */
      int
      read (const std::string &file_name, pcl::PolygonMesh &mesh,
            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
            int &file_version, const int offset = 0);

      /** \brief Read a point cloud data from a FILE file and store it into a
        * pcl/PolygonMesh.
        * \param[in] file_name the name of the file containing data
        * \param[out] mesh the resultant PolygonMesh read from disk
        * \param[in] offset the offset in the file where to expect the true
        * header to begin.
        *
        * \return 0 on success.
        */
      int
      read (const std::string &file_name, pcl::PolygonMesh &mesh, const int offset = 0);

      /** \brief Read a point cloud data from any FILE file, and convert it to the given
        * template format.
        * \param[in] file_name the name of the file containing the actual PointCloud data
        * \param[out] cloud the resultant PointCloud message read from disk
        * \param[in] offset the offset in the file where to expect the true header to begin.
        * One usage example for setting the offset parameter is for reading
        * data from a TAR "archive containing multiple files: TAR files always
        * add a 512 byte header in front of the actual file, so set the offset
        * to the next byte after the header (e.g., 513).
        */
      template<typename PointT> inline int
      read (const std::string &file_name, pcl::PointCloud<PointT> &cloud,
            const int offset  =0)
      {
        pcl::PCLPointCloud2 blob;
        int file_version;
        int res = read (file_name, blob, cloud.sensor_origin_, cloud.sensor_orientation_,
                        file_version, offset);
        if (res < 0)
          return (res);

        pcl::fromPCLPointCloud2 (blob, cloud);
        return (0);
      }

    private:
      /// Usually OBJ files come MTL files where texture materials are stored
      std::vector<pcl::MTLReader> companions_;
  };

  namespace io
  {
    /** \brief Load any OBJ file into a templated PointCloud type.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \param[out] origin the sensor acquisition origin, null
      * \param[out] orientation the sensor acquisition orientation, identity
      * \ingroup io
      */
    inline int
    loadOBJFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud,
                 Eigen::Vector4f &origin, Eigen::Quaternionf &orientation)
    {
      pcl::OBJReader p;
      int obj_version;
      return (p.read (file_name, cloud, origin, orientation, obj_version));
    }

    /** \brief Load an OBJ file into a PCLPointCloud2 blob type.
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    inline int
    loadOBJFile (const std::string &file_name, pcl::PCLPointCloud2 &cloud)
    {
      pcl::OBJReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Load any OBJ file into a templated PointCloud type
      * \param[in] file_name the name of the file to load
      * \param[out] cloud the resultant templated point cloud
      * \ingroup io
      */
    template<typename PointT> inline int
    loadOBJFile (const std::string &file_name, pcl::PointCloud<PointT> &cloud)
    {
      pcl::OBJReader p;
      return (p.read (file_name, cloud));
    }

    /** \brief Load any OBJ file into a PolygonMesh type.
      * \param[in] file_name the name of the file to load
      * \param[out] mesh the resultant mesh
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    inline int
    loadOBJFile (const std::string &file_name, pcl::PolygonMesh &mesh)
    {
      pcl::OBJReader p;
      return (p.read (file_name, mesh));
    }

    /** \brief Load any OBJ file into a TextureMesh type.
      * \param[in] file_name the name of the file to load
      * \param[out] mesh the resultant mesh
      * \return 0 on success < 0 on error
      *
      * \ingroup io
      */
    inline int
    loadOBJFile (const std::string &file_name, pcl::TextureMesh &mesh)
    {
      pcl::OBJReader p;
      return (p.read (file_name, mesh));
    }

    /** \brief Saves a TextureMesh in ascii OBJ format.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] tex_mesh the texture mesh to save
      * \param[in] precision the output ASCII precision
      * \ingroup io
      */
    PCL_EXPORTS int
    saveOBJFile (const std::string &file_name,
                 const pcl::TextureMesh &tex_mesh,
                 unsigned precision = 5);

    /** \brief Saves a PolygonMesh in ascii PLY format.
      * \param[in] file_name the name of the file to write to disk
      * \param[in] mesh the polygonal mesh to save
      * \param[in] precision the output ASCII precision default 5
      * \ingroup io
      */
    PCL_EXPORTS int
    saveOBJFile (const std::string &file_name,
                 const pcl::PolygonMesh &mesh,
                 unsigned precision = 5);

  }
}
