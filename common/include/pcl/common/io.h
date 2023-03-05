/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */

#pragma once

#include <numeric>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/pcl_macros.h>
#include <pcl/PolygonMesh.h>
#include <locale>

namespace pcl
{
  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \param[in] cloud the point cloud message
    * \param[in] field_name the string defining the field name
    * \ingroup common
    */
  inline int
  getFieldIndex (const pcl::PCLPointCloud2 &cloud, const std::string &field_name)
  {
    // Get the index we need
    const auto result = std::find_if(cloud.fields.begin (), cloud.fields.end (),
        [&field_name](const auto field) { return field.name == field_name; });
    if (result == cloud.fields.end ())
      return -1;
    return std::distance(cloud.fields.begin (), result);
  }

  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \tparam PointT datatype for which fields is being queries
    * \param[in] field_name the string defining the field name
    * \param[out] fields a vector to the original \a PCLPointField vector that the raw PointCloud message contains
    * \ingroup common
    */
  template <typename PointT> inline int
  getFieldIndex (const std::string &field_name,
                 std::vector<pcl::PCLPointField> &fields);
  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \tparam PointT datatype for which fields is being queries
    * \param[in] field_name the string defining the field name
    * \param[in] fields a vector to the original \a PCLPointField vector that the raw PointCloud message contains
    * \ingroup common
    */
  template <typename PointT> inline int
  getFieldIndex (const std::string &field_name,
                 const std::vector<pcl::PCLPointField> &fields);

  /** \brief Get the list of available fields (i.e., dimension/channel)
    * \tparam PointT datatype whose details are requested
    * \ingroup common
    */
  template <typename PointT> inline std::vector<pcl::PCLPointField>
  getFields ();

  /** \brief Get the list of all fields available in a given cloud
    * \param[in] cloud the point cloud message
    * \ingroup common
    */
  template <typename PointT> inline std::string
  getFieldsList (const pcl::PointCloud<PointT> &cloud);

  /** \brief Get the available point cloud fields as a space separated string
    * \param[in] cloud a pointer to the PointCloud message
    * \ingroup common
    */
  inline std::string
  getFieldsList (const pcl::PCLPointCloud2 &cloud)
  {
    return std::accumulate(std::next (cloud.fields.begin ()), cloud.fields.end (), cloud.fields[0].name,
        [](const auto& acc, const auto& field) { return acc + " " + field.name; });
  }

  /** \brief Obtains the size of a specific field data type in bytes
    * \param[in] datatype the field data type (see PCLPointField.h)
    * \ingroup common
    */
  inline int
  getFieldSize (const int datatype)
  {
    switch (datatype)
    {
      case pcl::PCLPointField::BOOL:
          return sizeof(bool);

      case pcl::PCLPointField::INT8: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT8:
        return (1);

      case pcl::PCLPointField::INT16: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT16:
        return (2);

      case pcl::PCLPointField::INT32: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT32: PCL_FALLTHROUGH
      case pcl::PCLPointField::FLOAT32:
        return (4);

      case pcl::PCLPointField::INT64: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT64: PCL_FALLTHROUGH
      case pcl::PCLPointField::FLOAT64:
        return (8);

      default:
        return (0);
    }
  }

  /** \brief Obtain a vector with the sizes of all valid fields (e.g., not "_")
    * \param[in] fields the input vector containing the fields
    * \param[out] field_sizes the resultant field sizes in bytes
    */
  PCL_EXPORTS void
  getFieldsSizes (const std::vector<pcl::PCLPointField> &fields,
                  std::vector<int> &field_sizes);

  /** \brief Obtains the type of the PCLPointField from a specific size and type
    * \param[in] size the size in bytes of the data field
    * \param[in] type a char describing the type of the field  ('B' = bool, 'F' = float, 'I' = signed, 'U' = unsigned)
    * \ingroup common
    */
  inline int
  getFieldType (const int size, char type)
  {
    type = std::toupper (type, std::locale::classic ());

    // extra logic for bool because its size is undefined
    if (type == 'B') {
      if (size == sizeof(bool)) {
        return pcl::PCLPointField::BOOL;
      } else {
        return -1;
      }
    }

    switch (size)
    {
      case 1:
        if (type == 'I')
          return (pcl::PCLPointField::INT8);
        if (type == 'U')
          return (pcl::PCLPointField::UINT8);
        break;

      case 2:
        if (type == 'I')
          return (pcl::PCLPointField::INT16);
        if (type == 'U')
          return (pcl::PCLPointField::UINT16);
        break;

      case 4:
        if (type == 'I')
          return (pcl::PCLPointField::INT32);
        if (type == 'U')
          return (pcl::PCLPointField::UINT32);
        if (type == 'F')
          return (pcl::PCLPointField::FLOAT32);
        break;

      case 8:
        if (type == 'I')
          return (pcl::PCLPointField::INT64);
        if (type == 'U')
          return (pcl::PCLPointField::UINT64);
        if (type == 'F')
          return (pcl::PCLPointField::FLOAT64);
        break;
    }
    return (-1);
  }

  /** \brief Obtains the type of the PCLPointField from a specific PCLPointField as a char
    * \param[in] type the PCLPointField field type
    * \ingroup common
    */
  inline char
  getFieldType (const int type)
  {
    switch (type)
    {
      case pcl::PCLPointField::BOOL:
        return ('B');

      case pcl::PCLPointField::INT8: PCL_FALLTHROUGH
      case pcl::PCLPointField::INT16: PCL_FALLTHROUGH
      case pcl::PCLPointField::INT32: PCL_FALLTHROUGH
      case pcl::PCLPointField::INT64:
        return ('I');

      case pcl::PCLPointField::UINT8: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT16: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT32: PCL_FALLTHROUGH
      case pcl::PCLPointField::UINT64:
        return ('U');

      case pcl::PCLPointField::FLOAT32: PCL_FALLTHROUGH
      case pcl::PCLPointField::FLOAT64:
        return ('F');

      default:
        return ('?');
    }
  }

  enum InterpolationType
  {
    BORDER_CONSTANT = 0, BORDER_REPLICATE = 1,
    BORDER_REFLECT = 2, BORDER_WRAP = 3,
    BORDER_REFLECT_101 = 4, BORDER_TRANSPARENT = 5,
    BORDER_DEFAULT = BORDER_REFLECT_101
  };

  /** \brief \return the right index according to the interpolation type.
    * \note this is adapted from OpenCV
    * \param p the index of point to interpolate
    * \param length the top/bottom row or left/right column index
    * \param type the requested interpolation
    * \throws pcl::BadArgumentException if type is unknown
    */
  PCL_EXPORTS int
  interpolatePointIndex (int p, int length, InterpolationType type);

  /** \brief Concatenate two pcl::PointCloud<PointT>
    * \param[in] cloud1 the first input point cloud dataset
    * \param[in] cloud2 the second input point cloud dataset
    * \param[out] cloud_out the resultant output point cloud dataset
    * \return true if successful, false otherwise
    * \ingroup common
    */
  template <typename PointT>
  PCL_EXPORTS bool
  concatenate (const pcl::PointCloud<PointT> &cloud1,
               const pcl::PointCloud<PointT> &cloud2,
               pcl::PointCloud<PointT> &cloud_out)
  {
    return pcl::PointCloud<PointT>::concatenate(cloud1, cloud2, cloud_out);
  }

  /** \brief Concatenate two pcl::PCLPointCloud2
    *
    * \warning This function will concatenate IFF the non-skip fields are in the correct
    * order and same in number.
    * \param[in] cloud1 the first input point cloud dataset
    * \param[in] cloud2 the second input point cloud dataset
    * \param[out] cloud_out the resultant output point cloud dataset
    * \return true if successful, false otherwise
    * \ingroup common
    */
  PCL_EXPORTS inline bool
  concatenate (const pcl::PCLPointCloud2 &cloud1,
               const pcl::PCLPointCloud2 &cloud2,
               pcl::PCLPointCloud2 &cloud_out)
  {
    return pcl::PCLPointCloud2::concatenate(cloud1, cloud2, cloud_out);
  }

  /** \brief Concatenate two pcl::PolygonMesh
    * \param[in] mesh1 the first input mesh
    * \param[in] mesh2 the second input mesh
    * \param[out] mesh_out the resultant output mesh
    * \return true if successful, false otherwise
    * \ingroup common
    */
  PCL_EXPORTS inline bool
  concatenate (const pcl::PolygonMesh &mesh1,
               const pcl::PolygonMesh &mesh2,
               pcl::PolygonMesh &mesh_out)
  {
    return pcl::PolygonMesh::concatenate(mesh1, mesh2, mesh_out);
  }

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  PCL_EXPORTS void
  copyPointCloud (const pcl::PCLPointCloud2 &cloud_in,
                  const Indices &indices,
                  pcl::PCLPointCloud2 &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  PCL_EXPORTS void
  copyPointCloud (const pcl::PCLPointCloud2 &cloud_in,
                  const IndicesAllocator< Eigen::aligned_allocator<index_t> > &indices,
                  pcl::PCLPointCloud2 &cloud_out);

  /** \brief Copy fields and point cloud data from \a cloud_in to \a cloud_out
    * \param[in] cloud_in the input point cloud dataset
    * \param[out] cloud_out the resultant output point cloud dataset
    * \ingroup common
    */
  PCL_EXPORTS void
  copyPointCloud (const pcl::PCLPointCloud2 &cloud_in,
                  pcl::PCLPointCloud2 &cloud_out);

  /** \brief Check if two given point types are the same or not. */
template <typename Point1T, typename Point2T> constexpr bool
  isSamePointType() noexcept
  {
    return (std::is_same<remove_cvref_t<Point1T>, remove_cvref_t<Point2T>>::value);
  }

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointT, typename IndicesVectorAllocator = std::allocator<index_t>> void
  copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const IndicesAllocator< IndicesVectorAllocator> &indices,
                  pcl::PointCloud<PointT> &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the PointIndices structure representing the points to be copied from cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointT> void
  copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const PointIndices &indices,
                  pcl::PointCloud<PointT> &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointT> void
  copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  const std::vector<pcl::PointIndices> &indices,
                  pcl::PointCloud<PointT> &cloud_out);

  /** \brief Copy all the fields from a given point cloud into a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[out] cloud_out the resultant output point cloud dataset
    * \ingroup common
    */
  template <typename PointInT, typename PointOutT> void
  copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                  pcl::PointCloud<PointOutT> &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointInT, typename PointOutT, typename IndicesVectorAllocator = std::allocator<index_t>> void
  copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                  const IndicesAllocator<IndicesVectorAllocator> &indices,
                  pcl::PointCloud<PointOutT> &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the PointIndices structure representing the points to be copied from cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointInT, typename PointOutT> void
  copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                  const PointIndices &indices,
                  pcl::PointCloud<PointOutT> &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointInT, typename PointOutT> void
  copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in,
                  const std::vector<pcl::PointIndices> &indices,
                  pcl::PointCloud<PointOutT> &cloud_out);

  /** \brief Copy a point cloud inside a larger one interpolating borders.
    * \param[in] cloud_in the input point cloud dataset
    * \param[out] cloud_out the resultant output point cloud dataset
    * \param top
    * \param bottom
    * \param left
    * \param right
    * Position of cloud_in inside cloud_out is given by \a top, \a left, \a bottom \a right.
    * \param[in] border_type the interpolating method (pcl::BORDER_XXX)
    *  BORDER_REPLICATE:     aaaaaa|abcdefgh|hhhhhhh
    *  BORDER_REFLECT:       fedcba|abcdefgh|hgfedcb
    *  BORDER_REFLECT_101:   gfedcb|abcdefgh|gfedcba
    *  BORDER_WRAP:          cdefgh|abcdefgh|abcdefg
    *  BORDER_CONSTANT:      iiiiii|abcdefgh|iiiiiii  with some specified 'i'
    *  BORDER_TRANSPARENT:   mnopqr|abcdefgh|tuvwxyz  where m-r and t-z are original values of cloud_out
    * \param value
    * \throw pcl::BadArgumentException if any of top, bottom, left or right is negative.
    * \ingroup common
    */
  template <typename PointT> void
  copyPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                  pcl::PointCloud<PointT> &cloud_out,
                  int top, int bottom, int left, int right,
                  pcl::InterpolationType border_type, const PointT& value);

  /** \brief Concatenate two datasets representing different fields.
    *
    * \note If the input datasets have overlapping fields (i.e., both contain
    * the same fields), then the data in the second cloud (cloud2_in) will
    * overwrite the data in the first (cloud1_in).
    *
    * \param[in] cloud1_in the first input dataset
    * \param[in] cloud2_in the second input dataset (overwrites the fields of the first dataset for those that are shared)
    * \param[out] cloud_out the resultant output dataset created by the concatenation of all the fields in the input datasets
    * \ingroup common
    */
  template <typename PointIn1T, typename PointIn2T, typename PointOutT> void
  concatenateFields (const pcl::PointCloud<PointIn1T> &cloud1_in,
                     const pcl::PointCloud<PointIn2T> &cloud2_in,
                     pcl::PointCloud<PointOutT> &cloud_out);

  /** \brief Concatenate two datasets representing different fields.
    *
    * \note If the input datasets have overlapping fields (i.e., both contain
    * the same fields), then the data in the second cloud (cloud2_in) will
    * overwrite the data in the first (cloud1_in).
    *
    * \param[in] cloud1_in the first input dataset
    * \param[in] cloud2_in the second input dataset (overwrites the fields of the first dataset for those that are shared)
    * \param[out] cloud_out the output dataset created by concatenating all the fields in the input datasets
    * \ingroup common
    */
  PCL_EXPORTS bool
  concatenateFields (const pcl::PCLPointCloud2 &cloud1_in,
                     const pcl::PCLPointCloud2 &cloud2_in,
                     pcl::PCLPointCloud2 &cloud_out);

  /** \brief Copy the XYZ dimensions of a pcl::PCLPointCloud2 into Eigen format
    * \param[in] in the point cloud message
    * \param[out] out the resultant Eigen MatrixXf format containing XYZ0 / point
    * \ingroup common
    */
  PCL_EXPORTS bool
  getPointCloudAsEigen (const pcl::PCLPointCloud2 &in, Eigen::MatrixXf &out);

  /** \brief Copy the XYZ dimensions from an Eigen MatrixXf into a pcl::PCLPointCloud2 message
    * \param[in] in the Eigen MatrixXf format containing XYZ0 / point
    * \param[out] out the resultant point cloud message
    * \note the method assumes that the PCLPointCloud2 message already has the fields set up properly !
    * \ingroup common
    */
  PCL_EXPORTS bool
  getEigenAsPointCloud (Eigen::MatrixXf &in, pcl::PCLPointCloud2 &out);

  namespace io
  {
    /** \brief swap bytes order of a char array of length N
      * \param bytes char array to swap
      * \ingroup common
      */
    template <std::size_t N> void
    swapByte (char* bytes);

   /** \brief specialization of swapByte for dimension 1
     * \param bytes char array to swap
     */
    template <> inline void
    swapByte<1> (char* bytes) { bytes[0] = bytes[0]; }


   /** \brief specialization of swapByte for dimension 2
     * \param bytes char array to swap
     */
    template <> inline void
    swapByte<2> (char* bytes) { std::swap (bytes[0], bytes[1]); }

   /** \brief specialization of swapByte for dimension 4
     * \param bytes char array to swap
     */
    template <> inline void
    swapByte<4> (char* bytes)
    {
      std::swap (bytes[0], bytes[3]);
      std::swap (bytes[1], bytes[2]);
    }

   /** \brief specialization of swapByte for dimension 8
     * \param bytes char array to swap
     */
    template <> inline void
    swapByte<8> (char* bytes)
    {
      std::swap (bytes[0], bytes[7]);
      std::swap (bytes[1], bytes[6]);
      std::swap (bytes[2], bytes[5]);
      std::swap (bytes[3], bytes[4]);
    }

    /** \brief swaps byte of an arbitrary type T casting it to char*
      * \param value the data you want its bytes swapped
      */
    template <typename T> void
    swapByte (T& value)
    {
      pcl::io::swapByte<sizeof(T)> (reinterpret_cast<char*> (&value));
    }
  }
}

#include <pcl/common/impl/io.hpp>
