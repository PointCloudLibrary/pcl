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

#ifndef PCL_COMMON_IO_H_
#define PCL_COMMON_IO_H_

#include <string>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <pcl/exceptions.h>
#include <locale>

namespace pcl
{
  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \param[in] cloud the the point cloud message
    * \param[in] field_name the string defining the field name
    * \ingroup common
    */
  inline int
  getFieldIndex (const pcl::PCLPointCloud2 &cloud, const std::string &field_name)
  {
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size (); ++d)
      if (cloud.fields[d].name == field_name)
        return (static_cast<int>(d));
    return (-1);
  }

  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \param[in] cloud the the point cloud message
    * \param[in] field_name the string defining the field name
    * \param[out] fields a vector to the original \a PCLPointField vector that the raw PointCloud message contains
    * \ingroup common
    */
  template <typename PointT> inline int 
  getFieldIndex (const pcl::PointCloud<PointT> &cloud, const std::string &field_name, 
                 std::vector<pcl::PCLPointField> &fields);

  /** \brief Get the index of a specified field (i.e., dimension/channel)
    * \param[in] field_name the string defining the field name
    * \param[out] fields a vector to the original \a PCLPointField vector that the raw PointCloud message contains
    * \ingroup common
    */
  template <typename PointT> inline int 
  getFieldIndex (const std::string &field_name, 
                 std::vector<pcl::PCLPointField> &fields);

  /** \brief Get the list of available fields (i.e., dimension/channel)
    * \param[in] cloud the point cloud message
    * \param[out] fields a vector to the original \a PCLPointField vector that the raw PointCloud message contains
    * \ingroup common
    */
  template <typename PointT> inline void 
  getFields (const pcl::PointCloud<PointT> &cloud, std::vector<pcl::PCLPointField> &fields);

  /** \brief Get the list of available fields (i.e., dimension/channel)
    * \param[out] fields a vector to the original \a PCLPointField vector that the raw PointCloud message contains
    * \ingroup common
    */
  template <typename PointT> inline void 
  getFields (std::vector<pcl::PCLPointField> &fields);

  /** \brief Get the list of all fields available in a given cloud
    * \param[in] cloud the the point cloud message
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
    std::string result;
    for (size_t i = 0; i < cloud.fields.size () - 1; ++i)
      result += cloud.fields[i].name + " ";
    result += cloud.fields[cloud.fields.size () - 1].name;
    return (result);
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
      case pcl::PCLPointField::INT8:
      case pcl::PCLPointField::UINT8:
        return (1);

      case pcl::PCLPointField::INT16:
      case pcl::PCLPointField::UINT16:
        return (2);

      case pcl::PCLPointField::INT32:
      case pcl::PCLPointField::UINT32:
      case pcl::PCLPointField::FLOAT32:
        return (4);

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
    * \param[in] type a char describing the type of the field  ('F' = float, 'I' = signed, 'U' = unsigned)
    * \ingroup common
    */
  inline int
  getFieldType (const int size, char type)
  {
    type = std::toupper (type, std::locale::classic ());
    switch (size)
    {
      case 1:
        if (type == 'I')
          return (pcl::PCLPointField::INT8);
        if (type == 'U')
          return (pcl::PCLPointField::UINT8);

      case 2:
        if (type == 'I')
          return (pcl::PCLPointField::INT16);
        if (type == 'U')
          return (pcl::PCLPointField::UINT16);

      case 4:
        if (type == 'I')
          return (pcl::PCLPointField::INT32);
        if (type == 'U')
          return (pcl::PCLPointField::UINT32);
        if (type == 'F')
          return (pcl::PCLPointField::FLOAT32);

      case 8:
        return (pcl::PCLPointField::FLOAT64);

      default:
        return (-1);
    }
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
      case pcl::PCLPointField::INT8:
      case pcl::PCLPointField::INT16:
      case pcl::PCLPointField::INT32:
        return ('I');

      case pcl::PCLPointField::UINT8:
      case pcl::PCLPointField::UINT16:
      case pcl::PCLPointField::UINT32:
        return ('U');

      case pcl::PCLPointField::FLOAT32:
      case pcl::PCLPointField::FLOAT64:
        return ('F');
      default:
        return ('?');
    }
  }

  typedef enum
  {
    BORDER_CONSTANT = 0, BORDER_REPLICATE = 1,
    BORDER_REFLECT = 2, BORDER_WRAP = 3,
    BORDER_REFLECT_101 = 4, BORDER_TRANSPARENT = 5,
    BORDER_DEFAULT = BORDER_REFLECT_101
  } InterpolationType;

  /** \brief \return the right index according to the interpolation type.
    * \note this is adapted from OpenCV
    * \param p the index of point to interpolate
    * \param length the top/bottom row or left/right column index
    * \param type the requested interpolation
    * \throws pcl::BadArgumentException if type is unknown
    */
  PCL_EXPORTS int
  interpolatePointIndex (int p, int length, InterpolationType type);

  /** \brief Concatenate two pcl::PCLPointCloud2.
    * \param[in] cloud1 the first input point cloud dataset
    * \param[in] cloud2 the second input point cloud dataset
    * \param[out] cloud_out the resultant output point cloud dataset
    * \return true if successful, false if failed (e.g., name/number of fields differs)
    * \ingroup common
    */
  PCL_EXPORTS bool 
  concatenatePointCloud (const pcl::PCLPointCloud2 &cloud1,
                         const pcl::PCLPointCloud2 &cloud2,
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
                  const std::vector<int> &indices, 
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
                  const std::vector<int, Eigen::aligned_allocator<int> > &indices, 
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
  template <typename Point1T, typename Point2T> inline bool
  isSamePointType ()
  {
    return (typeid (Point1T) == typeid (Point2T));
  }

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointT> void 
  copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                  const std::vector<int> &indices, 
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
                  const std::vector<int, Eigen::aligned_allocator<int> > &indices, 
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
  template <typename PointInT, typename PointOutT> void 
  copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in, 
                  const std::vector<int> &indices, 
                  pcl::PointCloud<PointOutT> &cloud_out);

  /** \brief Extract the indices of a given point cloud as a new point cloud
    * \param[in] cloud_in the input point cloud dataset
    * \param[in] indices the vector of indices representing the points to be copied from \a cloud_in
    * \param[out] cloud_out the resultant output point cloud dataset
    * \note Assumes unique indices.
    * \ingroup common
    */
  template <typename PointInT, typename PointOutT> void 
  copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in, 
                  const std::vector<int, Eigen::aligned_allocator<int> > &indices, 
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
    *  BORDER_TRANSPARENT:   mnopqr|abcdefgh|tuvwxyz  where m-r and t-z are orignal values of cloud_out
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

#endif  //#ifndef PCL_COMMON_IO_H_

