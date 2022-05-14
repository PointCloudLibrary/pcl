/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#pragma once

#include <iostream>
#include <vector>

namespace pcl
{

namespace octree
{

/** \brief @b ColorCoding class
 *  \note This class encodes 8-bit color information for octree-based point cloud compression.
 *  \note
 *  \note typename: PointT: type of point used in pointcloud
 *  \author Julius Kammerl (julius@kammerl.de)
 */
template<typename PointT>
class ColorCoding
{
  // public typedefs
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  /** \brief Constructor.
   *
   * */
  ColorCoding () :
    output_ (), colorBitReduction_ (0)
  {
  }

  /** \brief Empty class constructor. */
  virtual
  ~ColorCoding ()
  {
  }

  /** \brief Define color bit depth of encoded color information.
    * \param bitDepth_arg: amounts of bits for representing one color component
    */
  inline
  void
  setBitDepth (unsigned char bitDepth_arg)
  {
    colorBitReduction_ = static_cast<unsigned char> (8 - bitDepth_arg);
  }

  /** \brief Retrieve color bit depth of encoded color information.
    * \return amounts of bits for representing one color component
    */
  inline unsigned char
  getBitDepth ()
  {
    return (static_cast<unsigned char> (8 - colorBitReduction_));
  }

  /** \brief Set amount of voxels containing point color information and reserve memory
    * \param voxelCount_arg: amounts of voxels
    */
  inline void
  setVoxelCount (unsigned int voxelCount_arg)
  {
    pointAvgColorDataVector_.reserve (voxelCount_arg * 3);
  }

  /** \brief Set amount of points within point cloud to be encoded and reserve memory
   *  \param pointCount_arg: amounts of points within point cloud
   * */
  inline
  void
  setPointCount (unsigned int pointCount_arg)
  {
    pointDiffColorDataVector_.reserve (pointCount_arg * 3);
  }

  /** \brief Initialize encoding of color information
   * */
  void
  initializeEncoding ()
  {
    pointAvgColorDataVector_.clear ();

    pointDiffColorDataVector_.clear ();
  }

  /** \brief Initialize decoding of color information
   * */
  void
  initializeDecoding ()
  {
    pointAvgColorDataVector_Iterator_ = pointAvgColorDataVector_.begin ();

    pointDiffColorDataVector_Iterator_ = pointDiffColorDataVector_.begin ();
  }

  /** \brief Get reference to vector containing averaged color data
   * */
  std::vector<char>&
  getAverageDataVector ()
  {
    return pointAvgColorDataVector_;
  }

  /** \brief Get reference to vector containing differential color data
   * */
  std::vector<char>&
  getDifferentialDataVector ()
  {
    return pointDiffColorDataVector_;
  }

  /** \brief Encode averaged color information for a subset of points from point cloud
   * \param indexVector_arg indices defining a subset of points from points cloud
   * \param rgba_offset_arg offset to color information
   * \param inputCloud_arg input point cloud
   * */
  void
  encodeAverageOfPoints (const Indices& indexVector_arg, unsigned char rgba_offset_arg, PointCloudConstPtr inputCloud_arg)
  {
    uindex_t avgRed = 0;
    uindex_t avgGreen = 0;
    uindex_t avgBlue = 0;

    // iterate over points
    for (const auto& idx: indexVector_arg)
    {
      // get color information from points
      const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[idx]);
      const int& colorInt = *reinterpret_cast<const int*> (idxPointPtr+rgba_offset_arg);

      // add color information
      avgRed += (colorInt >> 0) & 0xFF;
      avgGreen += (colorInt >> 8) & 0xFF;
      avgBlue += (colorInt >> 16) & 0xFF;

    }

    const uindex_t len = static_cast<uindex_t> (indexVector_arg.size());
    // calculated average color information
    if (len > 1)
    {
      avgRed   /= len;
      avgGreen /= len;
      avgBlue  /= len;
    }

    // remove least significant bits
    avgRed >>= colorBitReduction_;
    avgGreen >>= colorBitReduction_;
    avgBlue >>= colorBitReduction_;

    // add to average color vector
    pointAvgColorDataVector_.push_back (static_cast<char> (avgRed));
    pointAvgColorDataVector_.push_back (static_cast<char> (avgGreen));
    pointAvgColorDataVector_.push_back (static_cast<char> (avgBlue));
  }

  /** \brief Encode color information of a subset of points from point cloud
   * \param indexVector_arg indices defining a subset of points from points cloud
   * \param rgba_offset_arg offset to color information
   * \param inputCloud_arg input point cloud
   * */
  void
  encodePoints (const Indices& indexVector_arg, unsigned char rgba_offset_arg, PointCloudConstPtr inputCloud_arg)
  {
    uindex_t avgRed;
    uindex_t avgGreen;
    uindex_t avgBlue;

    // initialize
    avgRed = avgGreen = avgBlue = 0;

    // iterate over points
    for (const auto& idx: indexVector_arg)
    {
      // get color information from point
      const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[idx]);
      const int& colorInt = *reinterpret_cast<const int*> (idxPointPtr+rgba_offset_arg);

      // add color information
      avgRed += (colorInt >> 0) & 0xFF;
      avgGreen += (colorInt >> 8) & 0xFF;
      avgBlue += (colorInt >> 16) & 0xFF;

    }

    const uindex_t len = static_cast<uindex_t> (indexVector_arg.size());
    if (len > 1)
    {
      unsigned char diffRed;
      unsigned char diffGreen;
      unsigned char diffBlue;

      // calculated average color information
      avgRed   /= len;
      avgGreen /= len;
      avgBlue  /= len;

      // iterate over points for differential encoding
      for (const auto& idx: indexVector_arg)
      {
        const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[idx]);
        const int& colorInt = *reinterpret_cast<const int*> (idxPointPtr+rgba_offset_arg);

        // extract color components and do XOR encoding with predicted average color
        diffRed = (static_cast<unsigned char> (avgRed)) ^ static_cast<unsigned char> (((colorInt >> 0) & 0xFF));
        diffGreen = (static_cast<unsigned char> (avgGreen)) ^ static_cast<unsigned char> (((colorInt >> 8) & 0xFF));
        diffBlue = (static_cast<unsigned char> (avgBlue)) ^ static_cast<unsigned char> (((colorInt >> 16) & 0xFF));

        // remove least significant bits
        diffRed = static_cast<unsigned char> (diffRed >> colorBitReduction_);
        diffGreen = static_cast<unsigned char> (diffGreen >> colorBitReduction_);
        diffBlue = static_cast<unsigned char> (diffBlue >> colorBitReduction_);

        // add to differential color vector
        pointDiffColorDataVector_.push_back (static_cast<char> (diffRed));
        pointDiffColorDataVector_.push_back (static_cast<char> (diffGreen));
        pointDiffColorDataVector_.push_back (static_cast<char> (diffBlue));
      }
    }

    // remove least significant bits from average color information
    avgRed   >>= colorBitReduction_;
    avgGreen >>= colorBitReduction_;
    avgBlue  >>= colorBitReduction_;

    // add to differential color vector
    pointAvgColorDataVector_.push_back (static_cast<char> (avgRed));
    pointAvgColorDataVector_.push_back (static_cast<char> (avgGreen));
    pointAvgColorDataVector_.push_back (static_cast<char> (avgBlue));

  }

  /** \brief Decode color information
    * \param outputCloud_arg output point cloud
    * \param beginIdx_arg index indicating first point to be assigned with color information
    * \param endIdx_arg index indicating last point to be assigned with color information
    * \param rgba_offset_arg offset to color information
    */
  void
  decodePoints (PointCloudPtr outputCloud_arg, uindex_t beginIdx_arg, uindex_t endIdx_arg, unsigned char rgba_offset_arg)
  {
    assert (beginIdx_arg <= endIdx_arg);

    // amount of points to be decoded
    const index_t pointCount = endIdx_arg - beginIdx_arg;

    // get averaged color information for current voxel
    unsigned char avgRed = *(pointAvgColorDataVector_Iterator_++);
    unsigned char avgGreen = *(pointAvgColorDataVector_Iterator_++);
    unsigned char avgBlue = *(pointAvgColorDataVector_Iterator_++);

    // invert bit shifts during encoding
    avgRed = static_cast<unsigned char> (avgRed << colorBitReduction_);
    avgGreen = static_cast<unsigned char> (avgGreen << colorBitReduction_);
    avgBlue = static_cast<unsigned char> (avgBlue << colorBitReduction_);

    // iterate over points
    for (index_t i = 0; i < pointCount; i++)
    {
      unsigned int colorInt;
      if (pointCount > 1)
      {
        // get differential color information from input vector
        unsigned char diffRed   = static_cast<unsigned char> (*(pointDiffColorDataVector_Iterator_++));
        unsigned char diffGreen = static_cast<unsigned char> (*(pointDiffColorDataVector_Iterator_++));
        unsigned char diffBlue  = static_cast<unsigned char> (*(pointDiffColorDataVector_Iterator_++));

        // invert bit shifts during encoding
        diffRed = static_cast<unsigned char> (diffRed << colorBitReduction_);
        diffGreen = static_cast<unsigned char> (diffGreen << colorBitReduction_);
        diffBlue = static_cast<unsigned char> (diffBlue << colorBitReduction_);

        // decode color information
        colorInt = ((avgRed ^ diffRed) << 0) |
                   ((avgGreen ^ diffGreen) << 8) |
                   ((avgBlue ^ diffBlue) << 16);
      }
      else
      {
        // decode color information
        colorInt = (avgRed << 0) | (avgGreen << 8) | (avgBlue << 16);
      }

      char* idxPointPtr = reinterpret_cast<char*> (&(*outputCloud_arg)[beginIdx_arg + i]);
      int& pointColor = *reinterpret_cast<int*> (idxPointPtr+rgba_offset_arg);
      // assign color to point from point cloud
      pointColor=colorInt;
    }
  }

  /** \brief Set default color to points
   * \param outputCloud_arg output point cloud
   * \param beginIdx_arg index indicating first point to be assigned with color information
   * \param endIdx_arg index indicating last point to be assigned with color information
   * \param rgba_offset_arg offset to color information
   * */
  void
  setDefaultColor (PointCloudPtr outputCloud_arg, std::size_t beginIdx_arg, std::size_t endIdx_arg, unsigned char rgba_offset_arg)
  {
    assert (beginIdx_arg <= endIdx_arg);

    // amount of points to be decoded
    unsigned int pointCount = static_cast<unsigned int> (endIdx_arg - beginIdx_arg);

    // iterate over points
    for (std::size_t i = 0; i < pointCount; i++)
    {
      char* idxPointPtr = reinterpret_cast<char*> (&(*outputCloud_arg)[beginIdx_arg + i]);
      int& pointColor = *reinterpret_cast<int*> (idxPointPtr+rgba_offset_arg);
      // assign color to point from point cloud
      pointColor = defaultColor_;
    }
  }

protected:
  /** \brief Pointer to output point cloud dataset. */
  PointCloudPtr output_;

  /** \brief Vector for storing average color information  */
  std::vector<char> pointAvgColorDataVector_;

  /** \brief Iterator on average color information vector */
  std::vector<char>::const_iterator pointAvgColorDataVector_Iterator_;

  /** \brief Vector for storing differential color information  */
  std::vector<char> pointDiffColorDataVector_;

  /** \brief Iterator on differential color information vector */
  std::vector<char>::const_iterator pointDiffColorDataVector_Iterator_;

  /** \brief Amount of bits to be removed from color components before encoding */
  unsigned char colorBitReduction_;

  // frame header identifier
  static const int defaultColor_;
};

// define default color
template<typename PointT>
const int ColorCoding<PointT>::defaultColor_ = ((255) << 0) |
                                               ((255) << 8) |
                                               ((255) << 16);

} // namespace octree
} // namespace pcl

#define PCL_INSTANTIATE_ColorCoding(T) template class PCL_EXPORTS pcl::octree::ColorCoding<T>;

