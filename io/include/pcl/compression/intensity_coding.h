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

#include <pcl/point_cloud.h>
#include <pcl/types.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iterator>
#include <vector>

namespace pcl
{

namespace octree
{

/** \brief @b IntensityCoding class
 *  \note This class encodes float intensity information for octree-based point cloud compression.
 *  It mirrors ColorCoding: the average intensity of the points within a voxel is stored once per
 *  voxel, and the intensity of every point is stored as the bitwise XOR of its IEEE-754 bit
 *  pattern with the average intensity bit pattern. XOR coding is exactly invertible, so the
 *  intensity round-trips losslessly for any float value, regardless of its range. The entropy
 *  range coder compresses the resulting byte stream well when intensities within a voxel are
 *  similar, since the high bytes of the XOR differences are then zero.
 *  \note
 *  \note typename: PointT: type of point used in pointcloud
 */
template<typename PointT>
class IntensityCoding
{
  // public typedefs
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

public:
  /** \brief Constructor.
   *
   * */
  IntensityCoding () = default;

  /** \brief Empty class constructor. */
  virtual
  ~IntensityCoding () = default;

  /** \brief Set amount of voxels containing point intensity information and reserve memory
    * \param voxelCount_arg: amounts of voxels
    */
  inline void
  setVoxelCount (unsigned int voxelCount_arg)
  {
    pointAvgIntensityDataVector_.reserve (voxelCount_arg * 4);
  }

  /** \brief Set amount of points within point cloud to be encoded and reserve memory
   * \param pointCount_arg: amounts of points within point cloud
   * */
  inline
  void
  setPointCount (unsigned int pointCount_arg)
  {
    pointDiffIntensityDataVector_.reserve (pointCount_arg * 4);
  }

  /** \brief Initialize encoding of intensity information
   * */
  void
  initializeEncoding ()
  {
    pointAvgIntensityDataVector_.clear ();

    pointDiffIntensityDataVector_.clear ();
  }

  /** \brief Initialize decoding of intensity information
   * */
  void
  initializeDecoding ()
  {
    pointAvgIntensityDataVector_Iterator_ = pointAvgIntensityDataVector_.begin ();

    pointDiffIntensityDataVector_Iterator_ = pointDiffIntensityDataVector_.begin ();

    // A truncated frame must not poison the next frame.
    decoding_failed_ = false;
  }

  /** \brief Get reference to vector containing averaged intensity data
   * */
  std::vector<char>&
  getAverageDataVector ()
  {
    return pointAvgIntensityDataVector_;
  }

  /** \brief Get reference to vector containing differential intensity data
   * */
  std::vector<char>&
  getDifferentialDataVector ()
  {
    return pointDiffIntensityDataVector_;
  }

  /** \brief Encode averaged intensity information for a subset of points from point cloud
   * \param indexVector_arg indices defining a subset of points from points cloud
   * \param intensity_offset_arg offset to intensity information
   * \param inputCloud_arg input point cloud
   * */
  void
  encodeAverageOfPoints (const Indices& indexVector_arg, std::size_t intensity_offset_arg, PointCloudConstPtr inputCloud_arg)
  {
    double avgIntensity = 0.0;

    // iterate over points
    for (const auto& idx: indexVector_arg)
    {
      // get intensity information from point
      const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[idx]);
      const float& intensity = *reinterpret_cast<const float*> (idxPointPtr+intensity_offset_arg);

      // add intensity information
      avgIntensity += static_cast<double> (intensity);
    }

    const auto len = static_cast<uindex_t> (indexVector_arg.size());
    // calculate average intensity information
    if (len > 1)
    {
      avgIntensity /= len;
    }

    // add to average intensity vector
    if (len == 1)
    {
      // A single point must keep its exact bit pattern: accumulating it
      // from +0.0 would turn -0.0f into +0.0f, breaking the lossless
      // contract (no differential value is stored to restore the sign).
      const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[indexVector_arg[0]]);
      const float& intensity = *reinterpret_cast<const float*> (idxPointPtr + intensity_offset_arg);
      appendFloat (pointAvgIntensityDataVector_, intensity);
    }
    else
    {
      appendFloat (pointAvgIntensityDataVector_, static_cast<float> (avgIntensity));
    }
  }

  /** \brief Encode intensity information of a subset of points from point cloud
   * \param indexVector_arg indices defining a subset of points from points cloud
   * \param intensity_offset_arg offset to intensity information
   * \param inputCloud_arg input point cloud
   * */
  void
  encodePoints (const Indices& indexVector_arg, std::size_t intensity_offset_arg, PointCloudConstPtr inputCloud_arg)
  {
    double avgIntensity = 0.0;

    // iterate over points
    for (const auto& idx: indexVector_arg)
    {
      // get intensity information from point
      const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[idx]);
      const float& intensity = *reinterpret_cast<const float*> (idxPointPtr+intensity_offset_arg);

      // add intensity information
      avgIntensity += static_cast<double> (intensity);
    }

    const auto len = static_cast<uindex_t> (indexVector_arg.size());
    if (len > 1)
    {
      // calculate average intensity information
      avgIntensity /= len;

      const std::uint32_t avgBits = floatToBits (static_cast<float> (avgIntensity));

      // iterate over points for differential encoding
      for (const auto& idx: indexVector_arg)
      {
        const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[idx]);
        const float& intensity = *reinterpret_cast<const float*> (idxPointPtr+intensity_offset_arg);

        // extract intensity and do XOR encoding with predicted average intensity
        const std::uint32_t diffBits = avgBits ^ floatToBits (intensity);

        // add to differential intensity vector
        appendBits (pointDiffIntensityDataVector_, diffBits);
      }
    }

    // add to average intensity vector
    if (len == 1)
    {
      // A single point must keep its exact bit pattern: accumulating it
      // from +0.0 would turn -0.0f into +0.0f, and no differential value
      // is stored to restore the sign.
      const char* idxPointPtr = reinterpret_cast<const char*> (&(*inputCloud_arg)[indexVector_arg[0]]);
      const float& intensity = *reinterpret_cast<const float*> (idxPointPtr + intensity_offset_arg);
      appendFloat (pointAvgIntensityDataVector_, intensity);
    }
    else
    {
      appendFloat (pointAvgIntensityDataVector_, static_cast<float> (avgIntensity));
    }
  }

  /** \brief Decode intensity information
    * \param outputCloud_arg output point cloud
    * \param beginIdx_arg index indicating first point to be assigned with intensity information
    * \param endIdx_arg index indicating last point to be assigned with intensity information
    * \param intensity_offset_arg offset to intensity information
    */
  void
  decodePoints (PointCloudPtr outputCloud_arg, uindex_t beginIdx_arg, uindex_t endIdx_arg, std::size_t intensity_offset_arg)
  {
    assert (beginIdx_arg <= endIdx_arg);

    // amount of points to be decoded
    const index_t pointCount = endIdx_arg - beginIdx_arg;

    // Reject a truncated frame instead of reading past the end of the
    // intensity buffers: 4 bytes per voxel from the average buffer and,
    // for multi-point voxels, 4 bytes per point from the differential
    // buffer. Undersized buffers leave the default intensity in place.
    // Once a frame's intensity stream is truncated, later voxels in the
    // same frame would read misaligned leftover bytes, so the failure
    // sticks for the rest of the frame: every remaining voxel gets
    // defaults without either iterator being read. initializeDecoding
    // clears the flag so the next frame decodes normally.
    if (decoding_failed_ ||
        (std::distance (pointAvgIntensityDataVector_Iterator_, pointAvgIntensityDataVector_.cend ()) < 4) ||
        ((pointCount > 1) &&
         (std::distance (pointDiffIntensityDataVector_Iterator_, pointDiffIntensityDataVector_.cend ()) < 4 * pointCount)))
    {
      decoding_failed_ = true;
      for (index_t i = 0; i < pointCount; i++)
      {
        char* idxPointPtr = reinterpret_cast<char*> (&(*outputCloud_arg)[beginIdx_arg + i]);
        float& pointIntensity = *reinterpret_cast<float*> (idxPointPtr+intensity_offset_arg);
        pointIntensity = defaultIntensity_;
      }
      return;
    }

    // get averaged intensity information for current voxel
    const std::uint32_t avgBits = readBits (pointAvgIntensityDataVector_Iterator_);

    // iterate over points
    for (index_t i = 0; i < pointCount; i++)
    {
      std::uint32_t intensityBits;
      if (pointCount > 1)
      {
        // get differential intensity information from input vector
        const std::uint32_t diffBits = readBits (pointDiffIntensityDataVector_Iterator_);

        // decode intensity information
        intensityBits = avgBits ^ diffBits;
      }
      else
      {
        // decode intensity information
        intensityBits = avgBits;
      }

      char* idxPointPtr = reinterpret_cast<char*> (&(*outputCloud_arg)[beginIdx_arg + i]);
      float& pointIntensity = *reinterpret_cast<float*> (idxPointPtr+intensity_offset_arg);
      // assign intensity to point from point cloud
      pointIntensity = bitsToFloat (intensityBits);
    }
  }

  /** \brief Set default intensity to points
   * \param outputCloud_arg output point cloud
   * \param beginIdx_arg index indicating first point to be assigned with intensity information
   * \param endIdx_arg index indicating last point to be assigned with intensity information
   * \param intensity_offset_arg offset to intensity information
   * */
  void
  setDefaultIntensity (PointCloudPtr outputCloud_arg, std::size_t beginIdx_arg, std::size_t endIdx_arg, std::size_t intensity_offset_arg)
  {
    assert (beginIdx_arg <= endIdx_arg);

    // amount of points to be decoded
    auto pointCount = static_cast<unsigned int> (endIdx_arg - beginIdx_arg);

    // iterate over points
    for (std::size_t i = 0; i < pointCount; i++)
    {
      char* idxPointPtr = reinterpret_cast<char*> (&(*outputCloud_arg)[beginIdx_arg + i]);
      float& pointIntensity = *reinterpret_cast<float*> (idxPointPtr+intensity_offset_arg);
      // assign intensity to point from point cloud
      pointIntensity = defaultIntensity_;
    }
  }

protected:
  /** \brief Convert a float to its IEEE-754 bit pattern. */
  static std::uint32_t
  floatToBits (float value)
  {
    std::uint32_t bits;
    std::memcpy (&bits, &value, sizeof (bits));
    return bits;
  }

  /** \brief Convert an IEEE-754 bit pattern back to a float. */
  static float
  bitsToFloat (std::uint32_t bits)
  {
    float value;
    std::memcpy (&value, &bits, sizeof (value));
    return value;
  }

  /** \brief Append a float bit pattern to a data vector in little-endian byte order. */
  static void
  appendBits (std::vector<char>& dataVector_arg, std::uint32_t bits)
  {
    dataVector_arg.push_back (static_cast<char> ((bits >> 0) & 0xFF));
    dataVector_arg.push_back (static_cast<char> ((bits >> 8) & 0xFF));
    dataVector_arg.push_back (static_cast<char> ((bits >> 16) & 0xFF));
    dataVector_arg.push_back (static_cast<char> ((bits >> 24) & 0xFF));
  }

  /** \brief Append a float to a data vector in little-endian byte order. */
  static void
  appendFloat (std::vector<char>& dataVector_arg, float value)
  {
    appendBits (dataVector_arg, floatToBits (value));
  }

  /** \brief Read a float bit pattern (little-endian byte order) from a data vector iterator. */
  static std::uint32_t
  readBits (std::vector<char>::const_iterator& iterator_arg)
  {
    // Read each byte in its own statement: the four increments in a
    // single expression are unsequenced, so evaluation order could
    // assign bytes to the wrong shifts.
    const std::uint32_t byte0 =
        static_cast<std::uint32_t> (static_cast<unsigned char> (*(iterator_arg++)));
    const std::uint32_t byte1 =
        static_cast<std::uint32_t> (static_cast<unsigned char> (*(iterator_arg++)));
    const std::uint32_t byte2 =
        static_cast<std::uint32_t> (static_cast<unsigned char> (*(iterator_arg++)));
    const std::uint32_t byte3 =
        static_cast<std::uint32_t> (static_cast<unsigned char> (*(iterator_arg++)));

    return (byte0 << 0) | (byte1 << 8) | (byte2 << 16) | (byte3 << 24);
  }

  /** \brief Pointer to output point cloud dataset. */
  PointCloudPtr output_{nullptr};

  /** \brief Vector for storing average intensity information  */
  std::vector<char> pointAvgIntensityDataVector_;

  /** \brief Iterator on average intensity information vector */
  std::vector<char>::const_iterator pointAvgIntensityDataVector_Iterator_;

  /** \brief Vector for storing differential intensity information  */
  std::vector<char> pointDiffIntensityDataVector_;

  /** \brief Iterator on differential intensity information vector */
  std::vector<char>::const_iterator pointDiffIntensityDataVector_Iterator_;

  /** \brief True once this frame's intensity stream was found truncated; all
    * remaining voxels in the frame decode to the default intensity without
    * reading either buffer. Cleared by initializeDecoding. */
  bool decoding_failed_{false};

  /** \brief Default intensity assigned when the stream carries no intensity data. */
  static const float defaultIntensity_;
};

// define default intensity
template<typename PointT>
const float IntensityCoding<PointT>::defaultIntensity_ = 0.0f;

} // namespace octree
} // namespace pcl

#define PCL_INSTANTIATE_IntensityCoding(T) template class PCL_EXPORTS pcl::octree::IntensityCoding<T>;
