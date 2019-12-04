///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///

/// @file   localTypes.h
/// @details A set of useful typedefs, forward declarations and constants.
/// @author  Yue Li and Matthew Hielsberg

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>
#include <memory>

// Forward declaration for commonly used objects
class Command;
class CommandQueue;
class Selection;
class CopyBuffer;
class Cloud;

// Some helpful typedef's for commonly used objects

/// The type for the 3D points in the point cloud.
using Point3D = pcl::PointXYZRGBA;

/// The type used as internal representation of a cloud object.
using Cloud3D = pcl::PointCloud<Point3D>;

/// The type for the 3D point vector in the point cloud.
using Point3DVector = pcl::PointCloud<Point3D>::VectorType;

/// The type for std shared pointer pointing to a PCL cloud object.
using PclCloudPtr = Cloud3D::Ptr;

/// The type for std shared pointer pointing to a cloud object
using CloudPtr = std::shared_ptr<Cloud>;

/// The type for std shared pointer pointing to a constant cloud
/// object
using ConstCloudPtr = std::shared_ptr<const Cloud>;

/// The type for std shared pointer pointing to a selection buffer
using SelectionPtr = std::shared_ptr<Selection>;

/// The type for std shared pointer pointing to a constant selection
/// buffer
using ConstSelectionPtr = std::shared_ptr<const Selection>;

/// The type for std shared pointer pointing to a copy buffer
using CopyBufferPtr = std::shared_ptr<CopyBuffer>;

/// The type for std shared pointer pointing to a constant copy
/// buffer
using ConstCopyBufferPtr = std::shared_ptr<const CopyBuffer>;

/// The type for std shared pointer pointing to a command object
using CommandPtr = std::shared_ptr<Command>;

/// The type used for vectors holding the indices of points in a cloud
using IndexVector = std::vector<unsigned int>;

/// The type used for vectors holding the constant indices of points in
/// a cloud
using ConstIndexVector = std::vector<const int>;

/// The type for std shared pointer pointing to a command queue
/// object
using CommandQueuePtr = std::shared_ptr<CommandQueue>;

/// The type for bit masks used for recognizing key pressed by user.
using BitMask = unsigned int;

/// ID's for the key modifiers.
enum KeyModifier
{
  NONE = 0x00000000,
  SHFT = 0x02000000,
  CTRL = 0x04000000,
  ALT  = 0x08000000
};

/// ID's for the mouse buttons.
enum MouseButton
{
  NOBUTTON,
  LEFT,
  RIGHT
};

/// Indices for the coordinate axes
/// It is assumed that the ColorScheme X,Y,Z match these values
enum Axis
{
  X,
  Y,
  Z
};

/// Indices for color components
enum Color
{
  RED,
  GREEN,
  BLUE,
  RGB
};

/// Scheme used for coloring the whole cloud.
/// It is assumed that the Axiz X,Y,Z match the COLOR_BY_[X,Y,Z] values
enum ColorScheme
{
  COLOR_BY_X = 0,
  COLOR_BY_Y,
  COLOR_BY_Z,
  COLOR_BY_RGB,
  COLOR_BY_PURE
};

/// Simple functor that produces sequential integers from an initial value
struct IncIndex
{
  unsigned int val_;
  IncIndex(int v=0)
  {
    val_ = v;
  }
  unsigned int operator()()
  {
    return (val_++);
  }
};

/// A helpful const representing the number of elements in our vectors.
/// This is used for all operations that require the copying of the vector.
/// Although this is used in a fairly generic way, the display requires 3D.
const unsigned int XYZ_SIZE = 3;

/// A helpful const representing the number of elements in each row/col in
/// our matrices. This is used for all operations that require the copying of
/// the matrix.
const unsigned int MATRIX_SIZE_DIM = 4;

/// A helpful const representing the number of elements in our matrices.
/// This is used for all operations that require the copying of the matrix.
const unsigned int MATRIX_SIZE = MATRIX_SIZE_DIM * MATRIX_SIZE_DIM;

/// The default window width
const unsigned int WINDOW_WIDTH = 1200;
/// The default window height
const unsigned int WINDOW_HEIGHT = 1000;

/// The default z translation used to push the world origin in front of the
/// display
const float DISPLAY_Z_TRANSLATION = -2.0f;

/// The radius of the trackball given as a percentage of the screen width
const float TRACKBALL_RADIUS_SCALE = 0.4f;
