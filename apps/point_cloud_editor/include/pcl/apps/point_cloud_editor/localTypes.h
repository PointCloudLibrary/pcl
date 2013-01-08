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


#ifndef LOCAL_TYPES_H_
#define LOCAL_TYPES_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Forward declaration for commonly used objects
class Command;
class CommandQueue;
class Selection;
class CopyBuffer;
class Cloud;

// Some helpful typedef's for commonly used objects

/// The type for the 3D points in the point cloud.
typedef pcl::PointXYZRGBA Point3D;

/// The type used as internal representation of a cloud object.
typedef pcl::PointCloud<Point3D> Cloud3D;

/// The type for boost shared pointer pointing to a PCL cloud object.
typedef Cloud3D::Ptr PclCloudPtr;

/// The type for boost shared pointer pointing to a cloud object
typedef boost::shared_ptr<Cloud> CloudPtr;

/// The type for boost shared pointer pointing to a constant cloud
/// object
typedef boost::shared_ptr<const Cloud> ConstCloudPtr;

/// The type for boost shared pointer pointing to a selection buffer
typedef boost::shared_ptr<Selection> SelectionPtr;

/// The type for boost shared pointer pointing to a constant selection
/// buffer
typedef boost::shared_ptr<const Selection> ConstSelectionPtr;

/// The type for boost shared pointer pointing to a copy buffer
typedef boost::shared_ptr<CopyBuffer> CopyBufferPtr;

/// The type for boost shared pointer pointing to a constant copy
/// buffer
typedef boost::shared_ptr<const CopyBuffer> ConstCopyBufferPtr;

/// The type for boost shared pointer pointing to a command object
typedef boost::shared_ptr<Command> CommandPtr;

/// The type used for vectors holding the indices of points in a cloud
typedef std::vector<unsigned int> IndexVector;

/// The type used for vectors holding the constant indices of points in
/// a cloud
typedef std::vector<const int> ConstIndexVector;

/// The type for boost shared pointer pointing to a command queue
/// object
typedef boost::shared_ptr<CommandQueue> CommandQueuePtr;

/// The type for bit masks used for recognizing key pressed by user.
typedef unsigned int BitMask;

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



#endif // LOCAL_TYPES_H_
