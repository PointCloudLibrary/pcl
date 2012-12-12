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

///
/// @file common.cpp
/// @details The set of simple common operations used throughout the project.
/// @author Yue Li and Matthew Hielsberg
///

#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

void
setIdentity(float* matrix)
{
  std::fill_n(matrix, MATRIX_SIZE, 0.0f);
  for (unsigned int i = 0; i < MATRIX_SIZE; i+=MATRIX_SIZE_DIM+1)
    matrix[i] = 1.0f;
}

void
multMatrix(const float* left, const float* right, float* result)
{
  float r[MATRIX_SIZE];
  for(unsigned int i = 0; i < MATRIX_SIZE_DIM; ++i)
  {
    for(unsigned int j = 0; j < MATRIX_SIZE_DIM; ++j)
    {
      float sum = 0.0;
      for(unsigned int k = 0; k < MATRIX_SIZE_DIM; ++k)
        sum += left[i * MATRIX_SIZE_DIM + k] * right[k * MATRIX_SIZE_DIM + j];
      r[i * MATRIX_SIZE_DIM + j] = sum;
    }
  }
  std::copy(r, r+MATRIX_SIZE, result);
}


// This code was found on:
// http://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
// and is listed as being part of an open soure project (the MESA project)
//
// The original code in MESA comes from __gluInvertMatrixd() in project.c
//
// The mesa license (can be found on http://www.mesa3d.org) is as follows:
//
// The Mesa distribution consists of several components. Different copyrights
// and licenses apply to different components. For example, some demo programs
// are copyrighted by SGI, some of the Mesa device drivers are copyrighted by
// their authors. See below for a list of Mesa's main components and the license
// for each.
//
//The core Mesa library is licensed according to the terms of the MIT license.
// This allows integration with the XFree86, Xorg and DRI projects.
//
//The default Mesa license is as follows:
//
//Copyright (C) 1999-2007  Brian Paul   All Rights Reserved.
//
//Permission is hereby granted, free of charge, to any person obtaining a
//copy of this software and associated documentation files (the "Software"),
//to deal in the Software without restriction, including without limitation
//the rights to use, copy, modify, merge, publish, distribute, sublicense,
//and/or sell copies of the Software, and to permit persons to whom the
//Software is furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included
//in all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
//BRIAN PAUL BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
//AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
bool invertMatrix(const float* matrix, float* inverse)
{
  double inv[16], det;

  inv[0] = matrix[5]  * matrix[10] * matrix[15] -
      matrix[5]  * matrix[11] * matrix[14] -
      matrix[9]  * matrix[6]  * matrix[15] +
      matrix[9]  * matrix[7]  * matrix[14] +
      matrix[13] * matrix[6]  * matrix[11] -
      matrix[13] * matrix[7]  * matrix[10];

  inv[4] = -matrix[4]  * matrix[10] * matrix[15] +
      matrix[4]  * matrix[11] * matrix[14] +
      matrix[8]  * matrix[6]  * matrix[15] -
      matrix[8]  * matrix[7]  * matrix[14] -
      matrix[12] * matrix[6]  * matrix[11] +
      matrix[12] * matrix[7]  * matrix[10];

  inv[8] = matrix[4]  * matrix[9] * matrix[15] -
      matrix[4]  * matrix[11] * matrix[13] -
      matrix[8]  * matrix[5] * matrix[15] +
      matrix[8]  * matrix[7] * matrix[13] +
      matrix[12] * matrix[5] * matrix[11] -
      matrix[12] * matrix[7] * matrix[9];

  inv[12] = -matrix[4]  * matrix[9] * matrix[14] +
      matrix[4]  * matrix[10] * matrix[13] +
      matrix[8]  * matrix[5] * matrix[14] -
      matrix[8]  * matrix[6] * matrix[13] -
      matrix[12] * matrix[5] * matrix[10] +
      matrix[12] * matrix[6] * matrix[9];

  inv[1] = -matrix[1]  * matrix[10] * matrix[15] +
      matrix[1]  * matrix[11] * matrix[14] +
      matrix[9]  * matrix[2] * matrix[15] -
      matrix[9]  * matrix[3] * matrix[14] -
      matrix[13] * matrix[2] * matrix[11] +
      matrix[13] * matrix[3] * matrix[10];

  inv[5] = matrix[0]  * matrix[10] * matrix[15] -
      matrix[0]  * matrix[11] * matrix[14] -
      matrix[8]  * matrix[2] * matrix[15] +
      matrix[8]  * matrix[3] * matrix[14] +
      matrix[12] * matrix[2] * matrix[11] -
      matrix[12] * matrix[3] * matrix[10];

  inv[9] = -matrix[0]  * matrix[9] * matrix[15] +
      matrix[0]  * matrix[11] * matrix[13] +
      matrix[8]  * matrix[1] * matrix[15] -
      matrix[8]  * matrix[3] * matrix[13] -
      matrix[12] * matrix[1] * matrix[11] +
      matrix[12] * matrix[3] * matrix[9];

  inv[13] = matrix[0]  * matrix[9] * matrix[14] -
      matrix[0]  * matrix[10] * matrix[13] -
      matrix[8]  * matrix[1] * matrix[14] +
      matrix[8]  * matrix[2] * matrix[13] +
      matrix[12] * matrix[1] * matrix[10] -
      matrix[12] * matrix[2] * matrix[9];

  inv[2] = matrix[1]  * matrix[6] * matrix[15] -
      matrix[1]  * matrix[7] * matrix[14] -
      matrix[5]  * matrix[2] * matrix[15] +
      matrix[5]  * matrix[3] * matrix[14] +
      matrix[13] * matrix[2] * matrix[7] -
      matrix[13] * matrix[3] * matrix[6];

  inv[6] = -matrix[0]  * matrix[6] * matrix[15] +
      matrix[0]  * matrix[7] * matrix[14] +
      matrix[4]  * matrix[2] * matrix[15] -
      matrix[4]  * matrix[3] * matrix[14] -
      matrix[12] * matrix[2] * matrix[7] +
      matrix[12] * matrix[3] * matrix[6];

  inv[10] = matrix[0]  * matrix[5] * matrix[15] -
      matrix[0]  * matrix[7] * matrix[13] -
      matrix[4]  * matrix[1] * matrix[15] +
      matrix[4]  * matrix[3] * matrix[13] +
      matrix[12] * matrix[1] * matrix[7] -
      matrix[12] * matrix[3] * matrix[5];

  inv[14] = -matrix[0]  * matrix[5] * matrix[14] +
      matrix[0]  * matrix[6] * matrix[13] +
      matrix[4]  * matrix[1] * matrix[14] -
      matrix[4]  * matrix[2] * matrix[13] -
      matrix[12] * matrix[1] * matrix[6] +
      matrix[12] * matrix[2] * matrix[5];

  inv[3] = -matrix[1] * matrix[6] * matrix[11] +
      matrix[1] * matrix[7] * matrix[10] +
      matrix[5] * matrix[2] * matrix[11] -
      matrix[5] * matrix[3] * matrix[10] -
      matrix[9] * matrix[2] * matrix[7] +
      matrix[9] * matrix[3] * matrix[6];

  inv[7] = matrix[0] * matrix[6] * matrix[11] -
      matrix[0] * matrix[7] * matrix[10] -
      matrix[4] * matrix[2] * matrix[11] +
      matrix[4] * matrix[3] * matrix[10] +
      matrix[8] * matrix[2] * matrix[7] -
      matrix[8] * matrix[3] * matrix[6];

  inv[11] = -matrix[0] * matrix[5] * matrix[11] +
      matrix[0] * matrix[7] * matrix[9] +
      matrix[4] * matrix[1] * matrix[11] -
      matrix[4] * matrix[3] * matrix[9] -
      matrix[8] * matrix[1] * matrix[7] +
      matrix[8] * matrix[3] * matrix[5];

  inv[15] = matrix[0] * matrix[5] * matrix[10] -
      matrix[0] * matrix[6] * matrix[9] -
      matrix[4] * matrix[1] * matrix[10] +
      matrix[4] * matrix[2] * matrix[9] +
      matrix[8] * matrix[1] * matrix[6] -
      matrix[8] * matrix[2] * matrix[5];

  det = matrix[0] * inv[0] + matrix[1] * inv[4] +
        matrix[2] * inv[8] + matrix[3] * inv[12];

  if (det == 0)
    return (false);

  det = 1.0 / det;

  for (unsigned int i = 0; i < MATRIX_SIZE; ++i)
    inverse[i] = inv[i] * det;

  return (true);
}

void
stringToLower(std::string &s)
{
  std::transform(s.begin(), s.end(), s.begin(), tolower);
}
