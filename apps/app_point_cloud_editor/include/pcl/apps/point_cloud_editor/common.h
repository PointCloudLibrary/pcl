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

/// @file common.h
/// @details The set of simple common operations used throughout the project.
/// @author Yue Li and Matthew Hielsberg

#ifndef COMMON_H_
#define COMMON_H_

#include <sstream>

/// @brief Sets an array representing a 4x4 matrix to the identity
/// @param matrix A pointer to memory representing at least MATRIX_SIZE
/// elements
/// @pre Assumes the pointer is valid.
void
setIdentity(float* matrix);

/// @brief Performs result = left * right
/// @param left A pointer to memory representing at least MATRIX_SIZE
/// elements
/// @param right A pointer to memory representing at least MATRIX_SIZE
/// elements
/// @param result A pointer to memory representing at least MATRIX_SIZE
/// elements.  The output of left * right is stored in this matrix
/// @pre Assumes all pointers are valid.
void
multMatrix(const float* left, const float* right, float* result);

/// @brief Finds the inverse of a matrix
/// @param the input 4x4 column-major matrix following OpenGL's format
/// @param the output 4x4 column-major inverse matrix following OpenGL's format
bool
invertMatrix(const float* matrix, float* inverse);

/// @brief Helper function for converting objects to strings (using operator<<)
/// @param input The object to be converted
/// @param result A reference to the string where the resulting string will be
/// stored.
template<typename T>
void
toString(T input, std::string &result)
{
  std::stringstream ss;
  ss << input;
  result = ss.str();
}

/// @brief Converts the passed string to lowercase in place
/// @param s The string to be made lower.
void
stringToLower(std::string &s);

#endif // COMMON_H_
