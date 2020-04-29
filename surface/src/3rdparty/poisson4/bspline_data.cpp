/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, OpenPerception
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
 */

#include <pcl/surface/3rdparty/poisson4/bspline_data.h>

namespace pcl {
namespace poisson {
template <>
void
BSplineElements<1>::upSample(BSplineElements<1>& high) const
{
  high.resize(size() * 2);
  high.assign(high.size(), BSplineElementCoefficients<1>());
  for (int i = 0; i < int(size()); i++) {
    high[2 * i + 0][0] += 1 * (*this)[i][0];
    high[2 * i + 0][1] += 0 * (*this)[i][0];
    high[2 * i + 1][0] += 2 * (*this)[i][0];
    high[2 * i + 1][1] += 1 * (*this)[i][0];

    high[2 * i + 0][0] += 1 * (*this)[i][1];
    high[2 * i + 0][1] += 2 * (*this)[i][1];
    high[2 * i + 1][0] += 0 * (*this)[i][1];
    high[2 * i + 1][1] += 1 * (*this)[i][1];
  }
  high.denominator = denominator * 2;
}

template <>
void
BSplineElements<2>::upSample(BSplineElements<2>& high) const
{
  /*    /----\
   *   /      \
   *  /        \  = 1  /--\       +3    /--\     +3      /--\   +1        /--\
   * /          \     /    \           /    \           /    \           /    \
   * |----------|     |----------|   |----------|   |----------|   |----------|
   */

  high.resize(size() * 2);
  high.assign(high.size(), BSplineElementCoefficients<2>());
  for (int i = 0; i < int(size()); i++) {
    high[2 * i + 0][0] += 1 * (*this)[i][0];
    high[2 * i + 0][1] += 0 * (*this)[i][0];
    high[2 * i + 0][2] += 0 * (*this)[i][0];
    high[2 * i + 1][0] += 3 * (*this)[i][0];
    high[2 * i + 1][1] += 1 * (*this)[i][0];
    high[2 * i + 1][2] += 0 * (*this)[i][0];

    high[2 * i + 0][0] += 3 * (*this)[i][1];
    high[2 * i + 0][1] += 3 * (*this)[i][1];
    high[2 * i + 0][2] += 1 * (*this)[i][1];
    high[2 * i + 1][0] += 1 * (*this)[i][1];
    high[2 * i + 1][1] += 3 * (*this)[i][1];
    high[2 * i + 1][2] += 3 * (*this)[i][1];

    high[2 * i + 0][0] += 0 * (*this)[i][2];
    high[2 * i + 0][1] += 1 * (*this)[i][2];
    high[2 * i + 0][2] += 3 * (*this)[i][2];
    high[2 * i + 1][0] += 0 * (*this)[i][2];
    high[2 * i + 1][1] += 0 * (*this)[i][2];
    high[2 * i + 1][2] += 1 * (*this)[i][2];
  }
  high.denominator = denominator * 4;
}
} // namespace poisson
} // namespace pcl
