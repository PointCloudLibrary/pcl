/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2025-, Open Perception, Inc.
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
 */

#pragma once

#include <Eigen/Core>

namespace pcl
{
  using Vector2fMap = Eigen::Map<Eigen::Vector2f>;
  using Vector2fMapConst = const Eigen::Map<const Eigen::Vector2f>;
  using Vector3fMap = Eigen::Map<Eigen::Vector3f>;
  using Vector3fMapConst = const Eigen::Map<const Eigen::Vector3f>;
  using Vector4fMap = Eigen::Map<Eigen::Vector4f, Eigen::Aligned>;
  using Vector4fMapConst =
      const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned>;

  using Vector3c = Eigen::Matrix<std::uint8_t, 3, 1>;
  using Vector3cMap = Eigen::Map<Vector3c>;
  using Vector3cMapConst = const Eigen::Map<const Vector3c>;
  using Vector4c = Eigen::Matrix<std::uint8_t, 4, 1>;
  using Vector4cMap = Eigen::Map<Vector4c, Eigen::Aligned>;
  using Vector4cMapConst = const Eigen::Map<const Vector4c, Eigen::Aligned>;
} // namespace pcl
