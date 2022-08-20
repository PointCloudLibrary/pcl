/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/geometry/triangle_mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>

namespace pcl {
namespace ihs {
struct PointIHS;
using CloudIHS = pcl::PointCloud<PointIHS>;
using CloudIHSPtr = CloudIHS::Ptr;
using CloudIHSConstPtr = CloudIHS::ConstPtr;
} // End namespace ihs
} // End namespace pcl

#include <pcl/apps/in_hand_scanner/impl/common_types.hpp>

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::ihs::_PointIHS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, normal_x, normal_x)
                                  (float, normal_y, normal_y)
                                  (float, normal_z, normal_z)
                                  (float, rgb, rgb)
                                  (float, weight, weight)
                                  (unsigned int, age, age)
                                  (std::uint32_t, directions, directions)
                                 )
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::ihs::PointIHS, pcl::ihs::_PointIHS)
// clang-format on

namespace pcl {
namespace ihs {
struct MeshTraits {
  using VertexData = PointIHS;
  using HalfEdgeData = pcl::geometry::NoData;
  using EdgeData = pcl::geometry::NoData;
  using FaceData = pcl::geometry::NoData;
  using IsManifold = std::true_type;
};

// NOTE: The drawMesh method in pcl::ihs::InHandScanner only supports triangles!
using Mesh = pcl::geometry::TriangleMesh<MeshTraits>;
using MeshPtr = Mesh::Ptr;
using MeshConstPtr = Mesh::ConstPtr;
} // End namespace ihs
} // End namespace pcl
