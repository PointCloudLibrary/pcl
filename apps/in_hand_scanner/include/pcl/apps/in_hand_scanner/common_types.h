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

#ifndef PCL_IN_HAND_SCANNER_COMMON_TYPES_H
#define PCL_IN_HAND_SCANNER_COMMON_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/apps/in_hand_scanner/eigen.h>
#include <pcl/geometry/impl/triangle_mesh.hpp>

namespace pcl
{
  namespace ihs
  {

    typedef pcl::PointXYZRGBA            PointInput;
    typedef pcl::PointCloud <PointInput> CloudInput;
    typedef CloudInput::Ptr              CloudInputPtr;
    typedef CloudInput::ConstPtr         CloudInputConstPtr;

    typedef pcl::PointXYZRGBNormal           PointProcessed;
    typedef pcl::PointCloud <PointProcessed> CloudProcessed;
    typedef CloudProcessed::Ptr              CloudProcessedPtr;
    typedef CloudProcessed::ConstPtr         CloudProcessedConstPtr;

    struct PointModel;
    typedef pcl::PointCloud <PointModel> CloudModel;
    typedef CloudModel::Ptr              CloudModelPtr;
    typedef CloudModel::ConstPtr         CloudModelConstPtr;

    typedef Eigen::Matrix4f Transformation;

  } // End namespace ihs
} // End namespace pcl

#include <pcl/apps/in_hand_scanner/impl/common_types.hpp>

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ihs::PointModel,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
                                   (float, rgb, rgb)
                                   (float, weight, weight)
                                   (int, age, age)
                                   )

namespace pcl
{
  namespace ihs
  {
    typedef pcl::TriangleMesh <true, PointModel> Mesh;
    typedef boost::shared_ptr <Mesh>             MeshPtr;
    typedef boost::shared_ptr <const Mesh>       MeshConstPtr;
    typedef Mesh::Vertex                         Vertex;
    typedef Mesh::Face                           Face;
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_COMMON_TYPES_H
