/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2017-, Southwest Research Institute
 * Copyright (c) 2017-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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
#ifndef PCL_COMMON_NORMALS_H_
#define PCL_COMMON_NORMALS_H_

/**
  * \file pcl/common/normals.h
  * Define methods related to normals
  * \ingroup common
  */
#include <pcl/point_types.h>

namespace pcl
{
  /** \brief Align pn's normal with av so they point in the same direction */
  bool
  alignNormals (Eigen::Ref<Eigen::Vector3f> pn, const Eigen::Ref<const Eigen::Vector3f> &av);

  /** \brief Check if two normals are within a tolerance
    * \param[in] n1 First normal.
    * \param[in] n2 Second normal.
    * \param[in] angle_threshold The angle threshold in radians.
    */
  bool
  checkNormalsEqual (const Eigen::Vector3f &n1, const Eigen::Vector3f &n2, const double &angle_threshold);


}

#include <pcl/common/impl/normals.hpp>

#endif  // PCL_COMMON_NORMALS_H_
