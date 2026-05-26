/*
 * SPDX-License-Identifier: BSD-3-Clause
 *  *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2026-, Open Perception Inc.
 *  *  All rights reserved
 */

#pragma once

namespace pcl
{
  namespace detail
  {
    namespace traits
    {
      template<typename FeaturePointT> struct descriptorSize {};


      template<typename FeaturePointT>
      static constexpr int descriptorSize_v = descriptorSize<FeaturePointT>::value;
    }
  }
}
