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

#ifndef PCL_IN_HAND_SCANNER_INTEGRATION_H
#define PCL_IN_HAND_SCANNER_INTEGRATION_H

#include <pcl/apps/in_hand_scanner/common_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <typename PointT>
  class KdTree;
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// Integration
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {

    class Integration
    {

      public:

        typedef pcl::ihs::PointProcessed         PointProcessed;
        typedef pcl::ihs::CloudProcessed         CloudProcessed;
        typedef pcl::ihs::CloudProcessedPtr      CloudProcessedPtr;
        typedef pcl::ihs::CloudProcessedConstPtr CloudProcessedConstPtr;

        typedef pcl::ihs::PointModel         PointModel;
        typedef pcl::ihs::CloudModel         CloudModel;
        typedef pcl::ihs::CloudModelPtr      CloudModelPtr;
        typedef pcl::ihs::CloudModelConstPtr CloudModelConstPtr;

        typedef pcl::ihs::Transformation         Transformation;

        typedef pcl::KdTree <PointModel>         KdTree;
        typedef boost::shared_ptr <KdTree>       KdTreePtr;
        typedef boost::shared_ptr <const KdTree> KdTreeConstPtr;

      public:

        Integration ();

        void
        setInitialModel (const CloudModelPtr&          cloud_model,
                         const CloudProcessedConstPtr& cloud_data) const;

        bool
        merge (const CloudModelPtr&          cloud_model,
               const CloudProcessedConstPtr& cloud_data,
               const Transformation&         T) const;

      private:

        const uint8_t
        rgbTrimming (const float val) const
        { return (static_cast <uint8_t> (val > 255.f ? 255 : val)); }

        // Nearest neighbor search
        KdTreePtr kd_tree_;

        // Maximum squared distance below which points are averaged out
        float     squared_distance_max_;

        // Minium dot product between normals above which points are averaged out
        float     dot_normal_min_;
    };

  } // End namespace ihs
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_INTEGRATION_H
