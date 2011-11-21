/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimationPointToPlane uses Levenberg Marquardt optimization to find the
      * transformation that minimizes the point-to-plane distance between the given correspondences.
      *
      * \author Michael Dixon
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget>
    class TransformationEstimationPointToPlane : public TransformationEstimationLM<PointSource, PointTarget>
    {
      public:
        typedef boost::shared_ptr<TransformationEstimationPointToPlane<PointSource, PointTarget> > Ptr;
        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef PointIndices::Ptr PointIndicesPtr;
        typedef PointIndices::ConstPtr PointIndicesConstPtr;


        TransformationEstimationPointToPlane () {};
        virtual ~TransformationEstimationPointToPlane () {};

      protected:
        virtual double
        computeDistance (const PointSource &p_src, const PointTarget &p_tgt)
        { 
          // Compute the point-to-plane distance
          Vector4fMapConst s = p_src.getVector4fMap ();
          Vector4fMapConst t = p_tgt.getVector4fMap ();
          Vector4fMapConst n = p_tgt.getNormalVector4fMap ();
          return ((s - t).dot (n));
        }

    };
  }
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_PLANE_H_ */

