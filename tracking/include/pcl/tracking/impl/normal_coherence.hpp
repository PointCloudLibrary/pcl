/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#ifndef PCL_TRACKING_IMPL_NORMAL_COHERENCE_H_
#define PCL_TRACKING_IMPL_NORMAL_COHERENCE_H_

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/tracking/normal_coherence.h>

template <typename PointInT> double 
pcl::tracking::NormalCoherence<PointInT>::computeCoherence (PointInT &source, PointInT &target)
{
    Eigen::Vector4f n = source.getNormalVector4fMap ();
    Eigen::Vector4f n_dash = target.getNormalVector4fMap ();
    if ( n.norm () <= 1e-5 || n_dash.norm () <= 1e-5 )
    {
        PCL_ERROR("norm might be ZERO!\n");
        std::cout << "source: " << source << std::endl;
        std::cout << "target: " << target << std::endl;
        exit (1);
        return 0.0;
    }
    else
    {
        n.normalize ();
        n_dash.normalize ();
        double theta = pcl::getAngle3D (n, n_dash);
        if (!pcl_isnan (theta))
            return 1.0 / (1.0 + weight_ * theta * theta);
        else
            return 0.0;
    }
}


#define PCL_INSTANTIATE_NormalCoherence(T) template class PCL_EXPORTS pcl::tracking::NormalCoherence<T>;

#endif
