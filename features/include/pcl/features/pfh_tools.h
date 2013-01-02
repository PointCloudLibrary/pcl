/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_FEATURES_PFH_TOOLS_H_
#define PCL_FEATURES_PFH_TOOLS_H_

#if defined __GNUC__
#  pragma GCC system_header 
#endif

#include <pcl/pcl_exports.h>
#include <Eigen/Core>

namespace pcl
{
  /** \brief Compute the 4-tuple representation containing the three angles and one distance between two points
    * represented by Cartesian coordinates and normals.
    * \note For explanations about the features, please see the literature mentioned above (the order of the
    * features might be different).
    * \param[in] p1 the first XYZ point
    * \param[in] n1 the first surface normal
    * \param[in] p2 the second XYZ point
    * \param[in] n2 the second surface normal
    * \param[out] f1 the first angular feature (angle between the projection of nq_idx and u)
    * \param[out] f2 the second angular feature (angle between nq_idx and v)
    * \param[out] f3 the third angular feature (angle between np_idx and |p_idx - q_idx|)
    * \param[out] f4 the distance feature (p_idx - q_idx)
    *
    * \note For efficiency reasons, we assume that the point data passed to the method is finite.
    * \ingroup features
    */
  PCL_EXPORTS bool 
  computePairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, 
                       const Eigen::Vector4f &p2, const Eigen::Vector4f &n2, 
                       float &f1, float &f2, float &f3, float &f4);

  PCL_EXPORTS bool
  computeRGBPairFeatures (const Eigen::Vector4f &p1, const Eigen::Vector4f &n1, const Eigen::Vector4i &colors1,
                          const Eigen::Vector4f &p2, const Eigen::Vector4f &n2, const Eigen::Vector4i &colors2,
                          float &f1, float &f2, float &f3, float &f4, float &f5, float &f6, float &f7);

}

#endif  //#ifndef PCL_FEATURES_PFH_TOOLS_H_

