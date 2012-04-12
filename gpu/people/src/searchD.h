/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 * $Id: $
 * @author: Anatoly Baksheev
 */

#ifndef PCL_GPU_PEOPLE_SEARCHD_H_
#define PCL_GPU_PEOPLE_SEARCHD_H_

#include <opencv2/core/core.hpp>

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/time.h>

#include <pcl/point_types_conversion.h> // can't include because not inline function causes multiple definition errors
//namespace pcl
//{
//   void PointXYZRGBtoXYZHSV (PointXYZRGB& in, PointXYZHSV& out);
//}

#include <iostream>
#include <algorithm>
#include <numeric>

using namespace std;
using namespace pcl;
using namespace cv;

namespace
{
    class SearchD : public pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>
    {
    public:
        typedef  pcl::search::OrganizedNeighbor<pcl::PointXYZRGB> Base;

        using Base::getProjectedRadiusSearchBox;
        /** \brief the projection matrix. Either set by user or calculated by the first / each input cloud */
        using Base::projection_matrix_;
        /** \brief inveser of the left 3x3 projection matrix which is K * R (with K being the camera matrix and R the rotation matrix)*/
        using Base::KR_;
        /** \brief inveser of the left 3x3 projection matrix which is K * R (with K being the camera matrix and R the rotation matrix)*/
        using Base::KR_KRT_;    
    };


    template<typename PointT1, typename PointT2>
    double sqnorm(const PointT1& p1, const PointT2& p2)
    {
        return (p1.getVector3fMap () - p2.getVector3fMap ()).squaredNorm ();
    }

    template<typename It, typename Val>
    void yota(It beg, It end, Val seed)
    {
        for(It pos = beg; pos < end;)
            *pos++ = seed++;
    }
}
#endif // PCL_GPU_PEOPLE_SEARCHD_H_
