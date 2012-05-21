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
 * @author: Koen Buys
 * @file blob2.h
 * @brief This file contains the Blob2 structure and the inline <<-operator for it
 */

#ifndef PCL_GPU_PEOPLE_LABEL_BLOB2_H_
#define PCL_GPU_PEOPLE_LABEL_BLOB2_H_

#include <pcl/PointIndices.h>
#include <pcl/gpu/people/label_common.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {      
      /**
       * @brief This structure containts all parameters to describe blobs and their parent/child relations
       * @todo: clean this out in the end, perhaps place the children in a separate struct
       */
      struct Blob2 
      {
        int    id;                      // specific identification number of this blob
        part_t label;                   // labels which part this blob is, defined in common.
        int    lid;                     // label id, which number of this type of part is this

        Eigen::Vector4f  mean;          // mean in xyz
        Eigen::Matrix3f  cov;           // covariance in 3x3 matrix
        Eigen::Vector3f  eigenval;      // eigenvalue of blob
        Eigen::Matrix3f  eigenvect;     // eigenvector of blob

        //These variables are added in order to be able to build trees with them
        int    child_id[MAX_CHILD];     // id of the best found child
        int    child_lid[MAX_CHILD];    // lid of the best found child
        float  child_dist[MAX_CHILD];   // result of evaluation function of this child
        char   child_label[MAX_CHILD];  // makes displaying the tree easier
      
        pcl::PointIndices indices;      // The indices of the pointcloud
        Eigen::Vector4f   min;          // The min of the bounding box
        Eigen::Vector4f   max;          // The max of the bounding box
      };

      inline std::ostream& operator << (std::ostream& os, const Blob2& b)
      {
        os << " Blob2 id " << b.id << " label " << b.label << " lid " << b.lid << std::endl;
        os << " mean " << b.mean(0) << " , " << b.mean(1) << " , " << b.mean(2) << " , " << b.mean(3) << std::endl;
        os << " cov " << std::endl << b.cov << std::endl;
        os << " eigenval " << b.eigenval(0) << " , " << b.eigenval(1) << " , " << b.eigenval(2) << std::endl;
        os << " eigenvect " << std::endl << b.eigenvect << std::endl;
        os << " min " << b.min(0) << " , " << b.min(1) << " , " << b.min(2) << " , " << b.min(3) << std::endl;
        os << " max " << b.max(0) << " , " << b.max(1) << " , " << b.max(2) << " , " << b.max(3) << std::endl;
        os << " indices length " << b.indices.indices.size() << std::endl;
        for(int i = 0; i < MAX_CHILD; i++)
        {
          os << " child " << i << " id " << b.child_id[i] << " lid " << b.child_lid[i] << " dist " << b.child_dist[i] << " label " << b.child_label[i] << std::endl;
        }
        return (os);
      }      
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl

#endif
