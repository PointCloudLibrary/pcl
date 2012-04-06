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
 * @author Koen Buys
 * @file tree2.h
 * @brief This file contains the Tree2 structure and the inline <<-operator for it.
 */

#ifndef PCL_GPU_PEOPLE_LABEL_SKELETON_TREE2_H_
#define PCL_GPU_PEOPLE_LABEL_SKELETON_TREE2_H_

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// Our header
#include <pcl/gpu/people/label_skeleton/common.h>
#include <pcl/gpu/people/label_skeleton/blob2.h>

namespace pcl
{
  namespace gpu
  {
    namespace people
    {
      namespace label_skeleton
      {
        /**
         * @brief This structure containts all parameters to describe the segmented tree
         */
        struct Tree2 {
          //Inline constructor
          inline Tree2() {
            id = NO_CHILD;
            lid = NO_CHILD;
            nr_parts = 0;
            for(int i=0;i<NUM_PARTS;i++)
              parts_lid[i] = NO_CHILD;
          }

          inline Tree2( const Tree2& T) {
            *this = T;
          }

          Tree2& operator = ( const Tree2& T ) {
            id    = T.id;     // unique ID for every tree in the  frame
            label = T.label;  // the root part label of this tree
            lid   = T.lid;    // the label id, namely the number of blob of this label to uniquely identify a tree

            for(int i=0;i<3;++i)       mean(i) = T.mean(i);
            for(int i=0;i<9;++i)        cov(i) = T.cov(i);
            for(int i=0;i<3;++i)   eigenval(i) = T.eigenval(i);
            for(int i=0;i<9;++i)  eigenvect(i) = T.eigenvect(i);

            indices = T.indices;

            for(int i=0;i<3;++i)   min(i) = T.min(i);
            for(int i=0;i<3;++i)   max(i) = T.max(i);

            return *this;
          }

          int     id;                     // specific identification number of this tree
          part_t  label;                  // labels which part the root of this tree is
          int     lid;                    // label id, which number of this type of part is this
          int     nr_parts;               // the number of parts in this tree
          int     parts_lid[NUM_PARTS];   // Indicate the used parts
          float   total_dist_error;       // sum of all distance errors
          float   norm_dist_error;         // total_dist_error/nr_parts

          Eigen::Vector4f  mean;          // mean in xyz
          Eigen::Matrix3f  cov;           // covariance in 3x3 matrix
          Eigen::Vector3f  eigenval;      // eigenvalue of blob
          Eigen::Matrix3f  eigenvect;     // eigenvector of blob

          pcl::PointIndices indices;      // The indices of the pointcloud
          Eigen::Vector4f   min;          // The min of the bounding box
          Eigen::Vector4f   max;          // The max of the bounding box
        };

        inline std::ostream& operator << (std::ostream& os, const Tree2& t)
        {
          os << " Tree2 id " << t.id << " label " << t.label << " lid " << t.lid << " nr_parts " << t.nr_parts << std::endl;
          os << " total_dist_error " << t.total_dist_error << " norm_dist_error " << t.norm_dist_error << std::endl;
          os << " mean " << t.mean(0) << " , " << t.mean(1) << " , " << t.mean(2) << " , " << t.mean(3) << std::endl;
          os << " cov " << std::endl << t.cov << std::endl;
          os << " eigenval " << t.eigenval(0) << " , " << t.eigenval(1) << " , " << t.eigenval(2) << std::endl;
          os << " eigenvect " << std::endl << t.eigenvect << std::endl;
          os << " min " << t.min(0) << " , " << t.min(1) << " , " << t.min(2) << " , " << t.min(3) << std::endl;
          os << " max " << t.max(0) << " , " << t.max(1) << " , " << t.max(2) << " , " << t.max(3) << std::endl;
          os << " indices length " << t.indices.indices.size() << std::endl;
          return (os);
        }
      } // end namespace LabelSkel
    } // end namespace people
  } // end namespace gpu
} // end namespace pcl
#endif
