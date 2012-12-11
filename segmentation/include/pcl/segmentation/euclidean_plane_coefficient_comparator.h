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
 *
 *
 */

#ifndef PCL_EUCLIDEAN_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
#define PCL_EUCLIDEAN_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_

#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>

namespace pcl
{
  /** \brief EuclideanPlaneCoefficientComparator is a Comparator that operates on plane coefficients, 
    * for use in planar segmentation.
    * In conjunction with OrganizedConnectedComponentSegmentation, this allows planes to be segmented from organized data.
    *
    * \author Alex Trevor
    */
  template<typename PointT, typename PointNT>
  class EuclideanPlaneCoefficientComparator: public PlaneCoefficientComparator<PointT, PointNT>
  {
    public:
      typedef typename Comparator<PointT>::PointCloud PointCloud;
      typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;
      typedef typename pcl::PointCloud<PointNT> PointCloudN;
      typedef typename PointCloudN::Ptr PointCloudNPtr;
      typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;
      
      typedef boost::shared_ptr<EuclideanPlaneCoefficientComparator<PointT, PointNT> > Ptr;
      typedef boost::shared_ptr<const EuclideanPlaneCoefficientComparator<PointT, PointNT> > ConstPtr;

      using pcl::Comparator<PointT>::input_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::normals_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::angular_threshold_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::distance_threshold_;
      
      /** \brief Empty constructor for PlaneCoefficientComparator. */
      EuclideanPlaneCoefficientComparator ()
      {
      }

      /** \brief Destructor for PlaneCoefficientComparator. */
      virtual
      ~EuclideanPlaneCoefficientComparator ()
      {
      }

      /** \brief Compare two neighboring points, by using normal information, and euclidean distance information.
        * \param[in] idx1 The index of the first point.
        * \param[in] idx2 The index of the second point.
        */
      virtual bool
      compare (int idx1, int idx2) const
      {
        float dx = input_->points[idx1].x - input_->points[idx2].x;
        float dy = input_->points[idx1].y - input_->points[idx2].y;
        float dz = input_->points[idx1].z - input_->points[idx2].z;
        float dist = sqrtf (dx*dx + dy*dy + dz*dz);
        
        return ( (dist < distance_threshold_)
                 && (normals_->points[idx1].getNormalVector3fMap ().dot (normals_->points[idx2].getNormalVector3fMap () ) > angular_threshold_ ) );
      }
  };
}

#endif // PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
