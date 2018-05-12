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
 * $Id: extract_clusters.h 5027 2012-03-12 03:10:45Z rusu $
 *
 */

#ifndef PCL_SEGMENTATION_EDGE_AWARE_PLANE_COMPARATOR_H_
#define PCL_SEGMENTATION_EDGE_AWARE_PLANE_COMPARATOR_H_

#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>

namespace pcl
{
  /** \brief EdgeAwarePlaneComparator is a Comparator that operates on plane coefficients, 
    * for use in planar segmentation.
    * In conjunction with OrganizedConnectedComponentSegmentation, this allows planes to be segmented from organized data.
    *
    * \author Stefan Holzer, Alex Trevor
    */
  template<typename PointT, typename PointNT>
  class EdgeAwarePlaneComparator: public PlaneCoefficientComparator<PointT, PointNT>
  {
    public:
      typedef typename Comparator<PointT>::PointCloud PointCloud;
      typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;
      
      typedef typename pcl::PointCloud<PointNT> PointCloudN;
      typedef typename PointCloudN::Ptr PointCloudNPtr;
      typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;
      
      typedef boost::shared_ptr<EdgeAwarePlaneComparator<PointT, PointNT> > Ptr;
      typedef boost::shared_ptr<const EdgeAwarePlaneComparator<PointT, PointNT> > ConstPtr;

      using pcl::PlaneCoefficientComparator<PointT, PointNT>::input_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::normals_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::plane_coeff_d_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::angular_threshold_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::distance_threshold_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::depth_dependent_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::z_axis_;

      /** \brief Empty constructor for PlaneCoefficientComparator. */
      EdgeAwarePlaneComparator () :
        distance_map_threshold_ (5),
        curvature_threshold_ (0.04f),
        euclidean_distance_threshold_ (0.04f)
      {
      }

      /** \brief Empty constructor for PlaneCoefficientComparator. 
        * \param[in] distance_map the distance map to use
        */
      EdgeAwarePlaneComparator (const float *distance_map) : 
        distance_map_ (distance_map),
        distance_map_threshold_ (5),
        curvature_threshold_ (0.04f),
        euclidean_distance_threshold_ (0.04f)
      {
      }

      /** \brief Destructor for PlaneCoefficientComparator. */
      virtual
      ~EdgeAwarePlaneComparator ()
      {
      }

      /** \brief Set a distance map to use. For an example of a valid distance map see 
        * \ref OrganizedIntegralImageNormalEstimation
        * \param[in] distance_map the distance map to use
        */
      inline void
      setDistanceMap (const float *distance_map)
      {
        distance_map_ = distance_map;
      }

      /** \brief Return the distance map used. */
      const float*
      getDistanceMap () const
      {
        return (distance_map_);
      }

      /** \brief Set the curvature threshold for creating a new segment
        * \param[in] curvature_threshold a threshold for the curvature
        */
      void
      setCurvatureThreshold (float curvature_threshold)
      {
        curvature_threshold_ = curvature_threshold;
      }

      /** \brief Get the curvature threshold. */
      inline float
      getCurvatureThreshold () const
      {
        return (curvature_threshold_);
      }

      /** \brief Set the distance map threshold -- the number of pixel away from a border / nan
        * \param[in] distance_map_threshold the distance map threshold
        */
      void
      setDistanceMapThreshold (float distance_map_threshold)
      {
        distance_map_threshold_ = distance_map_threshold;
      }

      /** \brief Get the distance map threshold (in pixels). */
      inline float
      getDistanceMapThreshold () const
      {
        return (distance_map_threshold_);
      }

      /** \brief Set the euclidean distance threshold.
        * \param[in] euclidean_distance_threshold the euclidean distance threshold in meters
        */
      void
      setEuclideanDistanceThreshold (float euclidean_distance_threshold)
      {
        euclidean_distance_threshold_ = euclidean_distance_threshold;
      }

      /** \brief Get the euclidean distance threshold. */
      inline float
      getEuclideanDistanceThreshold () const
      {
        return (euclidean_distance_threshold_);
      }
      
    protected:
      /** \brief Compare two neighboring points, by using normal information, curvature, and euclidean distance information.
        * \param[in] idx1 The index of the first point.
        * \param[in] idx2 The index of the second point.
        */
      bool
      compare (int idx1, int idx2) const
      {
        // Note: there are two distance thresholds here that make sense to scale with depth.
        // dist_threshold is on the perpendicular distance to the plane, as in plane comparator
        // We additionally check euclidean distance to ensure that we don't have neighboring coplanar points
        // that aren't close in euclidean space (think two tables separated by a meter, viewed from an angle
        // where the surfaces are adjacent in image space).
        float dist_threshold = distance_threshold_;
        float euclidean_dist_threshold = euclidean_distance_threshold_;
        if (depth_dependent_)
        {
          Eigen::Vector3f vec = input_->points[idx1].getVector3fMap ();
          float z = vec.dot (z_axis_);
          dist_threshold *= z * z;
          euclidean_dist_threshold *= z * z;
        }
        
        float dx = input_->points[idx1].x - input_->points[idx2].x;
        float dy = input_->points[idx1].y - input_->points[idx2].y;
        float dz = input_->points[idx1].z - input_->points[idx2].z;
        float dist = std::sqrt (dx*dx + dy*dy + dz*dz);

        bool normal_ok = (normals_->points[idx1].getNormalVector3fMap ().dot (normals_->points[idx2].getNormalVector3fMap () ) > angular_threshold_ );
        bool dist_ok = (dist < euclidean_dist_threshold);

        bool curvature_ok = normals_->points[idx1].curvature < curvature_threshold_;
        bool plane_d_ok = fabs ((*plane_coeff_d_)[idx1] - (*plane_coeff_d_)[idx2]) < dist_threshold;
        
        if (distance_map_[idx1] < distance_map_threshold_)    
          curvature_ok = false;
        
        return (dist_ok && normal_ok && curvature_ok && plane_d_ok);
      }

    protected:
      const float* distance_map_;
      int distance_map_threshold_;
      float curvature_threshold_;
      float euclidean_distance_threshold_;
  };
}

#endif // PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
