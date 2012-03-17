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
 *
 *
 */

#ifndef PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
#define PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_

#include <pcl/segmentation/comparator.h>
#include <boost/make_shared.hpp>

namespace pcl
{
  /** \brief PlaneCoefficientComparator is a Comparator that operates on plane coefficients, for use in planar segmentation.
    * In conjunction with OrganizedConnectedComponentSegmentation, this allows planes to be segmented from organized data.
    *
    * \author Alex Trevor
    */
  template<typename PointT, typename PointNT>
  class PlaneCoefficientComparator: public Comparator<PointT>
  {
    public:
      typedef typename Comparator<PointT>::PointCloud PointCloud;
      typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;
      
      typedef typename pcl::PointCloud<PointNT> PointCloudN;
      typedef typename PointCloudN::Ptr PointCloudNPtr;
      typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;
      
      typedef boost::shared_ptr<PlaneCoefficientComparator<PointT, PointNT> > Ptr;
      typedef boost::shared_ptr<const PlaneCoefficientComparator<PointT, PointNT> > ConstPtr;

      using pcl::Comparator<PointT>::input_;
      
      /** \brief Empty constructor for PlaneCoefficientComparator. */
      PlaneCoefficientComparator ()
        : normals_ (), plane_coeff_d_ (), angular_threshold_ (0), distance_threshold_ (0)
      {
      }

      /** \brief Constructor for PlaneCoefficientComparator.
        * \param[in] plane_coeff_d a reference to a vector of d coefficients of plane equations.  Must be the same size as the input cloud and input normals.  a, b, and c coefficients are in the input normals.
        */
      PlaneCoefficientComparator (boost::shared_ptr<std::vector<float> >& plane_coeff_d) 
        : normals_ (), plane_coeff_d_ (plane_coeff_d), angular_threshold_ (0), distance_threshold_ (0)
      {
      }
      
      /** \brief Destructor for PlaneCoefficientComparator. */
      virtual
      ~PlaneCoefficientComparator ()
      {
      }

      virtual void 
      setInputCloud (const PointCloudConstPtr& cloud)
      {
        input_ = cloud;
        Eigen::Matrix3f rot = input_->sensor_orientation_.toRotationMatrix ();
        z_axis_ = rot.col (2);
      }
      
      /** \brief Provide a pointer to the input normals.
        * \param[in] normals the input normal cloud
        */
      inline void
      setInputNormals (const PointCloudNConstPtr &normals)
      {
        normals_ = normals;
      }

      /** \brief Get the input normals. */
      inline PointCloudNConstPtr
      getInputNormals () const
      {
        return (normals_);
      }

      /** \brief Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.  a, b, and c are provided by the normal cloud.
        * \param[in] plane_coeff_d a pointer to the plane coefficients.
        */
      void
      setPlaneCoeffD (boost::shared_ptr<std::vector<float> >& plane_coeff_d)
      {
        plane_coeff_d_ = plane_coeff_d;
      }

      /** \brief Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.  a, b, and c are provided by the normal cloud.
        * \param[in] plane_coeff_d a pointer to the plane coefficients.
        */
      void
      setPlaneCoeffD (std::vector<float>& plane_coeff_d)
      {
        plane_coeff_d_ = boost::make_shared<std::vector<float> >(plane_coeff_d);
      }
      
      /** \brief Get a pointer to the vector of the d-coefficient of the planes' hessian normal form. */
      const std::vector<float>&
      getPlaneCoeffD () const
      {
        return (plane_coeff_d_);
      }

      /** \brief Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
        * \param[in] angular_threshold the tolerance in radians
        */
      virtual inline void
      setAngularThreshold (float angular_threshold)
      {
        angular_threshold_ = cosf (angular_threshold);
      }
      
      /** \brief Get the angular threshold in radians for difference in normal direction between neighboring points, to be considered part of the same plane. */
      inline float
      getAngularThreshold () const
      {
        return (acos (angular_threshold_) );
      }

      /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
        * \param[in] distance_threshold the tolerance in meters
        */
      inline void
      setDistanceThreshold (float distance_threshold, bool depth_dependent)
      {
        distance_threshold_ = distance_threshold;
        depth_dependent_ = depth_dependent;
      }

      /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
      inline float
      getDistanceThreshold () const
      {
        return (distance_threshold_);
      }
      
      /** \brief Compare points at two indices by their plane equations.  True if the angle between the normals is less than the angular threshold,
        * and the difference between the d component of the normals is less than distance threshold, else false
        * \param idx1 The first index for the comparison
        * \param idx2 The second index for the comparison
        */
      virtual bool
      compare (int idx1, int idx2) const
      {
        float threshold = distance_threshold_;
        if (depth_dependent_)
        {
          Eigen::Vector4f origin = input_->sensor_origin_;
          Eigen::Vector3f vec = input_->points[idx1].getVector3fMap () - origin.head<3> ();
          
          float z = vec.dot (z_axis_);
          threshold *= z * z;
        }
        return ( (fabs ((*plane_coeff_d_)[idx1] - (*plane_coeff_d_)[idx2]) < threshold)
                 && (normals_->points[idx1].getNormalVector3fMap ().dot (normals_->points[idx2].getNormalVector3fMap () ) > angular_threshold_ ) );
      }
      
    protected:
      PointCloudNConstPtr normals_;
      boost::shared_ptr<std::vector<float> > plane_coeff_d_;
      float angular_threshold_;
      float distance_threshold_;
      bool depth_dependent_;
      Eigen::Vector3f z_axis_;
  };
}

#endif // PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
