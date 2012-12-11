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

#ifndef PCL_SEGMENTATION_GROUND_PLANE_COMPARATOR_H_
#define PCL_SEGMENTATION_GROUND_PLANE_COMPARATOR_H_

#include <pcl/common/angles.h>
#include <pcl/segmentation/comparator.h>
#include <boost/make_shared.hpp>

namespace pcl
{
  /** \brief GroundPlaneComparator is a Comparator for detecting smooth surfaces suitable for driving.
    * In conjunction with OrganizedConnectedComponentSegmentation, this allows smooth groundplanes / road surfaces to be segmented from point clouds.
    *
    * \author Alex Trevor
    */
  template<typename PointT, typename PointNT>
  class GroundPlaneComparator: public Comparator<PointT>
  {
    public:
      typedef typename Comparator<PointT>::PointCloud PointCloud;
      typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;
      
      typedef typename pcl::PointCloud<PointNT> PointCloudN;
      typedef typename PointCloudN::Ptr PointCloudNPtr;
      typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;
      
      typedef boost::shared_ptr<GroundPlaneComparator<PointT, PointNT> > Ptr;
      typedef boost::shared_ptr<const GroundPlaneComparator<PointT, PointNT> > ConstPtr;

      using pcl::Comparator<PointT>::input_;
      
      /** \brief Empty constructor for GroundPlaneComparator. */
      GroundPlaneComparator ()
        : normals_ ()
        , plane_coeff_d_ ()
        , angular_threshold_ (cosf (pcl::deg2rad (2.0f)))
        , road_angular_threshold_ ( cosf(pcl::deg2rad (10.0f)))
        , distance_threshold_ (0.1f)
        , depth_dependent_ (true)
        , z_axis_ (Eigen::Vector3f (0.0, 0.0, 1.0) )
        , desired_road_axis_ (Eigen::Vector3f(0.0, -1.0, 0.0))
      {
      }

      /** \brief Constructor for GroundPlaneComparator.
        * \param[in] plane_coeff_d a reference to a vector of d coefficients of plane equations.  Must be the same size as the input cloud and input normals.  a, b, and c coefficients are in the input normals.
        */
      GroundPlaneComparator (boost::shared_ptr<std::vector<float> >& plane_coeff_d) 
        : normals_ ()
        , plane_coeff_d_ (plane_coeff_d)
        , angular_threshold_ (cosf (pcl::deg2rad (3.0f)))
        , distance_threshold_ (0.1f)
        , depth_dependent_ (true)
        , z_axis_ (Eigen::Vector3f (0.0f, 0.0f, 1.0f))
        , road_angular_threshold_ ( cosf(pcl::deg2rad (40.0f)))
        , desired_road_axis_ (Eigen::Vector3f(0.0, -1.0, 0.0))
      {
      }
      
      /** \brief Destructor for GroundPlaneComparator. */
      virtual
      ~GroundPlaneComparator ()
      {
      }
      /** \brief Provide the input cloud.
        * \param[in] cloud the input point cloud.
        */
      virtual void 
      setInputCloud (const PointCloudConstPtr& cloud)
      {
        input_ = cloud;
      }
      
      /** \brief Provide a pointer to the input normals.
        * \param[in] normals the input normal cloud.
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
      virtual void
      setAngularThreshold (float angular_threshold)
      {
        angular_threshold_ = cosf (angular_threshold);
      }

      /** \brief Set the tolerance in radians for difference in normal direction between a point and the expected ground normal.
        * \param[in] angular_threshold the
        */
      virtual void
      setGroundAngularThreshold (float angular_threshold)
      {
        road_angular_threshold_ = cosf (angular_threshold);
      }

      /** \brief Set the expected ground plane normal with respect to the sensor.  Pixels labeled as ground must be within ground_angular_threshold radians of this normal to be labeled as ground.
        * \param[in] normal The normal direction of the expected ground plane.
        */
      void
      setExpectedGroundNormal (Eigen::Vector3f normal)
      {
        desired_road_axis_ = normal;
      }
  
      
      /** \brief Get the angular threshold in radians for difference in normal direction between neighboring points, to be considered part of the same plane. */
      inline float
      getAngularThreshold () const
      {
        return (acosf (angular_threshold_) );
      }

      /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
        * \param[in] distance_threshold the tolerance in meters (at 1m)
        * \param[in] depth_dependent whether to scale the threshold based on range from the sensor (default: false)
        */
      void
      setDistanceThreshold (float distance_threshold, 
                            bool depth_dependent = false)
      {
        distance_threshold_ = distance_threshold;
        depth_dependent_ = depth_dependent;
      }

      /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
      inline float
      getDistanceThreshold () const
      {
        return distance_threshold_;
      }
      
      /** \brief Compare points at two indices by their plane equations.  True if the angle between the normals is less than the angular threshold,
        * and the difference between the d component of the normals is less than distance threshold, else false
        * \param idx1 The first index for the comparison
        * \param idx2 The second index for the comparison
        */
      virtual bool
      compare (int idx1, int idx2) const
      {
        // Normal must be similar to neighbor
        // Normal must be similar to expected normal
        float threshold = distance_threshold_;
        if (depth_dependent_)
        {
          Eigen::Vector3f vec = input_->points[idx1].getVector3fMap ();
          
          float z = vec.dot (z_axis_);
          threshold *= z * z;
        }

        return ( (normals_->points[idx1].getNormalVector3fMap ().dot (desired_road_axis_) > road_angular_threshold_) &&
                 (normals_->points[idx1].getNormalVector3fMap ().dot (normals_->points[idx2].getNormalVector3fMap () ) > angular_threshold_ ));
        
        // Euclidean proximity of neighbors does not seem to be required -- pixel adjacency handles this well enough 
        //return ( (normals_->points[idx1].getNormalVector3fMap ().dot (desired_road_axis_) > road_angular_threshold_) &&
        //          (normals_->points[idx1].getNormalVector3fMap ().dot (normals_->points[idx2].getNormalVector3fMap () ) > angular_threshold_ ) &&
        //         (pcl::euclideanDistance (input_->points[idx1], input_->points[idx2]) < distance_threshold_ ));
      }
      
    protected:
      PointCloudNConstPtr normals_;
      boost::shared_ptr<std::vector<float> > plane_coeff_d_;
      float angular_threshold_;
      float road_angular_threshold_;
      float distance_threshold_;
      bool depth_dependent_;
      Eigen::Vector3f z_axis_;
      Eigen::Vector3f desired_road_axis_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif // PCL_SEGMENTATION_GROUND_PLANE_COMPARATOR_H_
