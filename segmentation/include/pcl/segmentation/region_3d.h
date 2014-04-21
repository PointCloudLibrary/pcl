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
 */

#ifndef PCL_SEGMENTATION_REGION_3D_H_
#define PCL_SEGMENTATION_REGION_3D_H_

#include <Eigen/Core>
#include <vector>

namespace pcl
{
  /** \brief Region3D represents summary statistics of a 3D collection of points.
    * \author Alex Trevor
    */
  template <typename PointT>
  class Region3D
  {
    public:
      /** \brief Empty constructor for Region3D. */
      Region3D () : centroid_ (Eigen::Vector3f::Zero ()), covariance_ (Eigen::Matrix3f::Identity ()), count_ (0)
      {
      }
      
      /** \brief Constructor for Region3D. 
        * \param[in] centroid The centroid of the region.
        * \param[in] covariance The covariance of the region.
        * \param[in] count The number of points in the region.
        */
      Region3D (Eigen::Vector3f& centroid, Eigen::Matrix3f& covariance, unsigned count) 
        : centroid_ (centroid), covariance_ (covariance), count_ (count)
      {
      }
     
      /** \brief Destructor. */
      virtual ~Region3D () {}

      /** \brief Get the centroid of the region. */
      inline Eigen::Vector3f 
      getCentroid () const
      {
        return (centroid_);
      }
      
      /** \brief Get the covariance of the region. */
      inline Eigen::Matrix3f
      getCovariance () const
      {
        return (covariance_);
      }
      
      /** \brief Get the number of points in the region. */
      unsigned
      getCount () const
      {
        return (count_);
      }

      /** \brief Get the curvature of the region. */
      float
      getCurvature () const
      {
        return (curvature_);
      }

      /** \brief Set the curvature of the region. */
      void
      setCurvature (float curvature)
      {
        curvature_ = curvature;
      }

    protected:
      /** \brief The centroid of the region. */
      Eigen::Vector3f centroid_;
      
      /** \brief The covariance of the region. */
      Eigen::Matrix3f covariance_;
      
      /** \brief The number of points in the region. */
      unsigned count_;

      /** \brief The mean curvature of the region. */
      float curvature_;
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif //#ifndef  PCL_SEGMENTATION_REGION_3D_H_
