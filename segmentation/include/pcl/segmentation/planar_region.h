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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/segmentation/region_3d.h>
#include <pcl/geometry/planar_polygon.h>

namespace pcl
{
  /** \brief PlanarRegion represents a set of points that lie in a plane.  Inherits summary statistics about these points from Region3D, and  summary statistics of a 3D collection of points.
    * \author Alex Trevor
    */
  template <typename PointT>
  class PlanarRegion : public pcl::Region3D<PointT>, public pcl::PlanarPolygon<PointT>
  {
    protected:
      using Region3D<PointT>::centroid_;
      using Region3D<PointT>::covariance_; 
      using Region3D<PointT>::count_;
      using PlanarPolygon<PointT>::contour_;
      using PlanarPolygon<PointT>::coefficients_;

    public:
      /** \brief Empty constructor for PlanarRegion. */
      PlanarRegion () = default;

      /** \brief Constructor for Planar region from a Region3D and a PlanarPolygon. 
        * \param[in] region a Region3D for the input data
        * \param[in] polygon a PlanarPolygon for the input region
        */
      PlanarRegion (const pcl::Region3D<PointT>& region, const pcl::PlanarPolygon<PointT>& polygon) 
      {
        centroid_ = region.centroid;
        covariance_ = region.covariance;
        count_ = region.count;
        contour_ = polygon.contour;
        coefficients_ = polygon.coefficients;
      }
      
      /** \brief Destructor. */
      ~PlanarRegion () override = default;

      /** \brief Constructor for PlanarRegion.
        * \param[in] centroid the centroid of the region.
        * \param[in] covariance the covariance of the region.
        * \param[in] count the number of points in the region.
        * \param[in] contour the contour / boudnary for the region
        * \param[in] coefficients the model coefficients (a,b,c,d) for the plane
        */
      PlanarRegion (const Eigen::Vector3f& centroid, const Eigen::Matrix3f& covariance, unsigned count,
                    const typename pcl::PointCloud<PointT>::VectorType& contour,
                    const Eigen::Vector4f& coefficients) 
      {
        centroid_ = centroid;
        covariance_ = covariance;
        count_ = count;
        contour_ = contour;
        coefficients_ = coefficients;
      }
      
    private:
      /** \brief The labels (good=true, bad=false) for whether or not this boundary was observed, 
        * or was due to edge of frame / occlusion boundary. 
        */
      std::vector<bool> contour_labels_;

    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}
