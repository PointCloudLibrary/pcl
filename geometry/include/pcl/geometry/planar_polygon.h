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
 * $Id: planar_polygon.h 4696 2012-02-23 06:12:55Z rusu $
 *
 */

#pragma once

#include <pcl/common/eigen.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

namespace pcl
{
  /** \brief PlanarPolygon represents a planar (2D) polygon, potentially in a 3D space.
    * \author Alex Trevor 
    */
  template <typename PointT>
  class PlanarPolygon
  {
    public:
      using Ptr = shared_ptr<PlanarPolygon<PointT> >;
      using ConstPtr = shared_ptr<const PlanarPolygon<PointT> >;

       /** \brief Empty constructor for PlanarPolygon */
      PlanarPolygon () : contour_ ()
      {}
      
      /** \brief Constructor for PlanarPolygon
        * \param[in] contour a vector of points bounding the polygon
        * \param[in] coefficients a vector of the plane's coefficients (a,b,c,d)
        */
      PlanarPolygon (typename pcl::PointCloud<PointT>::VectorType &contour,
                     Eigen::Vector4f& coefficients) 
        : contour_ (contour), coefficients_ (coefficients)
      {}
      
      /** \brief Destructor. */
      virtual ~PlanarPolygon () {}

      /** \brief Set the internal contour
        * \param[in] contour the new planar polygonal contour
        */
      void
      setContour (const pcl::PointCloud<PointT> &contour)
      {
        contour_ = contour.points;
      }

      /** \brief Getter for the contour / boundary */
      typename pcl::PointCloud<PointT>::VectorType&
      getContour ()
      {
        return (contour_);
      }
      
      /** \brief Getter for the contour / boundary */
      const typename pcl::PointCloud<PointT>::VectorType&
      getContour () const
      {
        return (contour_);
      }

      /** \brief Setr the internal coefficients
        * \param[in] coefficients the new coefficients to be set 
        */
      void
      setCoefficients (const Eigen::Vector4f &coefficients)
      {
        coefficients_ = coefficients;
      }

      /** \brief Set the internal coefficients
        * \param[in] coefficients the new coefficients to be set 
        */
      void
      setCoefficients (const pcl::ModelCoefficients &coefficients)
      {
        for (int i = 0; i < 4; i++)
          coefficients_[i] = coefficients.values.at (i);
      }

      /** \brief Getter for the coefficients */
      Eigen::Vector4f&
      getCoefficients ()
      {
        return (coefficients_);
      }

      /** \brief Getter for the coefficients */
      const Eigen::Vector4f&
      getCoefficients () const
      {
        return (coefficients_);
      }
      
    protected:
      /** \brief A list of points on the boundary/contour of the planar region. */
      typename pcl::PointCloud<PointT>::VectorType contour_;
      
      /** \brief A list of model coefficients (a,b,c,d). */
      Eigen::Vector4f coefficients_;
    
    public:
      PCL_MAKE_ALIGNED_OPERATOR_NEW
  };
}
