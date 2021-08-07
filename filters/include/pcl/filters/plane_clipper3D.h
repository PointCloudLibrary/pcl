/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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

#include "clipper3D.h"

namespace pcl
{
  /**
   * @author Suat Gedikli <gedikli@willowgarage.com>
   * @brief Implementation of a plane clipper in 3D
   * \ingroup filters
   */
  template<typename PointT>
  class PlaneClipper3D : public Clipper3D<PointT>
  {
    public:

      using Ptr = shared_ptr< PlaneClipper3D<PointT> >;
      using ConstPtr = shared_ptr< const PlaneClipper3D<PointT> >;

      /**
       * @author Suat Gedikli <gedikli@willowgarage.com>
       * @brief Constructor taking the homogeneous representation of the plane as a Eigen::Vector4f
       * @param[in] plane_params plane parameters, need not necessarily be normalized
       */
      PlaneClipper3D (const Eigen::Vector4f& plane_params);

      virtual ~PlaneClipper3D () noexcept;

      /**
        * \brief Set new plane parameters
        * \param plane_params
        */
      void setPlaneParameters (const Eigen::Vector4f& plane_params);

      /**
        * \brief return the current plane parameters
        * \return the current plane parameters
        */
      const Eigen::Vector4f& getPlaneParameters () const;

      virtual bool
      clipPoint3D (const PointT& point) const;

      virtual bool
      clipLineSegment3D (PointT& from, PointT& to) const;

      virtual void
      clipPlanarPolygon3D (std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon) const;

      virtual void
      clipPlanarPolygon3D (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon, std::vector<PointT, Eigen::aligned_allocator<PointT> >& clipped_polygon) const;

      virtual void
      clipPointCloud3D (const pcl::PointCloud<PointT> &cloud_in, Indices& clipped, const Indices& indices = Indices ()) const;

      virtual Clipper3D<PointT>*
      clone () const;

    protected:
      float
      getDistance (const PointT& point) const;

    private:
      Eigen::Vector4f plane_params_;
  };
}

#include <pcl/filters/impl/plane_clipper3D.hpp>
