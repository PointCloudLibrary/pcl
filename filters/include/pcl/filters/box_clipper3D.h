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

#ifndef PCL_BOX_CLIPPER3D_H_
#define PCL_BOX_CLIPPER3D_H_
#include "clipper3D.h"

namespace pcl
{
  /**
    * \author Suat Gedikli <gedikli@willowgarage.com>
    * \brief Implementation of a box clipper in 3D. Actually it allows affine transformations, thus any parallelepiped in general pose.
    *        The affine transformation is used to transform the point before clipping it using the unit cube centered at origin and with an extend of -1 to +1 in each dimension
    * \ingroup filters
    */
  template<typename PointT>
  class BoxClipper3D : public Clipper3D<PointT>
  {
    public:

      typedef boost::shared_ptr< BoxClipper3D<PointT> > Ptr;
      typedef boost::shared_ptr< const BoxClipper3D<PointT> > ConstPtr;


      /**
        * \author Suat Gedikli <gedikli@willowgarage.com>
        * \brief Constructor taking an affine transformation matrix, which allows also shearing of the clipping area
        * \param[in] transformation the 3x3 affine transformation matrix that is used to describe the unit cube
        */
      BoxClipper3D (const Eigen::Affine3f& transformation);

      /**
        * \brief creates a BoxClipper object with a scaled box in general pose
        * \param[in] rodrigues the rotation axis and angle given by the vector direction and length respectively
        * \param[in] translation the position of the box center
        * \param[in] box_size the size of the box for each dimension
        */
      BoxClipper3D (const Eigen::Vector3f& rodrigues, const Eigen::Vector3f& translation, const Eigen::Vector3f& box_size);

      /**
        * \brief Set the affine transformation
        * \param[in] transformation
        */
      void setTransformation (const Eigen::Affine3f& transformation);

      /**
        * \brief sets the box in general pose given by the orientation position and size
        * \param[in] rodrigues the rotation axis and angle given by the vector direction and length respectively
        * \param[in] translation the position of the box center
        * \param[in] box_size the size of the box for each dimension
        */
      void setTransformation (const Eigen::Vector3f& rodrigues, const Eigen::Vector3f& translation, const Eigen::Vector3f& box_size);

      /**
        * \brief virtual destructor
        */
      virtual ~BoxClipper3D () throw ();

      virtual bool
      clipPoint3D (const PointT& point) const;

      virtual bool
      clipLineSegment3D (PointT& from, PointT& to) const;

      virtual void
      clipPlanarPolygon3D (std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon) const;

      virtual void
      clipPlanarPolygon3D (const std::vector<PointT, Eigen::aligned_allocator<PointT> >& polygon, std::vector<PointT, Eigen::aligned_allocator<PointT> >& clipped_polygon) const;

      virtual void
      clipPointCloud3D (const pcl::PointCloud<PointT> &cloud_in, std::vector<int>& clipped, const std::vector<int>& indices = std::vector<int> ()) const;

      virtual Clipper3D<PointT>*
      clone () const;

    protected:
      float getDistance (const PointT& point) const;
      void transformPoint (const PointT& pointIn, PointT& pointOut) const;
    private:
      /**
        * \brief the affine transformation that is applied before clipping is done on the unit cube.
        */
      Eigen::Affine3f transformation_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#include <pcl/filters/impl/box_clipper3D.hpp>

#endif // PCL_BOX_CLIPPER3D_H_
