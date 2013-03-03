/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */


#ifndef PCL_WARP_POINT_RIGID_3D_H_
#define PCL_WARP_POINT_RIGID_3D_H_

#include <pcl/registration/eigen.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b WarpPointRigid3D enables 3D (1D rotation + 2D translation) 
      * transformations for points.
      * 
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
    class WarpPointRigid3D : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
    {
      public:
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointRigid3D<PointSourceT, PointTargetT, Scalar> > Ptr;
        typedef boost::shared_ptr<const WarpPointRigid3D<PointSourceT, PointTargetT, Scalar> > ConstPtr;

        /** \brief Constructor. */
        WarpPointRigid3D () : WarpPointRigid<PointSourceT, PointTargetT, Scalar> (3) {}
      
        /** \brief Empty destructor */
        virtual ~WarpPointRigid3D () {}

        /** \brief Set warp parameters. 
          * \param[in] p warp parameters (tx ty rz)
          */
        virtual void 
        setParam (const VectorX & p)
        {
          assert (p.rows () == this->getDimension ());
          Matrix4 &trans = this->transform_matrix_;

          trans = Matrix4::Zero ();
          trans (3, 3) = 1;
          trans (2, 2) = 1; // Rotation around the Z-axis

          // Copy the rotation and translation components
          trans.block (0, 3, 4, 1) = Eigen::Matrix<Scalar, 4, 1> (p[0], p[1], 0, 1.0);

          // Compute w from the unit quaternion
          Eigen::Rotation2D<Scalar> r (p[2]);
          trans.topLeftCorner (2, 2) = r.toRotationMatrix ();
        }
    };
  }
}

#endif

