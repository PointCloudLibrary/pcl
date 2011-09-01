/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <pcl/pcl_config.h>
#ifdef HAVE_QHULL

#ifndef PCL_CONVEX_HULL_2D_H_
#define PCL_CONVEX_HULL_2D_H_

// PCL includes
#include "pcl/pcl_base.h"
#include "pcl/io/io.h"

#include "pcl/ModelCoefficients.h"
#include "pcl/PolygonMesh.h"
#include <math.h>

namespace pcl
{
  /** \brief Sort 2D points in a vector structure
    * \param p1 the first point
    * \param p2 the second point
    * \ingroup surface
    */
  inline bool
  comparePoints2D (const std::pair<int, Eigen::Vector4f> & p1, const std::pair<int, Eigen::Vector4f> & p2)
  {
    double angle1 = atan2 (p1.second[1], p1.second[0]) + M_PI;
    double angle2 = atan2 (p2.second[1], p2.second[0]) + M_PI;
    return (angle1 > angle2);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ConvexHull using libqhull library.
    * \author Aitor Aldoma
    * \ingroup surface
    */
  template<typename PointInT>
  class ConvexHull : public PCLBase<PointInT>
  {
    using PCLBase<PointInT>::input_;
    using PCLBase<PointInT>::indices_;
    using PCLBase<PointInT>::initCompute;
    using PCLBase<PointInT>::deinitCompute;

    public:
      typedef pcl::PointCloud<PointInT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      /** \brief Empty constructor. */
      ConvexHull ()
      {
        keep_information_ = false;
      };

      /** \brief Compute a convex hull for all points given 
        *
        * \param points the resultant points lying on the convex hull 
        * \param polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
      void
      reconstruct (PointCloud &points, 
                   std::vector<pcl::Vertices> &polygons);

      /** \brief Compute a convex hull for all points given 
        * \param output the resultant convex hull vertices
        */
      void
      reconstruct (PointCloud &output);

      /** \brief If keep_information_is set to true the convex hull
        * points keep other information like rgb, normals, ...
        * \param value where to keep the information or not, default is false
        */
      void
      setKeepInformation (bool value)
      {
        keep_information_ = value;
      }

    private:
      /** \brief The actual reconstruction method. 
        * 
        * \param points the resultant points lying on the convex hull 
        * \param polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        * \param fill_polygon_data true if polygons should be filled, false otherwise
        */
      void
      performReconstruction (PointCloud &points, 
                             std::vector<pcl::Vertices> &polygons, 
                             bool fill_polygon_data = false);

      bool keep_information_;

    protected:
      /** \brief Class get name method. */
      std::string
      getClassName () const
      {
        return ("ConvexHull");
      }
    };
}

#endif  //#ifndef PCL_CONVEX_HULL_2D_H_
#endif
