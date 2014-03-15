/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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

#ifndef PCL_OCTREE_CENTROID_POINT_H
#define PCL_OCTREE_CENTROID_POINT_H

#include <pcl/octree/impl/centroid_point.hpp>

namespace pcl
{

  namespace octree
  {

    /** A generic class that computes the centroid of points fed to it.
      *
      * Centroid computation is field-wise, i.e. the average is computed for
      * each of the supported fields separately. Currently the following fields
      * are supported:
      *
      * - XYZ
      * - Normal+Curvature
      * - RGB/RGBA
      *
      * The template parameter defines the type of points that may be
      * accumulated with this class. This may be arbitrary PCL point type, and
      * centroid computation will happen only for the fields that are present
      * in it and are supported.
      *
      * Current centroid may be retrieved at any time using get(). Note that
      * the function is templated on point type, so it is possible to fetch the
      * centroid into a point type that differs from the type of points that
      * are being accumulated. All the "extra" fields for which the centroid is
      * not being calculated will be left untouched.
      *
      * Example usage:
      *
      * \code
      * // Create and accumulate points
      * CentroidPoint<pcl::PointXYZ> centroid;
      * centroid.add (pcl::PointXYZ (1, 2, 3);
      * centroid.add (pcl::PointXYZ (5, 6, 7);
      * // Fetch centroid using `get()`
      * pcl::PointXYZ c1;
      * centroid.get (c1);
      * // The expected result is: c1.x == 3, c1.y == 4, c1.z == 5
      * // It is also okay to use `get()` with a different point type
      * pcl::PointXYZRGB c2;
      * centroid.get (c2);
      * // The expected result is: c2.x == 3, c2.y == 4, c2.z == 5,
      * // and c2.rgb is left untouched
      * \endcode
      *
      * Note that the template can be successfuly instantiated for *any* PCL
      * point type. Of course, each of the field averages is computed only if
      * the point type has the corresponding field.
      *
      * \ingroup octree
      * \author Sergey Alexandrov */
    template <typename PointT>
    class CentroidPoint
    {

      public:

        CentroidPoint ()
        : num_points_ (0)
        {
        }

        /** Add a new point to the centroid computation.
          *
          * In this function only the accumulators and point counter are being,
          * actual centroid computation does not happen here. */
        void
        add (const PointT& point)
        {
          // Invoke add point on each accumulator
          boost::fusion::for_each (accumulators_, detail::AddPoint<PointT> (point));
          ++num_points_;
        }

        /** Retrieve the current centroid.
          *
          * Computation (division of accumulated values by the number of points
          * and normalization where applicable) happens here. The result is not
          * cached, so any subsequent call to this function will trigger
          * re-computation.
          *
          * If the number of accumulated points is zero, then the point will be
          * left untouched. */
        template <typename PointOutT> void
        get (PointOutT& point) const
        {
          if (num_points_ != 0)
          {
            // Filter accumulators so that only those that are compatible with
            // both PointT and requested point type remain
            typename pcl::detail::Accumulators<PointT, PointOutT>::type ca (accumulators_);
            // Invoke get point on each accumulator in filtered list
            boost::fusion::for_each (ca, detail::GetPoint<PointOutT> (point, num_points_));
          }
        }

        /** Get the total number of points that were added. */
        size_t
        getSize () const
        {
          return (num_points_);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      private:

        size_t num_points_;
        typename pcl::detail::Accumulators<PointT>::type accumulators_;

    };

  }

}

#endif /* PCL_OCTREE_CENTROID_POINT_H */

