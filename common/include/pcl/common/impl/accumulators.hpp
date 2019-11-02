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

#ifndef PCL_COMMON_IMPL_DETAIL_ACCUMULATORS_HPP
#define PCL_COMMON_IMPL_DETAIL_ACCUMULATORS_HPP

#include <map>

#include <boost/mpl/filter_view.hpp>
#include <boost/fusion/include/mpl.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/as_vector.hpp>
#include <boost/fusion/include/filter_if.hpp>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

namespace pcl
{

  namespace detail
  {

    /* Below are several helper accumulator structures that are used by the
     * `CentroidPoint` class. Each of them is capable of accumulating
     * information from a particular field(s) of a point. The points are
     * inserted via `add()` and extracted via `get()` functions. Note that the
     * accumulators are not templated on point type, so in principle it is
     * possible to insert and extract points of different types. It is the
     * responsibility of the user to make sure that points have corresponding
     * fields. */

    struct AccumulatorXYZ
    {

      // Requires that point type has x, y, and z fields
      using IsCompatible = pcl::traits::has_xyz<boost::mpl::_1>;

      // Storage
      Eigen::Vector3f xyz = Eigen::Vector3f::Zero ();

      template <typename PointT> void
      add (const PointT& t) { xyz += t.getVector3fMap (); }

      template <typename PointT> void
      get (PointT& t, std::size_t n) const { t.getVector3fMap () = xyz / n; }

      PCL_MAKE_ALIGNED_OPERATOR_NEW

    };

    struct AccumulatorNormal
    {

      // Requires that point type has normal_x, normal_y, and normal_z fields
      using IsCompatible = pcl::traits::has_normal<boost::mpl::_1>;

      // Storage
      Eigen::Vector4f normal = Eigen::Vector4f::Zero ();

      // Requires that the normal of the given point is normalized, otherwise it
      // does not make sense to sum it up with the accumulated value.
      template <typename PointT> void
      add (const PointT& t) { normal += t.getNormalVector4fMap (); }

      template <typename PointT> void
      get (PointT& t, std::size_t) const
      {
#if EIGEN_VERSION_AT_LEAST (3, 3, 0)
        t.getNormalVector4fMap () = normal.normalized ();
#else
        if (normal.squaredNorm() > 0)
          t.getNormalVector4fMap () = normal.normalized ();
        else
          t.getNormalVector4fMap () = Eigen::Vector4f::Zero ();
#endif
      }

      PCL_MAKE_ALIGNED_OPERATOR_NEW

    };

    struct AccumulatorCurvature
    {

      // Requires that point type has curvature field
      using IsCompatible = pcl::traits::has_curvature<boost::mpl::_1>;

      // Storage
      float curvature = 0;

      template <typename PointT> void
      add (const PointT& t) { curvature += t.curvature; }

      template <typename PointT> void
      get (PointT& t, std::size_t n) const { t.curvature = curvature / n; }

    };

    struct AccumulatorRGBA
    {

      // Requires that point type has rgb or rgba field
      using IsCompatible = pcl::traits::has_color<boost::mpl::_1>;

      // Storage
      float r = 0, g = 0, b = 0, a = 0;

      template <typename PointT> void
      add (const PointT& t)
      {
        r += static_cast<float> (t.r);
        g += static_cast<float> (t.g);
        b += static_cast<float> (t.b);
        a += static_cast<float> (t.a);
      }

      template <typename PointT> void
      get (PointT& t, std::size_t n) const
      {
        t.rgba = static_cast<std::uint32_t> (a / n) << 24 |
                 static_cast<std::uint32_t> (r / n) << 16 |
                 static_cast<std::uint32_t> (g / n) <<  8 |
                 static_cast<std::uint32_t> (b / n);
      }

    };

    struct AccumulatorIntensity
    {

      // Requires that point type has intensity field
      using IsCompatible = pcl::traits::has_intensity<boost::mpl::_1>;

      // Storage
      float intensity = 0;

      template <typename PointT> void
      add (const PointT& t) { intensity += t.intensity; }

      template <typename PointT> void
      get (PointT& t, std::size_t n) const { t.intensity = intensity / n; }

    };

    struct AccumulatorLabel
    {

      // Requires that point type has label field
      using IsCompatible = pcl::traits::has_label<boost::mpl::_1>;

      // Storage
      // A better performance may be achieved with a heap structure
      std::map<std::uint32_t, std::size_t> labels;

      template <typename PointT> void
      add (const PointT& t)
      {
        auto itr = labels.find (t.label);
        if (itr == labels.end ())
          labels.insert (std::make_pair (t.label, 1));
        else
          ++itr->second;
      }

      template <typename PointT> void
      get (PointT& t, std::size_t) const
      {
        std::size_t max = 0;
        for (const auto &label : labels)
          if (label.second > max)
          {
            max = label.second;
            t.label = label.first;
          }
      }

    };

    /* Meta-function that checks if an accumulator is compatible with given
     * point type(s). */
    template <typename Point1T, typename Point2T = Point1T>
    struct IsAccumulatorCompatible {

      template <typename AccumulatorT>
      struct apply : boost::mpl::and_<
                       boost::mpl::apply<typename AccumulatorT::IsCompatible, Point1T>,
                       boost::mpl::apply<typename AccumulatorT::IsCompatible, Point2T>
                     > {};
    };

    /* Meta-function that creates a Fusion vector of accumulator types that are
     * compatible with a given point type. */
    template <typename PointT>
    struct Accumulators
    {
      using type =
        typename boost::fusion::result_of::as_vector<
          typename boost::mpl::filter_view<
            boost::mpl::vector<
              AccumulatorXYZ
            , AccumulatorNormal
            , AccumulatorCurvature
            , AccumulatorRGBA
            , AccumulatorIntensity
            , AccumulatorLabel
            >
          , IsAccumulatorCompatible<PointT>
          >
        >::type;
    };

    /* Fusion function object to invoke point addition on every accumulator in
     * a fusion sequence. */
    template <typename PointT>
    struct AddPoint
    {

      const PointT& p;

      AddPoint (const PointT& point) : p (point) { }

      template <typename AccumulatorT> void
      operator () (AccumulatorT& accumulator) const
      {
        accumulator.add (p);
      }

    };

    /* Fusion function object to invoke get point on every accumulator in a
     * fusion sequence. */
    template <typename PointT>
    struct GetPoint
    {

      PointT& p;
      std::size_t n;

      GetPoint (PointT& point, std::size_t num) : p (point), n (num) { }

      template <typename AccumulatorT> void
      operator () (AccumulatorT& accumulator) const
      {
        accumulator.get (p, n);
      }

    };

  }

}

#endif /* PCL_COMMON_IMPL_DETAIL_ACCUMULATORS_HPP */

