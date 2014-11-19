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

#ifndef PCL_COMMON_IMPL_COPY_POINT_HPP_
#define PCL_COMMON_IMPL_COPY_POINT_HPP_

#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/common/concatenate.h>

namespace pcl
{

  namespace detail
  {

    /* CopyPointHelper and its specializations copy the contents of a source
     * point to a target point. There are three cases:
     *
     *  - Points have the same type.
     *    In this case a single `memcpy` is used.
     *
     *  - Points have different types and one of the following is true:
     *      * both have RGB fields;
     *      * both have RGBA fields;
     *      * one or both have no RGB/RGBA fields.
     *    In this case we find the list of common fields and copy their
     *    contents one by one with `NdConcatenateFunctor`.
     *
     *  - Points have different types and one of these types has RGB field, and
     *    the other has RGBA field.
     *    In this case we also find the list of common fields and copy their
     *    contents. In order to account for the fact that RGB and RGBA do not
     *    match we have an additional `memcpy` to copy the contents of one into
     *    another.
     *
     * An appropriate version of CopyPointHelper is instantiated during
     * compilation time automatically, so there is absolutely no run-time
     * overhead. */

    template <typename PointInT, typename PointOutT, typename Enable = void>
    struct CopyPointHelper { };

    template <typename PointInT, typename PointOutT>
    struct CopyPointHelper<PointInT, PointOutT, typename boost::enable_if<boost::is_same<PointInT, PointOutT> >::type>
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
        memcpy (&point_out, &point_in, sizeof (PointInT));
      }
    };

    template <typename PointInT, typename PointOutT>
    struct CopyPointHelper<PointInT, PointOutT,
                           typename boost::enable_if<boost::mpl::and_<boost::mpl::not_<boost::is_same<PointInT, PointOutT> >,
                                                                      boost::mpl::or_<boost::mpl::not_<pcl::traits::has_color<PointInT> >,
                                                                                      boost::mpl::not_<pcl::traits::has_color<PointOutT> >,
                                                                                      boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgb>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgb> >,
                                                                                      boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgba>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgba> > > > >::type>
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
        typedef typename pcl::traits::fieldList<PointInT>::type FieldListInT;
        typedef typename pcl::traits::fieldList<PointOutT>::type FieldListOutT;
        typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (point_in, point_out));
      }
    };

    template <typename PointInT, typename PointOutT>
    struct CopyPointHelper<PointInT, PointOutT,
                           typename boost::enable_if<boost::mpl::and_<boost::mpl::not_<boost::is_same<PointInT, PointOutT> >,
                                                                      boost::mpl::or_<boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgb>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgba> >,
                                                                                      boost::mpl::and_<pcl::traits::has_field<PointInT, pcl::fields::rgba>,
                                                                                                       pcl::traits::has_field<PointOutT, pcl::fields::rgb> > > > >::type>
    {
      void operator () (const PointInT& point_in, PointOutT& point_out) const
      {
        typedef typename pcl::traits::fieldList<PointInT>::type FieldListInT;
        typedef typename pcl::traits::fieldList<PointOutT>::type FieldListOutT;
        typedef typename pcl::intersect<FieldListInT, FieldListOutT>::type FieldList;
        const uint32_t offset_in  = boost::mpl::if_<pcl::traits::has_field<PointInT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointInT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointInT, pcl::fields::rgba> >::type::value;
        const uint32_t offset_out = boost::mpl::if_<pcl::traits::has_field<PointOutT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointOutT, pcl::fields::rgb>,
                                                    pcl::traits::offset<PointOutT, pcl::fields::rgba> >::type::value;
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (point_in, point_out));
        memcpy (reinterpret_cast<char*> (&point_out) + offset_out,
                reinterpret_cast<const char*> (&point_in) + offset_in,
                4);
      }
    };

  }

}

template <typename PointInT, typename PointOutT> void
pcl::copyPoint (const PointInT& point_in, PointOutT& point_out)
{
  detail::CopyPointHelper<PointInT, PointOutT> copy;
  copy (point_in, point_out);
}

#endif //PCL_COMMON_IMPL_COPY_POINT_HPP_

