/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2025-, Open Perception, Inc.
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

#include <pcl/point_struct_traits.h>    // for pcl::traits::fieldList

// Forward declarations of common pcl field types
namespace pcl
{
namespace fields
{
struct x;
struct y;
struct z;
struct normal_x;
struct normal_y;
struct normal_z;
struct curvature;
struct intensity;
struct rgb;
struct rgba;
struct label;
} // namespace fields
} // namespace pcl

#include <boost/mpl/and.hpp>            // for boost::mpl::and_
#include <boost/mpl/bool.hpp>           // for boost::mpl::bool_
#include <boost/mpl/contains.hpp>       // for boost::mpl::contains
#include <boost/mpl/fold.hpp>           // for boost::mpl::fold
#include <boost/mpl/or.hpp>             // for boost::mpl::or_
#include <boost/mpl/placeholders.hpp>   // for boost::mpl::_1, boost::mpl::_2
#include <boost/mpl/vector.hpp>         // for boost::mpl::vector

namespace pcl
{
namespace traits
{

template <typename PointT, typename Field>
struct has_field : boost::mpl::contains<typename pcl::traits::fieldList<PointT>::type, Field>::type
{ };

template <typename PointT, typename Field>
struct has_all_fields : boost::mpl::fold<Field,
                                         boost::mpl::bool_<true>,
                                         boost::mpl::and_<boost::mpl::_1,
                                                          has_field<PointT, boost::mpl::_2> > >::type
{ };

template <typename PointT, typename Field>
struct has_any_field : boost::mpl::fold<Field,
                                        boost::mpl::bool_<false>,
                                        boost::mpl::or_<boost::mpl::_1,
                                                        has_field<PointT, boost::mpl::_2> > >::type
{ };

template <typename PointT>
struct has_xy : has_all_fields<PointT, boost::mpl::vector<pcl::fields::x,
                                                          pcl::fields::y> >
{ };

template <typename PointT>
struct has_xyz : has_all_fields<PointT, boost::mpl::vector<pcl::fields::x,
                                                           pcl::fields::y,
                                                           pcl::fields::z> >
{ };

template <typename PointT>
struct has_normal : has_all_fields<PointT, boost::mpl::vector<pcl::fields::normal_x,
                                                              pcl::fields::normal_y,
                                                              pcl::fields::normal_z> >
{ };

template <typename PointT>
struct has_curvature : has_field<PointT, pcl::fields::curvature>
{ };

template <typename PointT>
struct has_intensity : has_field<PointT, pcl::fields::intensity>
{ };

template <typename PointT>
struct has_color : has_any_field<PointT, boost::mpl::vector<pcl::fields::rgb,
                                                            pcl::fields::rgba> >
{ };

template <typename PointT>
struct has_label : has_field<PointT, pcl::fields::label>
{ };

} // namespace traits
} // namespace pcl
