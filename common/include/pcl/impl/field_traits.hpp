/*
* SPDX-License-Identifier: BSD-3-Clause
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2025-, Open Perception Inc.
*
*  All rights reserved
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
