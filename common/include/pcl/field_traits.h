/*
* SPDX-License-Identifier: BSD-3-Clause
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2025-, Open Perception Inc.
*
*  All rights reserved
*/

#pragma once

#include <type_traits> // for std::enable_if

namespace pcl
{
namespace traits
{

/** \brief Metafunction to check if a given point type has a given field.
 *
 *  Example usage at run-time:
 *
 *  \code
 *  bool curvature_available = pcl::traits::has_field<PointT, pcl::fields::curvature>::value;
 *  \endcode
 *
 *  Example usage at compile-time:
 *
 *  \code
 *  BOOST_MPL_ASSERT_MSG ((pcl::traits::has_field<PointT, pcl::fields::label>::value),
 *                        POINT_TYPE_SHOULD_HAVE_LABEL_FIELD,
 *                        (PointT));
 *  \endcode
 */
template <typename PointT, typename Field>
struct has_field;

/** Metafunction to check if a given point type has all given fields. */
template <typename PointT, typename Field>
struct has_all_fields;

/** Metafunction to check if a given point type has any of the given fields. */
template <typename PointT, typename Field>
struct has_any_field;

/** \brief Traits defined for ease of use with common fields
 *
 * has_<fields to be detected>: struct with `value` datamember defined at compiletime
 * has_<fields to be detected>_v: constexpr boolean
 * Has<Fields to be detected>: concept modelling name alias for `enable_if`
 */

/** Metafunction to check if a given point type has x and y fields. */
template <typename PointT>
struct has_xy;

template <typename PointT>
constexpr auto has_xy_v = has_xy<PointT>::value;

template <typename PointT>
using HasXY = std::enable_if_t<has_xy_v<PointT>, bool>;

template <typename PointT>
using HasNoXY = std::enable_if_t<!has_xy_v<PointT>, bool>;

/** Metafunction to check if a given point type has x, y, and z fields. */
template <typename PointT>
struct has_xyz;

template <typename PointT>
constexpr auto has_xyz_v = has_xyz<PointT>::value;

template <typename PointT>
using HasXYZ = std::enable_if_t<has_xyz_v<PointT>, bool>;

template <typename PointT>
using HasNoXYZ = std::enable_if_t<!has_xyz_v<PointT>, bool>;

/** Metafunction to check if a given point type has normal_x, normal_y, and
  * normal_z fields. */
template <typename PointT>
struct has_normal;

template <typename PointT>
constexpr auto has_normal_v = has_normal<PointT>::value;

template <typename PointT>
using HasNormal = std::enable_if_t<has_normal_v<PointT>, bool>;

template <typename PointT>
using HasNoNormal = std::enable_if_t<!has_normal_v<PointT>, bool>;

/** Metafunction to check if a given point type has curvature field. */
template <typename PointT>
struct has_curvature;

template <typename PointT>
constexpr auto has_curvature_v = has_curvature<PointT>::value;

template <typename PointT>
using HasCurvature = std::enable_if_t<has_curvature_v<PointT>, bool>;

template <typename PointT>
using HasNoCurvature = std::enable_if_t<!has_curvature_v<PointT>, bool>;

/** Metafunction to check if a given point type has intensity field. */
template <typename PointT>
struct has_intensity;

template <typename PointT>
constexpr auto has_intensity_v = has_intensity<PointT>::value;

template <typename PointT>
using HasIntensity = std::enable_if_t<has_intensity_v<PointT>, bool>;

template <typename PointT>
using HasNoIntensity = std::enable_if_t<!has_intensity_v<PointT>, bool>;

/** Metafunction to check if a given point type has either rgb or rgba field. */
template <typename PointT>
struct has_color;

template <typename PointT>
constexpr auto has_color_v = has_color<PointT>::value;

template <typename PointT>
using HasColor = std::enable_if_t<has_color_v<PointT>, bool>;

template <typename PointT>
using HasNoColor = std::enable_if_t<!has_color_v<PointT>, bool>;

/** Metafunction to check if a given point type has label field. */
template <typename PointT>
struct has_label;

template <typename PointT>
constexpr auto has_label_v = has_label<PointT>::value;

template <typename PointT>
using HasLabel = std::enable_if_t<has_label_v<PointT>, bool>;

template <typename PointT>
using HasNoLabel = std::enable_if_t<!has_label_v<PointT>, bool>;

} // namespace traits
} // namespace pcl

#include <pcl/impl/field_traits.hpp>
