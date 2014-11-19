/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 */

#ifndef PCL_POINT_TRAITS_H_
#define PCL_POINT_TRAITS_H_

#ifdef __GNUC__
#pragma GCC system_header
#endif

#include "pcl/pcl_macros.h"

#include <pcl/PCLPointField.h>
#include <boost/type_traits/remove_all_extents.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/mpl/assert.hpp>
#if PCL_LINEAR_VERSION(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__) == PCL_LINEAR_VERSION(4,4,3)
#include <boost/mpl/bool.hpp>
#endif

// This is required for the workaround at line 109
#ifdef _MSC_VER
#include <Eigen/Core>
#include <Eigen/src/StlSupport/details.h>
#endif

namespace pcl
{

  namespace fields
  {
    // Tag types get put in this namespace
  }

  namespace traits
  {
    // Metafunction to return enum value representing a type
    template<typename T> struct asEnum {};
    template<> struct asEnum<int8_t>   { static const uint8_t value = pcl::PCLPointField::INT8;    };
    template<> struct asEnum<uint8_t>  { static const uint8_t value = pcl::PCLPointField::UINT8;   };
    template<> struct asEnum<int16_t>  { static const uint8_t value = pcl::PCLPointField::INT16;   };
    template<> struct asEnum<uint16_t> { static const uint8_t value = pcl::PCLPointField::UINT16;  };
    template<> struct asEnum<int32_t>  { static const uint8_t value = pcl::PCLPointField::INT32;   };
    template<> struct asEnum<uint32_t> { static const uint8_t value = pcl::PCLPointField::UINT32;  };
    template<> struct asEnum<float>    { static const uint8_t value = pcl::PCLPointField::FLOAT32; };
    template<> struct asEnum<double>   { static const uint8_t value = pcl::PCLPointField::FLOAT64; };

    // Metafunction to return type of enum value
    template<int> struct asType {};
    template<> struct asType<pcl::PCLPointField::INT8>    { typedef int8_t   type; };
    template<> struct asType<pcl::PCLPointField::UINT8>   { typedef uint8_t  type; };
    template<> struct asType<pcl::PCLPointField::INT16>   { typedef int16_t  type; };
    template<> struct asType<pcl::PCLPointField::UINT16>  { typedef uint16_t type; };
    template<> struct asType<pcl::PCLPointField::INT32>   { typedef int32_t  type; };
    template<> struct asType<pcl::PCLPointField::UINT32>  { typedef uint32_t type; };
    template<> struct asType<pcl::PCLPointField::FLOAT32> { typedef float    type; };
    template<> struct asType<pcl::PCLPointField::FLOAT64> { typedef double   type; };

    // Metafunction to decompose a type (possibly of array of any number of dimensions) into
    // its scalar type and total number of elements.
    template<typename T> struct decomposeArray
    {
      typedef typename boost::remove_all_extents<T>::type type;
      static const uint32_t value = sizeof (T) / sizeof (type);
    };

    // For non-POD point types, this is specialized to return the corresponding POD type.
    template<typename PointT>
    struct POD
    {
      typedef PointT type;
    };

#ifdef _MSC_VER

    /* Sometimes when calling functions like `copyPoint()` or `copyPointCloud`
     * without explicitly specifying point types, MSVC deduces them to be e.g.
     * `Eigen::internal::workaround_msvc_stl_support<pcl::PointXYZ>` instead of
     * plain `pcl::PointXYZ`. Subsequently these types are passed to meta-
     * functions like `has_field` or `fieldList` and make them choke. This hack
     * makes use of the fact that internally `fieldList` always applies `POD` to
     * its argument type. This specialization therefore allows to unwrap the
     * contained point type. */
    template<typename PointT>
    struct POD<Eigen::internal::workaround_msvc_stl_support<PointT> >
    {
      typedef PointT type;
    };

#endif

    // name
    /* This really only depends on Tag, but we go through some gymnastics to avoid ODR violations.
       We template it on the point type PointT to avoid ODR violations when registering multiple
       point types with shared tags.
       The dummy parameter is so we can partially specialize name on PointT and Tag but leave it
       templated on dummy. Each specialization declares a static char array containing the tag
       name. The definition of the static member would conflict when linking multiple translation
       units that include the point type registration. But when the static member definition is
       templated (on dummy), we sidestep the ODR issue.
    */
    template<class PointT, typename Tag, int dummy = 0>
    struct name : name<typename POD<PointT>::type, Tag, dummy>
    {
      // Contents of specialization:
      // static const char value[];

      // Avoid infinite compile-time recursion
      BOOST_MPL_ASSERT_MSG((!boost::is_same<PointT, typename POD<PointT>::type>::value),
                           POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
    };

    // offset
    template<class PointT, typename Tag>
    struct offset : offset<typename POD<PointT>::type, Tag>
    {
      // Contents of specialization:
      // static const size_t value;

      // Avoid infinite compile-time recursion
      BOOST_MPL_ASSERT_MSG((!boost::is_same<PointT, typename POD<PointT>::type>::value),
                           POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
    };

    // datatype
    template<class PointT, typename Tag>
    struct datatype : datatype<typename POD<PointT>::type, Tag>
    {
      // Contents of specialization:
      // typedef ... type;
      // static const uint8_t value;
      // static const uint32_t size;

      // Avoid infinite compile-time recursion
      BOOST_MPL_ASSERT_MSG((!boost::is_same<PointT, typename POD<PointT>::type>::value),
                           POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
    };

    // fields
    template<typename PointT>
    struct fieldList : fieldList<typename POD<PointT>::type>
    {
      // Contents of specialization:
      // typedef boost::mpl::vector<...> type;

      // Avoid infinite compile-time recursion
      BOOST_MPL_ASSERT_MSG((!boost::is_same<PointT, typename POD<PointT>::type>::value),
                           POINT_TYPE_NOT_PROPERLY_REGISTERED, (PointT&));
    };
#if PCL_LINEAR_VERSION(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__) == PCL_LINEAR_VERSION(4,4,3)
    /*
      At least on GCC 4.4.3, but not later versions, some valid usages of the above traits for
      non-POD (but registered) point types fail with:
      error: ‘!(bool)mpl_::bool_<false>::value’ is not a valid template argument for type ‘bool’ because it is a non-constant expression

      "Priming the pump" with the trivial assertion below somehow fixes the problem...
     */
    //BOOST_MPL_ASSERT_MSG((!bool (mpl_::bool_<false>::value)), WTF_GCC443, (bool));
    BOOST_MPL_ASSERT_MSG((!bool (boost::mpl::bool_<false>::value)), WTF_GCC443, (bool));
#endif
  } //namespace traits

  // Return true if the PCLPointField matches the expected name and data type.
  // Written as a struct to allow partially specializing on Tag.
  template<typename PointT, typename Tag>
  struct FieldMatches
  {
    bool operator() (const pcl::PCLPointField& field)
    {
      return (field.name == traits::name<PointT, Tag>::value &&
              field.datatype == traits::datatype<PointT, Tag>::value &&
              (field.count == traits::datatype<PointT, Tag>::size ||
               field.count == 0 && traits::datatype<PointT, Tag>::size == 1 /* see bug #821 */));
    }
  };

  /** \brief A helper functor that can copy a specific value if the given field exists.
    *
    * \note In order to actually copy the value an instance of this functor should be passed
    * to a pcl::for_each_type loop. See the example below.
    *
    * \code
    * PointInT p;
    * bool exists;
    * float value;
    * typedef typename pcl::traits::fieldList<PointInT>::type FieldList;
    * pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<PointT, float> (p, "intensity", exists, value));
    * \endcode
    */
  template <typename PointInT, typename OutT>
  struct CopyIfFieldExists
  {
    typedef typename traits::POD<PointInT>::type Pod;

    /** \brief Constructor.
      * \param[in] pt the input point
      * \param[in] field the name of the field
      * \param[out] exists set to true if the field exists, false otherwise
      * \param[out] value the copied field value
      */
    CopyIfFieldExists (const PointInT &pt,
                       const std::string &field,
                       bool &exists,
                       OutT &value)
      : pt_ (reinterpret_cast<const Pod&>(pt)), name_ (field), exists_ (exists), value_ (value)
    {
      exists_ = false;
    }

    /** \brief Constructor.
      * \param[in] pt the input point
      * \param[in] field the name of the field
      * \param[out] value the copied field value
      */
    CopyIfFieldExists (const PointInT &pt,
                       const std::string &field,
                       OutT &value)
      : pt_ (reinterpret_cast<const Pod&>(pt)), name_ (field), exists_ (exists_tmp_), value_ (value)
    {
    }

    /** \brief Operator. Data copy happens here. */
    template <typename Key> inline void
    operator() ()
    {
      if (name_ == pcl::traits::name<PointInT, Key>::value)
      {
        exists_ = true;
        typedef typename pcl::traits::datatype<PointInT, Key>::type T;
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&pt_) + pcl::traits::offset<PointInT, Key>::value;
        value_ = static_cast<OutT> (*reinterpret_cast<const T*>(data_ptr));
      }
    }

    private:
      const Pod &pt_;
      const std::string &name_;
      bool &exists_;
      // Bogus entry
      bool exists_tmp_;
      OutT &value_;
  };

  /** \brief A helper functor that can set a specific value in a field if the field exists.
    *
    * \note In order to actually set the value an instance of this functor should be passed
    * to a pcl::for_each_type loop. See the example below.
    *
    * \code
    * PointT p;
    * typedef typename pcl::traits::fieldList<PointT>::type FieldList;
    * pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT, float> (p, "intensity", 42.0f));
    * \endcode
    */
  template <typename PointOutT, typename InT>
  struct SetIfFieldExists
  {
    typedef typename traits::POD<PointOutT>::type Pod;

    /** \brief Constructor.
      * \param[in] pt the input point
      * \param[in] field the name of the field
      * \param[out] value the value to set
      */
    SetIfFieldExists (PointOutT &pt,
                      const std::string &field,
                      const InT &value)
      : pt_ (reinterpret_cast<Pod&>(pt)), name_ (field), value_ (value)
    {
    }

    /** \brief Operator. Data copy happens here. */
    template <typename Key> inline void
    operator() ()
    {
      if (name_ == pcl::traits::name<PointOutT, Key>::value)
      {
        typedef typename pcl::traits::datatype<PointOutT, Key>::type T;
        uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&pt_) + pcl::traits::offset<PointOutT, Key>::value;
        *reinterpret_cast<T*>(data_ptr) = static_cast<T> (value_);
      }
    }

    private:
      Pod &pt_;
      const std::string &name_;
      const InT &value_;
  };

  /** \brief Set the value at a specified field in a point
    * \param[out] pt the point to set the value to
    * \param[in] field_offset the offset of the field
    * \param[in] value the value to set
    */
  template <typename PointT, typename ValT> inline void
  setFieldValue (PointT &pt, size_t field_offset, const ValT &value)
  {
    uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&pt) + field_offset;
    *reinterpret_cast<ValT*>(data_ptr) = value;
  }

  /** \brief Get the value at a specified field in a point
    * \param[in] pt the point to get the value from
    * \param[in] field_offset the offset of the field
    * \param[out] value the value to retreive
    */
  template <typename PointT, typename ValT> inline void
  getFieldValue (const PointT &pt, size_t field_offset, ValT &value)
  {
    const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&pt) + field_offset;
    value = *reinterpret_cast<const ValT*>(data_ptr);
  }
}

#endif  //#ifndef PCL_POINT_TRAITS_H_
