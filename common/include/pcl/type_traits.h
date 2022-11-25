/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2020-, Open Perception, Inc.
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

#include <pcl/point_struct_traits.h>  // for pcl::traits::POD, pcl::traits::name, pcl::traits::datatype, pcl::traits::offset

#include <cstddef>  // for std::size_t
#include <cstdint>  // for std::uint8_t

#include <functional>   // for std::function, needed till C++17
#include <string>       // for std::string
#include <type_traits>  // for std::false_type, std::true_type

namespace pcl
{
  namespace deprecated
  {
    /** \class DeprecatedType
    * \brief A dummy type to aid in template parameter deprecation
    */
    struct T {};
  }

  namespace fields
  {
    // Tag types get put in this namespace
  }

  namespace traits
  {
    namespace detail {
    /**
     * \brief Enumeration for different numerical types
     *
     * \details struct used to enable scope and implicit conversion to int
     */
    struct PointFieldTypes {
        static const std::uint8_t INT8 = 1,    UINT8 = 2,
                                  INT16 = 3,   UINT16 = 4,
                                  INT32 = 5,   UINT32 = 6,
                                  FLOAT32 = 7, FLOAT64 = 8,
                                  INT64 = 9,   UINT64 = 10,
                                  BOOL = 11;
    };
    }  // namespace detail

    // Metafunction to return enum value representing a type
    template<typename T> struct asEnum {};
    template<> struct asEnum<bool>          { static const std::uint8_t value = detail::PointFieldTypes::BOOL; };
    template<> struct asEnum<std::int8_t>   { static const std::uint8_t value = detail::PointFieldTypes::INT8;    };
    template<> struct asEnum<std::uint8_t>  { static const std::uint8_t value = detail::PointFieldTypes::UINT8;   };
    template<> struct asEnum<std::int16_t>  { static const std::uint8_t value = detail::PointFieldTypes::INT16;   };
    template<> struct asEnum<std::uint16_t> { static const std::uint8_t value = detail::PointFieldTypes::UINT16;  };
    template<> struct asEnum<std::int32_t>  { static const std::uint8_t value = detail::PointFieldTypes::INT32;   };
    template<> struct asEnum<std::uint32_t> { static const std::uint8_t value = detail::PointFieldTypes::UINT32;  };
    template<> struct asEnum<std::int64_t>  { static const std::uint8_t value = detail::PointFieldTypes::INT64;   };
    template<> struct asEnum<std::uint64_t> { static const std::uint8_t value = detail::PointFieldTypes::UINT64;  };
    template<> struct asEnum<float>         { static const std::uint8_t value = detail::PointFieldTypes::FLOAT32; };
    template<> struct asEnum<double>        { static const std::uint8_t value = detail::PointFieldTypes::FLOAT64; };

    template<typename T>
    static constexpr std::uint8_t asEnum_v = asEnum<T>::value;

    // Metafunction to return type of enum value
    template<int> struct asType {};
    template<> struct asType<detail::PointFieldTypes::BOOL>    { using type = bool; };
    template<> struct asType<detail::PointFieldTypes::INT8>    { using type = std::int8_t; };
    template<> struct asType<detail::PointFieldTypes::UINT8>   { using type = std::uint8_t; };
    template<> struct asType<detail::PointFieldTypes::INT16>   { using type = std::int16_t; };
    template<> struct asType<detail::PointFieldTypes::UINT16>  { using type = std::uint16_t; };
    template<> struct asType<detail::PointFieldTypes::INT32>   { using type = std::int32_t; };
    template<> struct asType<detail::PointFieldTypes::UINT32>  { using type = std::uint32_t; };
    template<> struct asType<detail::PointFieldTypes::INT64>   { using type = std::int64_t; };
    template<> struct asType<detail::PointFieldTypes::UINT64>  { using type = std::uint64_t; };
    template<> struct asType<detail::PointFieldTypes::FLOAT32> { using type = float; };
    template<> struct asType<detail::PointFieldTypes::FLOAT64> { using type = double; };

    template<int index>
    using asType_t = typename asType<index>::type;

  } // namespace traits

  /** \brief A helper functor that can copy a specific value if the given field exists.
    *
    * \note In order to actually copy the value an instance of this functor should be passed
    * to a pcl::for_each_type loop. See the example below.
    *
    * \code
    * PointInT p;
    * bool exists;
    * float value;
    * using FieldList = typename pcl::traits::fieldList<PointInT>::type;
    * pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<PointT, float> (p, "intensity", exists, value));
    * \endcode
    */
  template <typename PointInT, typename OutT>
  struct CopyIfFieldExists
  {
    using Pod = typename traits::POD<PointInT>::type;

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
        using T = typename pcl::traits::datatype<PointInT, Key>::type;
        const std::uint8_t* data_ptr = reinterpret_cast<const std::uint8_t*>(&pt_) + pcl::traits::offset<PointInT, Key>::value;
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
    * using FieldList = typename pcl::traits::fieldList<PointT>::type;
    * pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<PointT, float> (p, "intensity", 42.0f));
    * \endcode
    */
  template <typename PointOutT, typename InT>
  struct SetIfFieldExists
  {
    using Pod = typename traits::POD<PointOutT>::type;

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
        using T = typename pcl::traits::datatype<PointOutT, Key>::type;
        std::uint8_t* data_ptr = reinterpret_cast<std::uint8_t*>(&pt_) + pcl::traits::offset<PointOutT, Key>::value;
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
  setFieldValue (PointT &pt, std::size_t field_offset, const ValT &value)
  {
    std::uint8_t* data_ptr = reinterpret_cast<std::uint8_t*>(&pt) + field_offset;
    *reinterpret_cast<ValT*>(data_ptr) = value;
  }

  /** \brief Get the value at a specified field in a point
    * \param[in] pt the point to get the value from
    * \param[in] field_offset the offset of the field
    * \param[out] value the value to retrieve
    */
  template <typename PointT, typename ValT> inline void
  getFieldValue (const PointT &pt, std::size_t field_offset, ValT &value)
  {
    const std::uint8_t* data_ptr = reinterpret_cast<const std::uint8_t*>(&pt) + field_offset;
    value = *reinterpret_cast<const ValT*>(data_ptr);
  }

  template <typename ...> using void_t = void; // part of std in c++17

#ifdef DOXYGEN_ONLY

  /**
   * \brief Tests at compile time if type T has a custom allocator
   *
   * \see pcl::make_shared, PCL_MAKE_ALIGNED_OPERATOR_NEW
   * \tparam T Type of the object to test
   */
  template <typename T> struct has_custom_allocator;

#else

  template <typename, typename = void_t<>> struct has_custom_allocator : std::false_type {};
  template <typename T> struct has_custom_allocator<T, void_t<typename T::_custom_allocator_type_trait>> : std::true_type {};

#endif

  /**
   * \todo: Remove in C++17
   */
#ifndef __cpp_lib_is_invocable
  // Implementation taken from: https://stackoverflow.com/a/51188325
  template <typename F, typename... Args>
  constexpr bool is_invocable_v =
      std::is_constructible<std::function<void(Args...)>,
                            std::reference_wrapper<std::remove_reference_t<F>>>::value;

  template <typename R, typename F, typename... Args>
  constexpr bool is_invocable_r_v =
      std::is_constructible<std::function<R(Args...)>,
                            std::reference_wrapper<std::remove_reference_t<F>>>::value;
#else
  using std::is_invocable_v;
  using std::is_invocable_r_v;
#endif

  /**
   * \todo: Remove in C++17
   */
#ifndef __cpp_lib_remove_cvref
  template <typename T>
  using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;
#else
  using std::remove_cvref_t;
#endif
}
