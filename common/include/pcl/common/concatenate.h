/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *
 */
#ifndef PCL_COMMON_CONCATENATE_H_
#define PCL_COMMON_CONCATENATE_H_

#include <pcl/conversions.h>

// We're doing a lot of black magic with Boost here, so disable warnings in Maintainer mode, as we will never
// be able to fix them anyway
#ifdef BUILD_Maintainer
#  if defined __GNUC__
#    if __GNUC__ == 4 && __GNUC_MINOR__ > 3
#      pragma GCC diagnostic ignored "-Weffc++"
#      pragma GCC diagnostic ignored "-pedantic"
#    else
#      pragma GCC system_header 
#    endif
#  elif defined _MSC_VER
#    pragma warning(push, 1)
#  endif
#endif

namespace pcl
{
  /** \brief Helper functor structure for concatenate. 
    * \ingroup common
    */
  template<typename PointInT, typename PointOutT>
  struct NdConcatenateFunctor
  {
    typedef typename traits::POD<PointInT>::type PodIn;
    typedef typename traits::POD<PointOutT>::type PodOut;
    
    NdConcatenateFunctor (const PointInT &p1, PointOutT &p2)
      : p1_ (reinterpret_cast<const PodIn&> (p1))
      , p2_ (reinterpret_cast<PodOut&> (p2)) { }

    template<typename Key> inline void 
    operator () ()
    {
      // This sucks without Fusion :(
      //boost::fusion::at_key<Key> (p2_) = boost::fusion::at_key<Key> (p1_);
      typedef typename pcl::traits::datatype<PointInT, Key>::type InT;
      typedef typename pcl::traits::datatype<PointOutT, Key>::type OutT;
      // Note: don't currently support different types for the same field (e.g. converting double to float)
      BOOST_MPL_ASSERT_MSG ((boost::is_same<InT, OutT>::value),
                            POINT_IN_AND_POINT_OUT_HAVE_DIFFERENT_TYPES_FOR_FIELD,
                            (Key, PointInT&, InT, PointOutT&, OutT));
      memcpy (reinterpret_cast<uint8_t*>(&p2_) + pcl::traits::offset<PointOutT, Key>::value,
              reinterpret_cast<const uint8_t*>(&p1_) + pcl::traits::offset<PointInT, Key>::value,
              sizeof (InT));
    }

    private:
      const PodIn &p1_;
      PodOut &p2_;
  };
}

#ifdef BUILD_Maintainer
#  if defined __GNUC__
#    if __GNUC__ == 4 && __GNUC_MINOR__ > 3
#      pragma GCC diagnostic warning "-Weffc++"
#      pragma GCC diagnostic warning "-pedantic"
#    endif
#  elif defined _MSC_VER
#    pragma warning(pop)
#  endif
#endif

#endif // PCL_COMMON_CONCATENATE_H_

