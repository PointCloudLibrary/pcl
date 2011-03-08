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
 * $Id: for_each_type.h 33238 2010-03-11 00:46:58Z rusu $
 *
 */
/**

\author Patrick Mihelich

Helper class.

**/

#ifndef PCL_FOR_EACH_TYPE_H_
#define PCL_FOR_EACH_TYPE_H_

#include <boost/mpl/is_sequence.hpp>
#include <boost/mpl/begin_end.hpp>
#include <boost/mpl/next_prior.hpp>
#include <boost/mpl/deref.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/aux_/unwrap.hpp>
#include <boost/type_traits/is_same.hpp>

namespace pcl 
{
  template< bool done = true >
  struct for_each_type_impl
  {
    template<typename Iterator, typename LastIterator, typename F>
    static void execute (F) {}
  };

  template<>
  struct for_each_type_impl<false>
  {
    template<typename Iterator, typename LastIterator, typename F>
    static void execute (F f)
    {
      typedef typename boost::mpl::deref<Iterator>::type arg;

#ifdef _WIN32
      boost::mpl::aux::unwrap(f, 0).operator()<arg> ();
#else
      boost::mpl::aux::unwrap(f, 0).template operator()<arg> ();
#endif

      typedef typename boost::mpl::next<Iterator>::type iter;
      for_each_type_impl<boost::is_same<iter, LastIterator>::value>
        ::template execute<iter, LastIterator, F> (f);
    }
  };

  template<typename Sequence, typename F>
  void for_each_type (F f)
  {
    BOOST_MPL_ASSERT (( boost::mpl::is_sequence<Sequence> ));
    typedef typename boost::mpl::begin<Sequence>::type first;
    typedef typename boost::mpl::end<Sequence>::type last;
    for_each_type_impl< boost::is_same<first, last>::value >::template execute<first, last, F> (f);
  }
}

#endif  //#ifndef PCL_FOR_EACH_TYPE_H_
