/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *
 * Range Coder based on Dmitry Subbotin's carry-less implementation (http://www.compression.ru/ds/)
 * Added optimized symbol lookup and added implementation for static range coding (uses fixed precomputed frequency table)
 *
 * Author: Julius Kammerl (julius@kammerl.de)
 */

#ifndef __PCL_IO_RANGECODING__
#define __PCL_IO_RANGECODING__

#include <map>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdio.h>
#include <boost/cstdint.hpp>

namespace pcl
{

  using boost::uint8_t;
  using boost::uint32_t;
  using boost::uint64_t;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b AdaptiveRangeCoder compression class
   *  \note This class provides adaptive range coding functionality.
   *  \note Its symbol probability/frequency table is adaptively updated during encoding
   *  \note
   *  \author Julius Kammerl (julius@kammerl.de)
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  class AdaptiveRangeCoder
  {

  public:

    /** \brief Empty constructor. */
    AdaptiveRangeCoder () : outputCharVector_ ()
    {
    }

    /** \brief Empty deconstructor. */
    virtual
    ~AdaptiveRangeCoder ()
    {
    }

    /** \brief Encode char vector to output stream
     * \param inputByteVector_arg input vector
     * \param outputByteStream_arg output stream containing compressed data
     * \return amount of bytes written to output stream
     */
    unsigned long
    encodeCharVectorToStream (const std::vector<char>& inputByteVector_arg, std::ostream& outputByteStream_arg);

    /** \brief Decode char stream to output vector
     * \param inputByteStream_arg input stream of compressed data
     * \param outputByteVector_arg decompressed output vector
     * \return amount of bytes read from input stream
     */
    unsigned long
    decodeStreamToCharVector (std::istream& inputByteStream_arg, std::vector<char>& outputByteVector_arg);

  protected:
    typedef boost::uint32_t DWord; // 4 bytes

  private:
    /** vector containing compressed data
     */
    std::vector<char> outputCharVector_;

  };

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b StaticRangeCoder compression class
   *  \note This class provides static range coding functionality.
   *  \note Its symbol probability/frequency table is precomputed and encoded to the output stream
   *  \note
   *  \author Julius Kammerl (julius@kammerl.de)
   */
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  class StaticRangeCoder
  {
    public:
      /** \brief Constructor. */
      StaticRangeCoder () :
        cFreqTable_ (65537), outputCharVector_ ()
      {
      }

      /** \brief Empty deconstructor. */
      virtual
      ~StaticRangeCoder ()
      {
      }

      /** \brief Encode integer vector to output stream
        * \param[in] inputIntVector_arg input vector
        * \param[out] outputByterStream_arg output stream containing compressed data
        * \return amount of bytes written to output stream
        */
      unsigned long
      encodeIntVectorToStream (std::vector<unsigned int>& inputIntVector_arg, std::ostream& outputByterStream_arg);

      /** \brief Decode stream to output integer vector
       * \param inputByteStream_arg input stream of compressed data
       * \param outputIntVector_arg decompressed output vector
       * \return amount of bytes read from input stream
       */
      unsigned long
      decodeStreamToIntVector (std::istream& inputByteStream_arg, std::vector<unsigned int>& outputIntVector_arg);

      /** \brief Encode char vector to output stream
       * \param inputByteVector_arg input vector
       * \param outputByteStream_arg output stream containing compressed data
       * \return amount of bytes written to output stream
       */
      unsigned long
      encodeCharVectorToStream (const std::vector<char>& inputByteVector_arg, std::ostream& outputByteStream_arg);

      /** \brief Decode char stream to output vector
       * \param inputByteStream_arg input stream of compressed data
       * \param outputByteVector_arg decompressed output vector
       * \return amount of bytes read from input stream
       */
      unsigned long
      decodeStreamToCharVector (std::istream& inputByteStream_arg, std::vector<char>& outputByteVector_arg);

    protected:
      typedef boost::uint32_t DWord; // 4 bytes

      /** \brief Helper function to calculate the binary logarithm
       * \param n_arg: some value
       * \return binary logarithm (log2) of argument n_arg
       */
      inline double
      Log2 (double n_arg)
      {
        return log (n_arg) / log (2.0);
      }

    private:
      /** \brief Vector containing cumulative symbol frequency table. */
      std::vector<uint64_t> cFreqTable_;

      /** \brief Vector containing compressed data. */
      std::vector<char> outputCharVector_;

  };
}


//#include "impl/entropy_range_coder.hpp"

#endif

