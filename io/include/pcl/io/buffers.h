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

#ifndef PCL_IO_BUFFERS_H
#define PCL_IO_BUFFERS_H

#include <vector>
#include <limits>
#include <cassert>

#include <boost/cstdint.hpp>
#include <boost/thread/mutex.hpp>

namespace pcl
{

  namespace io
  {

    /** An abstract base class for fixed-size data buffers.
      *
      * A new chunk of data can be inserted using the push() method; the data
      * elements stored in the buffer can be accessed using operator[]().
      *
      * Concrete implementations of this interface (such as AverageBuffer or
      * MedianBuffer) may perform arbitrary data processing under the hood and
      * provide access to certain quantities computed based on the input data
      * rather than the data themselves.
      *
      * \author Sergey Alexandrov
      * \ingroup io */
    template <typename T>
    class Buffer
    {

      public:

        typedef T value_type;

        virtual
        ~Buffer ();

        /** Access an element at a given index. */
        virtual T
        operator[] (size_t idx) const = 0;

        /** Insert a new chunk of data into the buffer.
          *
          * Note that the \a data parameter is not `const`-qualified. This is
          * done to allow deriving classes to implement no-copy data insertion,
          * where the data is "stolen" from the input argument. */
        virtual void
        push (std::vector<T>& data) = 0;

        /** Get the size of the buffer. */
        inline size_t
        size () const
        {
          return (size_);
        }

      protected:

        Buffer (size_t size);

        const size_t size_;

    };

    /** A simple buffer that only stores data.
      *
      * The buffer is thread-safe. */
    template <typename T>
    class SingleBuffer : public Buffer<T>
    {

      public:

        /** Construct a buffer of given size. */
        SingleBuffer (size_t size);

        virtual
        ~SingleBuffer ();

        virtual T
        operator[] (size_t idx) const;

        virtual void
        push (std::vector<T>& data);

      private:

        std::vector<T> data_;
        mutable boost::mutex data_mutex_;

        using Buffer<T>::size_;

    };

    /** A buffer that computes running window median of the data inserted.
      *
      * The buffer and window sizes are specified at construction time. The
      * buffer size defines the number of elements in each data chunk that is
      * inserted in the buffer. The window size is the number of last data
      * chunks that are considered for median computation. The median is
      * computed separately for 1st, 2nd, etc. element in data chunks.
      *
      * The data can contain invalid elements. For integral types zeros are
      * assumed to be invalid elements, whereas for floating-point types it is
      * quiet NaN. Invalid elements are ignored when computing median.
      *
      * The buffer is thread-safe. */
    template <typename T>
    class MedianBuffer : public Buffer<T>
    {

      public:

        /** Construct a buffer of given size with given running window size.
          *
          * \param[in] size buffer size
          * \param[in] window_size running window size over which the median
          * value should be computed (0..255) */
        MedianBuffer (size_t size, unsigned char window_size);

        virtual
        ~MedianBuffer ();

        /** Access an element at a given index.
          *
          * This operation is constant time. */
        virtual T
        operator[] (size_t idx) const;

        /** Insert a new chunk of data into the buffer.
          *
          * This operation is linear in buffer size and window size.
          *
          * \param[in] data input data chunk, the memory will be "stolen" */
        virtual void
        push (std::vector<T>& data);

      private:

        /** Compare two data elements.
          *
          * Invalid value is assumed to be larger than everything else. If both values
          * are invalid, they are assumed to be equal.
          *
          * \return -1 if \c a < \c b, 0 if \c a == \c b, 1 if \c a > \c b */
        static int compare (T a, T b);

        const unsigned char window_size_;
        const unsigned char midpoint_;

        /// Data pushed into the buffer (last window_size_ chunks), logically
        /// organized as a circular buffer
        std::vector<std::vector<T> > data_;

        /// Index of the last pushed data chunk in the data_ circular buffer
        unsigned char data_current_idx_;

        /// Indices that the argsort function would produce for data_ (with
        /// dimensions swapped)
        std::vector<std::vector<unsigned char> > data_argsort_indices_;

        /// Number of invalid values in the buffer
        std::vector<unsigned char> data_invalid_count_;

        mutable boost::mutex data_mutex_;

        using Buffer<T>::size_;

    };

    /** A buffer that computes running window average of the data inserted.
      *
      * The buffer and window sizes are specified at construction time. The
      * buffer size defines the number of elements in each data chunk that is
      * inserted in the buffer. The window size is the number of last data
      * chunks that are considered for average computation. The average is
      * computed separately for 1st, 2nd, etc. element in data chunks.
      *
      * The data can contain invalid elements. For integral types zeros are
      * assumed to be invalid elements, whereas for floating-point types it is
      * quiet NaN. Invalid elements are ignored when computing average.
      *
      * The buffer is thread-safe. */
    template <typename T>
    class AverageBuffer : public Buffer<T>
    {

      public:

        /** Construct a buffer of given size with given running window size.
          *
          * \param[in] size buffer size
          * \param[in] window_size running window size over which the median
          * value should be computed (0..255) */
        AverageBuffer (size_t size, unsigned char window_size);

        virtual
        ~AverageBuffer ();

        /** Access an element at a given index.
          *
          * This operation is constant time. */
        virtual T
        operator[] (size_t idx) const;

        /** Insert a new chunk of data into the buffer.
          *
          * This operation is linear in buffer size.
          *
          * \param[in] data input data chunk, the memory will be "stolen" */
        virtual void
        push (std::vector<T>& data);

      private:

        const unsigned char window_size_;

        /// Data pushed into the buffer (last window_size_ chunks), logically
        /// organized as a circular buffer
        std::vector<std::vector<T> > data_;

        /// Index of the last pushed data chunk in the data_ circular buffer
        unsigned char data_current_idx_;

        /// Current sum of the buffer
        std::vector<T> data_sum_;

        /// Number of invalid values in the buffer
        std::vector<unsigned char> data_invalid_count_;

        mutable boost::mutex data_mutex_;

        using Buffer<T>::size_;

    };

  }

}

#include <pcl/io/impl/buffers.hpp>

#endif /* PCL_IO_BUFFERS_H */

