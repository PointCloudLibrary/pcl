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
 * $Id$
 *
 */

#ifndef PCL_CUDA_POINT_CLOUD_H_
#define PCL_CUDA_POINT_CLOUD_H_

#include <pcl/cuda/point_types.h>
#include <pcl/cuda/thrust.h>
#include <boost/shared_ptr.hpp>

namespace pcl
{
  namespace cuda
  {
    /** \brief misnamed class holding a 3x3 matrix */
    struct CovarianceMatrix
    {
      float3 data[3];
    };
  
    /** \brief Simple structure holding RGB data. */
    struct OpenNIRGB
    {
      unsigned char r, g, b;
    };
  
    /** \brief Host helper class. Contains several typedefs and some static
     *         functions to help writing portable code (that runs both on host
     *         and device) */
    template <typename T>
    struct Host
    {
      // vector type
      typedef typename thrust::host_vector<T> type;

//      // iterator type
//      typedef thrust::detail::normal_iterator<T*> type;
//      
//      // pointer type
//      typedef T* pointer_type;
//      
//      // allocator
//      static T* alloc (int size)
//      {
//        return (T*) malloc (size);
//      }
//
//      // cast to different pointer
//      template <typename U>
//      static U* cast (type ptr)
//      {
//        return (U*)ptr;
//      }
    };
  
    /** \brief Device helper class. Contains several typedefs and some static
     *         functions to help writing portable code (that runs both on host
     *         and device) */
    template <typename T>
    struct Device
    {
      // vector type
      typedef typename thrust::device_vector<T> type;
      
//      // iterator type
//      typedef thrust::detail::normal_iterator<thrust::device_ptr<T> > iterator_type;
//      
//      // pointer type
//      typedef thrust::device_ptr<T> pointer_type;
//
//      // allocator
//      static thrust::device_ptr<T> alloc (int size)
//      {
//        return thrust::device_malloc<T> (size);
//      }
//      
//      // cast to different pointer
//      template <typename U>
//      static thrust::device_ptr<U> cast (type ptr)
//      {
//        return thrust::device_ptr<U> ((U*)ptr.get());
//      }
//
//      // cast raw pointer to different pointer
//      template <typename U>
//      static thrust::device_ptr<U> cast (T* ptr)
//      {
//        return thrust::device_ptr<U> ((U*)ptr);
//      }
    };

    /** @b PointCloudAOS represents an AOS (Array of Structs) PointCloud
      * implementation for CUDA processing.
      *
      * This is the most efficient way to perform operations on x86 architectures 
      * (using SSE alignment).
      */
    template <template <typename> class Storage>
    class PointCloudAOS
    {
      public:
        PointCloudAOS () : width (0), height (0), is_dense (true)
        {}
        
        //////////////////////////////////////////////////////////////////////////////////////
        inline PointCloudAOS& operator = (const PointCloudAOS& rhs)
        {
          points   = rhs.points;
          width    = rhs.width;
          height   = rhs.height;
          is_dense = rhs.is_dense;
          return (*this);
        }
  
        //////////////////////////////////////////////////////////////////////////////////////
        template <typename OtherStorage>
        inline PointCloudAOS& operator << (const OtherStorage& rhs)
        {
          points   = rhs.points;
          // TODO: Test speed on operator () = vs resize+copy
          //points.resize (rhs.points.size ());
          //thrust::copy (rhs.points.begin (), rhs.points.end (), points.begin ());
          width    = rhs.width;
          height   = rhs.height;
          is_dense = rhs.is_dense;
          return (*this);
        }
  
        //////////////////////////////////////////////////////////////////////////////////////
        inline PointXYZRGB
        at (int u, int v) const
        {
          if (this->height > 1)
            return (points[v * this->width + u]);
          else
            return (PointXYZRGB (std::numeric_limits<float>::quiet_NaN (),
                                 std::numeric_limits<float>::quiet_NaN (),
                                 std::numeric_limits<float>::quiet_NaN (),
                                 0));
            // throw IsNotDenseException ("Can't use 2D indexing with a sparse point cloud");
        }
  
        //////////////////////////////////////////////////////////////////////////////////////
        inline PointXYZRGB& operator () (int u, int v)
        {
          return (points[v* this->width +u]);
        }
        inline const PointXYZRGB& operator () (int u, int v) const
        {
          return (points[v* this->width +u]);
        }
  
        /** \brief The point data. */
        //typename Storage<float3>::type points;
        typename Storage<PointXYZRGB>::type points;
  
        typedef typename Storage<PointXYZRGB>::type::iterator iterator;
  
        /** \brief The point cloud width (if organized as an image-structure). */
        unsigned int width;
        /** \brief The point cloud height (if organized as an image-structure). */
        unsigned int height;
  
        /** \brief True if no points are invalid (e.g., have NaN or Inf values). */
        bool is_dense;
  
        typedef boost::shared_ptr<PointCloudAOS<Storage> > Ptr;
        typedef boost::shared_ptr<const PointCloudAOS<Storage> > ConstPtr;
    };
  
    /** @b PointCloudSOA represents a SOA (Struct of Arrays) PointCloud
      * implementation for CUDA processing.
      */
    template <template <typename> class Storage>
    class PointCloudSOA
    {
      public:
        PointCloudSOA () : width (0), height (0), is_dense (true)
        {}
        
        //////////////////////////////////////////////////////////////////////////////////////
        inline PointCloudSOA& operator = (const PointCloudSOA& rhs)
        {
          points_x = rhs.points_x;
          points_y = rhs.points_y;
          points_z = rhs.points_z;
          width    = rhs.width;
          height   = rhs.height;
          is_dense = rhs.is_dense;
          return (*this);
        }
  
        //////////////////////////////////////////////////////////////////////////////////////
        template <typename OtherStorage>
        inline PointCloudSOA& operator << (const OtherStorage& rhs)
        {
          points_x = rhs.points_x;
          points_y = rhs.points_y;
          points_z = rhs.points_z;
          width    = rhs.width;
          height   = rhs.height;
          is_dense = rhs.is_dense;
          return (*this);
        }
  
        /** \brief Resize the internal point data vectors.
          * \param newsize the new size
          */
        void
        resize (size_t newsize)
        {
          assert (sane ());
          points_x.resize (newsize);
          points_y.resize (newsize);
          points_z.resize (newsize);
        }
  
        /** \brief Return the size of the internal vectors */
        std::size_t 
        size () const
        {
          assert (sane ());
          return (points_x.size ());
        }
  
        /** \brief Check if the internal pooint data vectors are valid. */
        bool 
        sane () const
        {
          return (points_x.size () == points_y.size () &&
                  points_x.size () == points_z.size ());
        }
  
        /** \brief The point data. */
        typename Storage<float>::type points_x;
        typename Storage<float>::type points_y;
        typename Storage<float>::type points_z;
        typename Storage<int>::type rgb;
  
        /** \brief The point cloud width (if organized as an image-structure). */
        unsigned int width;
        /** \brief The point cloud height (if organized as an image-structure). */
        unsigned int height;
  
        /** \brief True if no points are invalid (e.g., have NaN or Inf values). */
        bool is_dense;
  
        typedef boost::shared_ptr<PointCloudSOA<Storage> > Ptr;
        typedef boost::shared_ptr<const PointCloudSOA<Storage> > ConstPtr;
  
        //////////////////////////////////////////////////////////////////////////////////////
        // Extras. Testing ZIP iterators
        typedef thrust::tuple<float, float, float> tuple_type;
        typedef typename Storage<float>::type::iterator float_iterator;
        typedef thrust::tuple<float_iterator, float_iterator, float_iterator> iterator_tuple; 
        typedef thrust::zip_iterator<iterator_tuple> zip_iterator;
  
        zip_iterator 
        zip_begin ()
        {
          return (thrust::make_zip_iterator (thrust::make_tuple (points_x.begin (), 
                                                                 points_y.begin (), 
                                                                 points_z.begin ())));
        }
  
        zip_iterator
        zip_end ()
        {
          return (thrust::make_zip_iterator (thrust::make_tuple (points_x.end (), 
                                                                 points_y.end (), 
                                                                 points_z.end ())));
        }
    };
  
    template <template <typename> class Storage, typename T>
    struct PointIterator
    {
      typedef void type;
    };
  
    template <typename T>
    struct PointIterator<Device,T>
    {
      typedef thrust::detail::normal_iterator<thrust::device_ptr<T> > type;
    };
  
    template <typename T>
    struct PointIterator<Host,T>
    {
      typedef thrust::detail::normal_iterator<T*> type;
    };
  
    template <template <typename> class Storage, typename T>
    struct StoragePointer
    {
      // typedef void* type;
    };
  
    template <typename T>
    struct StoragePointer<Device,T>
    {
      typedef thrust::device_ptr<T> type;
      template <typename U>
      static thrust::device_ptr<U> cast (type ptr)
      {
        return thrust::device_ptr<U> ((U*)ptr.get());
      }
      template <typename U>
      static thrust::device_ptr<U> cast (T* ptr)
      {
        return thrust::device_ptr<U> ((U*)ptr);
      }
    };
  
    template <typename T>
    struct StoragePointer<Host,T>
    {
      typedef T* type;
      template <typename U>
      static U* cast (type ptr)
      {
        return (U*)ptr;
      }
    };
    template <template <typename> class Storage, typename T>
    struct StorageAllocator
    {
    };
  
    template <typename T>
    struct StorageAllocator<Device,T>
    {
      static thrust::device_ptr<T> alloc (int size)
      {
        return thrust::device_malloc<T> (size);
      }
    };
  
    template <typename T>
    struct StorageAllocator<Host,T>
    {
      static T* alloc (int size)
      {
        return (T*) malloc (size);
      }
    };
  
  
  } // namespace
} // namespace

#endif  //#ifndef PCL_CUDA_POINT_CLOUD_H_
