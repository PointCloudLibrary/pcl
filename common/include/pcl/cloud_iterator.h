/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 *
 */

#ifndef PCL_POINT_CLOUD_ITERATOR_H_
#define PCL_POINT_CLOUD_ITERATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/correspondence.h>

namespace pcl
{
  /** \brief Iterator class for point clouds with or without given indices
    * \author Suat Gedikli
    */
  template <typename PointT>
  class CloudIterator
  {
    public:
      CloudIterator (PointCloud<PointT>& cloud);

      CloudIterator (PointCloud<PointT>& cloud, const std::vector<int>& indices);

      CloudIterator (PointCloud<PointT>& cloud, const PointIndices& indices);

      CloudIterator (PointCloud<PointT>& cloud, const Correspondences& corrs, bool source);

      ~CloudIterator ();

      void operator ++ ();

      void operator ++ (int);

      PointT& operator* () const;

      PointT* operator-> () const;

      unsigned getCurrentPointIndex () const;

      unsigned getCurrentIndex () const;

      /** \brief Size of the range the iterator is going through. Depending on how the CloudIterator was constructed this is the size of the cloud or indices/correspondences. */
      size_t size () const;

      void reset ();

      bool isValid () const;

      operator bool () const
      {
        return isValid ();
      }
    private:

      class Iterator
      {
        public:
          virtual ~Iterator ()  {}

          virtual void operator ++ () = 0;

          virtual void operator ++ (int) = 0;

          virtual PointT& operator* () const = 0;

          virtual PointT* operator-> () const = 0;

          virtual unsigned getCurrentPointIndex () const = 0;

          virtual unsigned getCurrentIndex () const = 0;

          /** \brief Size of the range the iterator is going through. Depending on how the CloudIterator was constructed this is the size of the cloud or indices/correspondences. */
          virtual size_t size () const = 0;

          virtual void reset () = 0;

          virtual bool isValid () const = 0;
      };
      Iterator* iterator_;
  };

  /** \brief Iterator class for point clouds with or without given indices
    * \author Suat Gedikli
    */
  template <typename PointT>
  class ConstCloudIterator
  {
    public:
      ConstCloudIterator (const PointCloud<PointT>& cloud);

      ConstCloudIterator (const PointCloud<PointT>& cloud, const std::vector<int>& indices);

      ConstCloudIterator (const PointCloud<PointT>& cloud, const PointIndices& indices);

      ConstCloudIterator (const PointCloud<PointT>& cloud, const Correspondences& corrs, bool source);

      ~ConstCloudIterator ();

      void operator ++ ();

      void operator ++ (int);

      const PointT& operator* () const;

      const PointT* operator-> () const;

      unsigned getCurrentPointIndex () const;

      unsigned getCurrentIndex () const;

      /** \brief Size of the range the iterator is going through. Depending on how the ConstCloudIterator was constructed this is the size of the cloud or indices/correspondences. */
      size_t size () const;

      void reset ();

      bool isValid () const;

      operator bool () const
      {
        return isValid ();
      }
    private:

      class Iterator
      {
        public:
          virtual ~Iterator ()  {}

          virtual void operator ++ () = 0;

          virtual void operator ++ (int) = 0;

          virtual const PointT& operator* () const = 0;

          virtual const PointT* operator-> () const = 0;

          virtual unsigned getCurrentPointIndex () const = 0;

          virtual unsigned getCurrentIndex () const = 0;

          /** \brief Size of the range the iterator is going through. Depending on how the ConstCloudIterator was constructed this is the size of the cloud or indices/correspondences. */
          virtual size_t size () const = 0;

          virtual void reset () = 0;

          virtual bool isValid () const = 0;
      };

      class DefaultConstIterator;
      class ConstIteratorIdx;
      Iterator* iterator_;
  };

} // namespace pcl

#include <pcl/impl/cloud_iterator.hpp>

#endif    // PCL_POINT_CLOUD_ITERATOR_H_
