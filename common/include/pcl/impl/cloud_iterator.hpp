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

#ifndef PCL_POINT_CLOUD_ITERATOR_HPP_
#define PCL_POINT_CLOUD_ITERATOR_HPP_

#include <pcl/cloud_iterator.h>

namespace pcl
{
  /** \brief
    * \author Suat Gedikli
    */
  template <class PointT>
  class DefaultIterator : public CloudIterator<PointT>::Iterator
  {
    public:
      DefaultIterator (PointCloud<PointT>& cloud)
      : cloud_ (cloud)
      , iterator_ (cloud.begin ())
      {
      }

      ~DefaultIterator ()
      {
      }

      void operator ++ ()
      {
        ++iterator_;
      }

      void operator ++ (int)
      {
        iterator_++;
      }

      PointT& operator* () const
      {
        return (*iterator_);
      }

      PointT* operator-> ()
      {
        return (&(*iterator_));
      }

      unsigned getCurrentPointIndex () const
      {
        return (iterator_ - cloud_.begin ());
      }

      unsigned getCurrentIndex () const
      {
        return (iterator_ - cloud_.begin ());
      }

      std::size_t size () const
      {
        return cloud_.size ();
      }

      void reset ()
      {
        iterator_ = cloud_.begin ();
      }

      bool isValid () const
      {
        return (iterator_ != cloud_.end ());
      }
    private:
      PointCloud<PointT>& cloud_;
      typename PointCloud<PointT>::iterator iterator_;
  };

  /** \brief
    * \author Suat Gedikli
    */
  template <class PointT>
  class IteratorIdx : public CloudIterator<PointT>::Iterator
  {
    public:
      IteratorIdx (PointCloud<PointT>& cloud, const Indices& indices)
        : cloud_ (cloud)
        , indices_ (indices)
        , iterator_ (indices_.begin ())
      {
      }

      IteratorIdx (PointCloud<PointT>& cloud, const PointIndices& indices)
        : cloud_ (cloud)
        , indices_ (indices.indices)
        , iterator_ (indices_.begin ())
      {
      }

      virtual ~IteratorIdx () {}

      void operator ++ ()
      {
        ++iterator_;
      }

      void operator ++ (int)
      {
        iterator_++;
      }

      PointT& operator* () const
      {
        return (cloud_.points [*iterator_]);
      }

      PointT* operator-> ()
      {
        return (&(cloud_.points [*iterator_]));
      }

      unsigned getCurrentPointIndex () const
      {
        return (*iterator_);
      }

      unsigned getCurrentIndex () const
      {
        return (iterator_ - indices_.begin ());
      }

      std::size_t size () const
      {
        return indices_.size ();
      }

      void reset ()
      {
        iterator_ = indices_.begin ();
      }

      bool isValid () const
      {
        return (iterator_ != indices_.end ());
      }

      private:
        PointCloud<PointT>& cloud_;
        Indices indices_;
        Indices::iterator iterator_;
  };

  /** \brief
    * \author Suat Gedikli
    */
  template <class PointT>
  class ConstCloudIterator<PointT>::DefaultConstIterator : public ConstCloudIterator<PointT>::Iterator
  {
    public:
      DefaultConstIterator (const PointCloud<PointT>& cloud)
        : cloud_ (cloud)
        , iterator_ (cloud.begin ())
      {
      }

      ~DefaultConstIterator ()
      {
      }

      void operator ++ () override
      {
        ++iterator_;
      }

      void operator ++ (int) override
      {
        iterator_++;
      }

      const PointT& operator* () const override
      {
        return (*iterator_);
      }

      const PointT* operator-> () const override
      {
        return (&(*iterator_));
      }

      unsigned getCurrentPointIndex () const override
      {
        return (unsigned (iterator_ - cloud_.begin ()));
      }

      unsigned getCurrentIndex () const override
      {
        return (unsigned (iterator_ - cloud_.begin ()));
      }

      std::size_t size () const override
      {
        return cloud_.size ();
      }

      void reset () override
      {
        iterator_ = cloud_.begin ();
      }

      bool isValid () const override
      {
        return (iterator_ != cloud_.end ());
      }
    private:
      const PointCloud<PointT>& cloud_;
      typename PointCloud<PointT>::const_iterator iterator_;
  };

  /** \brief
    * \author Suat Gedikli
    */
  template <class PointT>
  class ConstCloudIterator<PointT>::ConstIteratorIdx : public ConstCloudIterator<PointT>::Iterator
  {
    public:
      ConstIteratorIdx (const PointCloud<PointT>& cloud,
                        const Indices& indices)
        : cloud_ (cloud)
        , indices_ (indices)
        , iterator_ (indices_.begin ())
      {
      }

      ConstIteratorIdx (const PointCloud<PointT>& cloud,
                        const PointIndices& indices)
        : cloud_ (cloud)
        , indices_ (indices.indices)
        , iterator_ (indices_.begin ())
      {
      }

      ~ConstIteratorIdx () {}

      void operator ++ () override
      {
        ++iterator_;
      }

      void operator ++ (int) override
      {
        iterator_++;
      }

      const PointT& operator* () const override
      {
        return (cloud_[*iterator_]);
      }

      const PointT* operator-> () const override
      {
        return (&(cloud_.points [*iterator_]));
      }

      unsigned getCurrentPointIndex () const override
      {
        return (unsigned (*iterator_));
      }

      unsigned getCurrentIndex () const override
      {
        return (unsigned (iterator_ - indices_.begin ()));
      }

      std::size_t size () const override
      {
        return indices_.size ();
      }

      void reset () override
      {
        iterator_ = indices_.begin ();
      }

      bool isValid () const override
      {
        return (iterator_ != indices_.end ());
      }

      private:
        const PointCloud<PointT>& cloud_;
        Indices indices_;
        Indices::iterator iterator_;
  };
} // namespace pcl

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::CloudIterator<PointT>::CloudIterator (PointCloud<PointT>& cloud)
  : iterator_ (new DefaultIterator<PointT> (cloud))
{
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::CloudIterator<PointT>::CloudIterator (
    PointCloud<PointT>& cloud, const Indices& indices)
  : iterator_ (new IteratorIdx<PointT> (cloud, indices))
{
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::CloudIterator<PointT>::CloudIterator (
    PointCloud<PointT>& cloud, const PointIndices& indices)
  : iterator_ (new IteratorIdx<PointT> (cloud, indices))
{
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::CloudIterator<PointT>::CloudIterator (
    PointCloud<PointT>& cloud, const Correspondences& corrs, bool source)
{
  Indices indices;
  indices.reserve (corrs.size ());
  if (source)
  {
    for (const auto &corr : corrs)
      indices.push_back (corr.index_query);
  }
  else
  {
    for (const auto &corr : corrs)
      indices.push_back (corr.index_match);
  }
  iterator_ = new IteratorIdx<PointT> (cloud, indices);
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::CloudIterator<PointT>::~CloudIterator ()
{
  delete iterator_;
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> void
pcl::CloudIterator<PointT>::operator ++ ()
{
  iterator_->operator++ ();
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> void
pcl::CloudIterator<PointT>::operator ++ (int)
{
  iterator_->operator++ (0);
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> PointT&
pcl::CloudIterator<PointT>::operator* () const
{
  return (iterator_->operator * ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> PointT*
pcl::CloudIterator<PointT>::operator-> () const
{
  return (iterator_->operator-> ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> unsigned
pcl::CloudIterator<PointT>::getCurrentPointIndex () const
{
  return (iterator_->getCurrentPointIndex ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> unsigned
pcl::CloudIterator<PointT>::getCurrentIndex () const
{
  return (iterator_->getCurrentIndex ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> std::size_t
pcl::CloudIterator<PointT>::size () const
{
  return (iterator_->size ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> void
pcl::CloudIterator<PointT>::reset ()
{
  iterator_->reset ();
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> bool
pcl::CloudIterator<PointT>::isValid () const
{
  return (iterator_->isValid ());
}


//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (const PointCloud<PointT>& cloud)
  : iterator_ (new typename pcl::ConstCloudIterator<PointT>::DefaultConstIterator (cloud))
{
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (
    const PointCloud<PointT>& cloud, const Indices& indices)
  : iterator_ (new typename pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices))
{
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (
    const PointCloud<PointT>& cloud, const PointIndices& indices)
  : iterator_ (new typename pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices))
{
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (
    const PointCloud<PointT>& cloud, const Correspondences& corrs, bool source)
{
  Indices indices;
  indices.reserve (corrs.size ());
  if (source)
  {
    for (const auto &corr : corrs)
      indices.push_back (corr.index_query);
  }
  else
  {
    for (const auto &corr : corrs)
      indices.push_back (corr.index_match);
  }
  iterator_ = new typename pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices);
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT>
pcl::ConstCloudIterator<PointT>::~ConstCloudIterator ()
{
  delete iterator_;
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> void
pcl::ConstCloudIterator<PointT>::operator ++ ()
{
  iterator_->operator++ ();
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> void
pcl::ConstCloudIterator<PointT>::operator ++ (int)
{
  iterator_->operator++ (0);
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> const PointT&
pcl::ConstCloudIterator<PointT>::operator* () const
{
  return (iterator_->operator * ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> const PointT*
pcl::ConstCloudIterator<PointT>::operator-> () const
{
  return (iterator_->operator-> ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> unsigned
pcl::ConstCloudIterator<PointT>::getCurrentPointIndex () const
{
  return (iterator_->getCurrentPointIndex ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> unsigned
pcl::ConstCloudIterator<PointT>::getCurrentIndex () const
{
  return (iterator_->getCurrentIndex ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> std::size_t
pcl::ConstCloudIterator<PointT>::size () const
{
  return (iterator_->size ());
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> void
pcl::ConstCloudIterator<PointT>::reset ()
{
  iterator_->reset ();
}

//////////////////////////////////////////////////////////////////////////////
template <class PointT> bool
pcl::ConstCloudIterator<PointT>::isValid () const
{
  return (iterator_->isValid ());
}

#endif    // PCL_POINT_CLOUD_ITERATOR_HPP_

