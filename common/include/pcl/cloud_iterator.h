#pragma once
#include "point_cloud.h"
#include "PointIndices.h"
#include "correspondence.h"

namespace pcl
{
  /** \brief Iterator class for point clouds with or without given indices
    * \author Suat Gedikli
    */
  template<class PointT>
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

      void reset ();
      
      bool isValid () const;
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

          virtual void reset () = 0;
          
          virtual bool isValid () const = 0;
      };
      Iterator* iterator_;      
  };
  
  /** \brief Iterator class for point clouds with or without given indices
    * \author Suat Gedikli
    */
  template<class PointT>
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

      void reset ();
      
      bool isValid () const;
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

          virtual void reset () = 0;
          
          virtual bool isValid () const = 0;
      };
      
      class DefaultConstIterator;
      class ConstIteratorIdx;
      Iterator* iterator_;
  };
  
} // namespace pcl

#include <pcl/impl/cloud_iterator.hpp>