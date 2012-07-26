
template<class PointT>
class DefaultIterator : public pcl::CloudIterator<PointT>::Iterator
{
  public:
    DefaultIterator(pcl::PointCloud<PointT>& cloud)
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
      return *iterator_;
    }

    PointT* operator-> ()
    {
      return &(*iterator_);
    }

    unsigned getCurrentPointIndex () const
    {
      return iterator_ - cloud_.begin ();
    }

    unsigned getCurrentIndex () const
    {
      return iterator_ - cloud_.begin ();
    }
   
    void reset ()
    {
      iterator_ = cloud_.begin ();
    }

    bool isValid () const
    {
      return iterator_ != cloud_.end ();
    }
  private:
    pcl::PointCloud<PointT>& cloud_;
    typename pcl::PointCloud<PointT>::iterator iterator_;
};

template<class PointT>
class IteratorIdx : public pcl::CloudIterator<PointT>::Iterator
{
  public:
    IteratorIdx (pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices)
    : cloud_ (cloud)
    , indices_ (indices) 
    , iterator_ (indices_.begin ())
    {
    }

    IteratorIdx (pcl::PointCloud<PointT>& cloud, const pcl::PointIndices& indices)
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
      return cloud_.points [*iterator_];
    }

    PointT* operator-> ()
    {
      return &(cloud_.points [*iterator_]);
    }

    unsigned getCurrentPointIndex () const
    {
      return *iterator_;
    }

    unsigned getCurrentIndex () const
    {
      return iterator_ - indices_.begin ();
    }

    void reset ()
    {
      iterator_ = indices_.begin ();
    }

    bool isValid () const
    {
      return iterator_ != indices_.end ();
    }

    private:
      pcl::PointCloud<PointT>& cloud_;
      std::vector<int> indices_;
      std::vector<int>::iterator iterator_;
};

template<class PointT>
pcl::CloudIterator<PointT>::CloudIterator(PointCloud<PointT>& cloud)
: iterator_ (new DefaultIterator<PointT> (cloud))
{
}

template<class PointT>
pcl::CloudIterator<PointT>::CloudIterator (PointCloud<PointT>& cloud, const std::vector<int>& indices)
: iterator_ (new IteratorIdx<PointT> (cloud, indices))
{
}

template<class PointT>
pcl::CloudIterator<PointT>::CloudIterator (PointCloud<PointT>& cloud, const PointIndices& indices)
: iterator_ (new IteratorIdx<PointT> (cloud, indices))
{
}

template<class PointT>
pcl::CloudIterator<PointT>::CloudIterator (PointCloud<PointT>& cloud, const Correspondences& corrs, bool source)
{
  std::vector<int> indices;
  indices.reserve (corrs.size ());
  if (source)
  {
    for (typename Correspondences::const_iterator indexIt = corrs.begin (); indexIt != corrs.end (); ++indexIt)
      indices.push_back (indexIt->index_query);
  }
  else
  {
    for (typename Correspondences::const_iterator indexIt = corrs.begin (); indexIt != corrs.end (); ++indexIt)
      indices.push_back (indexIt->index_match);
  }
  iterator_ = new IteratorIdx<PointT> (cloud, indices);
}

template<class PointT>
pcl::CloudIterator<PointT>::~CloudIterator ()
{
  delete iterator_;
}

template<class PointT> void
pcl::CloudIterator<PointT>::operator ++ ()
{
  iterator_->operator++ ();
}

template<class PointT> void
pcl::CloudIterator<PointT>::operator ++ (int)
{
  iterator_->operator++ (0);
}

template<class PointT> PointT&
pcl::CloudIterator<PointT>::operator* () const
{
  return iterator_->operator * ();
}

template<class PointT> PointT*
pcl::CloudIterator<PointT>::operator-> () const
{
  return iterator_->operator-> ();
}

template<class PointT> unsigned
pcl::CloudIterator<PointT>::getCurrentPointIndex () const
{
  return iterator_->getCurrentPointIndex ();
}

template<class PointT> unsigned
pcl::CloudIterator<PointT>::getCurrentIndex () const
{
  return iterator_->getCurrentIndex ();
}

template<class PointT> void
pcl::CloudIterator<PointT>::reset ()
{
  iterator_->reset ();
}

template<class PointT> bool
pcl::CloudIterator<PointT>::isValid () const
{
  return iterator_->isValid ();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<class PointT>
class pcl::ConstCloudIterator<PointT>::DefaultConstIterator : public pcl::ConstCloudIterator<PointT>::Iterator
{
  public:
    DefaultConstIterator(const pcl::PointCloud<PointT>& cloud)
    : cloud_ (cloud)
    , iterator_ (cloud.begin ())
    {
    }
    
    ~DefaultConstIterator ()
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

    const PointT& operator* () const
    {
      return *iterator_;
    }

    const PointT* operator-> () const
    {
      return &(*iterator_);
    }

    unsigned getCurrentPointIndex () const
    {
      return iterator_ - cloud_.begin ();
    }

    unsigned getCurrentIndex () const
    {
      return iterator_ - cloud_.begin ();
    }
   
    void reset ()
    {
      iterator_ = cloud_.begin ();
    }

    bool isValid () const
    {
      return iterator_ != cloud_.end ();
    }
  private:
    const pcl::PointCloud<PointT>& cloud_;
    typename pcl::PointCloud<PointT>::const_iterator iterator_;
};

template<class PointT>
class pcl::ConstCloudIterator<PointT>::ConstIteratorIdx : public pcl::ConstCloudIterator<PointT>::Iterator
{
  public:
    ConstIteratorIdx (const pcl::PointCloud<PointT>& cloud, const std::vector<int>& indices)
    : cloud_ (cloud)
    , indices_ (indices) 
    , iterator_ (indices_.begin ())
    {
    }

    ConstIteratorIdx (const pcl::PointCloud<PointT>& cloud, const pcl::PointIndices& indices)
    : cloud_ (cloud)
    , indices_ (indices.indices) 
    , iterator_ (indices_.begin ())
    {
    }

    virtual ~ConstIteratorIdx () {}
    void operator ++ ()
    {
      ++iterator_;
    }

    void operator ++ (int)
    {
      iterator_++;
    }

    const PointT& operator* () const
    {
      return cloud_.points [*iterator_];
    }

    const PointT* operator-> () const
    {
      return &(cloud_.points [*iterator_]);
    }

    unsigned getCurrentPointIndex () const
    {
      return *iterator_;
    }

    unsigned getCurrentIndex () const
    {
      return iterator_ - indices_.begin ();
    }

    void reset ()
    {
      iterator_ = indices_.begin ();
    }

    bool isValid () const
    {
      return iterator_ != indices_.end ();
    }

    private:
      const pcl::PointCloud<PointT>& cloud_;
      std::vector<int> indices_;
      std::vector<int>::iterator iterator_;
};

template<class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator(const PointCloud<PointT>& cloud)
: iterator_ (new pcl::ConstCloudIterator<PointT>::DefaultConstIterator (cloud))
{
}

template<class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (const PointCloud<PointT>& cloud, const std::vector<int>& indices)
: iterator_ (new pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices))
{
}

template<class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (const PointCloud<PointT>& cloud, const PointIndices& indices)
: iterator_ (new pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices))
{
}

template<class PointT>
pcl::ConstCloudIterator<PointT>::ConstCloudIterator (const PointCloud<PointT>& cloud, const Correspondences& corrs, bool source)
{
  std::vector<int> indices;
  indices.reserve (corrs.size ());
  if (source)
  {
    for (typename Correspondences::const_iterator indexIt = corrs.begin (); indexIt != corrs.end (); ++indexIt)
      indices.push_back (indexIt->index_query);
  }
  else
  {
    for (typename Correspondences::const_iterator indexIt = corrs.begin (); indexIt != corrs.end (); ++indexIt)
      indices.push_back (indexIt->index_match);
  }
  iterator_ = new pcl::ConstCloudIterator<PointT>::ConstIteratorIdx (cloud, indices);
}

template<class PointT>
pcl::ConstCloudIterator<PointT>::~ConstCloudIterator ()
{
  delete iterator_;
}

template<class PointT> void
pcl::ConstCloudIterator<PointT>::operator ++ ()
{
  iterator_->operator++ ();
}

template<class PointT> void
pcl::ConstCloudIterator<PointT>::operator ++ (int)
{
  iterator_->operator++ (0);
}

template<class PointT> const PointT&
pcl::ConstCloudIterator<PointT>::operator* () const
{
  return iterator_->operator * ();
}

template<class PointT> const PointT*
pcl::ConstCloudIterator<PointT>::operator-> () const
{
  return iterator_->operator-> ();
}

template<class PointT> unsigned
pcl::ConstCloudIterator<PointT>::getCurrentPointIndex () const
{
  return iterator_->getCurrentPointIndex ();
}

template<class PointT> unsigned
pcl::ConstCloudIterator<PointT>::getCurrentIndex () const
{
  return iterator_->getCurrentIndex ();
}

template<class PointT> void
pcl::ConstCloudIterator<PointT>::reset ()
{
  iterator_->reset ();
}

template<class PointT> bool
pcl::ConstCloudIterator<PointT>::isValid () const
{
  return iterator_->isValid ();
}


