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



#ifndef __PCL_OUTOFCORE_LRU_CACHE__
#define __PCL_OUTOFCORE_LRU_CACHE__

#include <cassert>
#include <list>

template<typename T>
class LRUCacheItem
{
public:

  virtual size_t
  sizeOf () const
  {
    return sizeof (item);
  }

  virtual
  ~LRUCacheItem () { }

  T item;
  size_t timestamp;
};

template<typename KeyT, typename CacheItemT>
class LRUCache
{
public:

  typedef std::list<KeyT> KeyIndex;
  typedef typename KeyIndex::iterator KeyIndexIterator;

  typedef std::map<KeyT, std::pair<CacheItemT, typename KeyIndex::iterator> > Cache;
  typedef typename Cache::iterator CacheIterator;

  LRUCache (size_t c) :
      capacity_ (c), size_ (0)
  {
    assert(capacity_ != 0);
  }

  bool
  hasKey (const KeyT& k)
  {
    return (cache_.find (k) != cache_.end ());
  }

  CacheItemT&
  get (const KeyT& k)
  {
    // Get existing key
    const CacheIterator it = cache_.find (k);
    assert(it != cache_.end ());

    // Move key to MRU key index
    key_index_.splice (key_index_.end (), key_index_, (*it).second.second);

    // Return the retrieved item
    return it->second.first;
  }

  void
  touch (const KeyT& key)
  {
    // Get existing key
    const CacheIterator it = cache_.find (key);
    assert(it != cache_.end ());

    // Move key to MRU key index
    key_index_.splice (key_index_.end (), key_index_, it->second.second);
  }

  // Record a fresh key-value pair in the cache
  bool
  insert (const KeyT& key, const CacheItemT& value)
  {
    if (cache_.find (key) != cache_.end ())
    {
      touch (key);
      return true;
    }

    size_t size = size_;
    size_t item_size = value.sizeOf ();
    int evict_count = 0;

    // Get LRU key iterator
    KeyIndexIterator key_it = key_index_.begin ();

    while (size + item_size >= capacity_)
    {
      const CacheIterator cache_it = cache_.find (*key_it);

      // Get tail item (Least Recently Used)
      size_t tail_timestamp = cache_it->second.first.timestamp;
      size_t tail_size = cache_it->second.first.sizeOf ();

      // Check timestamp to see if we've completely filled the cache in one go
      if (value.timestamp == tail_timestamp)
      {
        return false;
      }

      size -= tail_size;
      key_it++;
      evict_count++;
    }

    // Evict enough items to make room for the new item
    evict (evict_count);

    size_ += item_size;

    // Insert most-recently-used key at the end of our key index
    KeyIndexIterator it = key_index_.insert (key_index_.end (), key);

    // Add to cache
    cache_.insert (std::make_pair (key, std::make_pair (value, it)));

    return true;
  }

  void
  setCapacity (size_t capacity)
  {
    capacity_ = capacity;
  }

  CacheItemT&
  tailItem ()
  {
    const CacheIterator it = cache_.find (key_index_.front ());
    return it->second.first;
  }

  size_t
  sizeOf (const CacheItemT& value)
  {
    return value.sizeOf ();
  }

  // Evict the least-recently-used item from the cache
  bool
  evict (int item_count=1)
  {
    for (int i=0; i < item_count; i++)
    {
      if (key_index_.empty ())
        return false;

      // Get LRU item
      const CacheIterator it = cache_.find (key_index_.front ());
      assert(it != cache_.end());

      // Remove LRU item from cache and key index
      size_ -= it->second.first.sizeOf ();
      cache_.erase (it);
      key_index_.pop_front ();
    }

    return true;
  }

  // Cache capacity in kilobytes
  size_t capacity_;

  // Current cache size in kilobytes
  size_t size_;

  // LRU key index LRU[0] ... MRU[N]
  KeyIndex key_index_;

  // LRU cache
  Cache cache_;
};

#endif //__PCL_OUTOFCORE_LRU_CACHE__
