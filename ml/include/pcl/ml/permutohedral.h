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
 */

#pragma once

#ifdef __GNUC__
#pragma GCC system_header 
#endif

#include <vector>
#include <map>
#include <pcl/common/eigen.h>
#include <boost/intrusive/hashtable.hpp>

// TODO: SWAP with Boost intrusive hash table
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cstdio>
#include <cmath>

namespace pcl
{
  /** \brief Implementation of a high-dimensional gaussian filtering using the permutohedral lattice
    * \author Christian Potthast (potthast@usc.edu)
    *
    * Adams_fasthigh-dimensional
    *   author = {Andrew Adams and Jongmin Baek and Myers Abraham Davis},
    *   title = {Fast high-dimensional filtering using the permutohedral lattice},
    *   booktitle = {Computer Graphics Forum (EG 2010 Proceedings},
    *   year = {},
    *   pages = {2010}
    * }
    */
  class Permutohedral
  {
    protected:
      struct Neighbors
      {
        int n1, n2;
        Neighbors (int n1 = 0, int n2 = 0) : n1 (n1), n2 (n2) {}
      };

    public:

      /** \brief Constructor for Permutohedral class */
      Permutohedral ();

      /** \brief Deconstructor for Permutohedral class */
      ~Permutohedral () {};

      /** \brief initialization */
      void
      init (const std::vector<float> &feature, const int feature_dimension, const int N);

      void 
      compute (std::vector<float> &out, const std::vector<float> &in, 
               int value_size, 
               int in_offset=0, int out_offset=0, 
               int in_size = -1, int out_size = -1) const;
      void
      initOLD (const std::vector<float> &feature, const int feature_dimension, const int N);

      void 
      computeOLD (std::vector<float> &out, const std::vector<float> &in, 
                  int value_size, 
                  int in_offset=0, int out_offset=0, 
                  int in_size = -1, int out_size = -1) const;

      void
      debug ();

      // Pseudo radnom generator
      inline
      size_t generateHashKey (const std::vector<short> &k) 
      {
        size_t r = 0;
        for (int i = 0; i < d_; i++)
        {
          r += k[i];
          r *= 1664525;
          //r *= 5;
        }
        return r;// % (2* N_ * (d_+1));
      }

    public:

      /** \brief Number of variables */
      int N_;

      std::vector<Neighbors> blur_neighbors_;

      /** \brief size of sparse discretized space */
      int M_;

      /** \brief dimension of feature */
      int d_;

      std::vector<float> offset_;
      std::vector<float> offsetTMP_;
      std::vector<float> barycentric_;

      Neighbors * blur_neighborsOLD_;
      int * offsetOLD_;
      float * barycentricOLD_;
      std::vector<float> baryOLD_;
      
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW      

  };

  class HashTableOLD
  {
    // Don't copy!
    HashTableOLD( const HashTableOLD & o ): key_size_ ( o.key_size_ ), filled_(0), capacity_(o.capacity_) {
      table_ = new int[ capacity_ ];
      keys_ = new short[ (capacity_/2+10)*key_size_ ];
      memset( table_, -1, capacity_*sizeof(int) );
    }
  protected:
    size_t key_size_, filled_, capacity_;
    short * keys_;
    int * table_;
    void grow(){
      std::cout << "GROW" << std::endl;
      
      // Swap out the old memory
      short * old_keys = keys_;
      int * old_table = table_;
      int old_capacity = static_cast<int> (capacity_);
      capacity_ *= 2;
      // Allocate the new memory
      keys_ = new short[ (old_capacity+10)*key_size_ ];
      table_ = new int[ capacity_ ];
      memset( table_, -1, capacity_*sizeof(int) );
      memcpy( keys_, old_keys, filled_*key_size_*sizeof(short) );
      
      // Reinsert each element
      for( int i=0; i<old_capacity; i++ )
        if (old_table[i] >= 0){
          int e = old_table[i];
          size_t h = hash( old_keys+(getKey(e)-keys_) ) % capacity_;
          for (; table_[h] >= 0; h = h<capacity_-1 ? h+1 : 0) { };
          table_[h] = e;
        }
      
      delete [] old_keys;
      delete [] old_table;
    }
    size_t hash( const short * k ) {
      size_t r = 0;
      for( size_t i=0; i<key_size_; i++ ){
        r += k[i];
        r *= 1664525;
      }
      return r;
    }
  public:
    explicit HashTableOLD( int key_size, int n_elements ) : key_size_ ( key_size ), filled_(0), capacity_(2*n_elements) {
      table_ = new int[ capacity_ ];
      keys_ = new short[ (capacity_/2+10)*key_size_ ];
      memset( table_, -1, capacity_*sizeof(int) );
    }
    ~HashTableOLD() {
      delete [] keys_;
      delete [] table_;
    }
    int size() const {
      return static_cast<int> (filled_);
    }
    void reset() {
      filled_ = 0;
      memset( table_, -1, capacity_*sizeof(int) );
    }
    int find( const short * k, bool create = false ){
      if (2*filled_ >= capacity_) grow();
      // Get the hash value
      size_t h = hash( k ) % capacity_;
      // Find the element with he right key, using linear probing
      while(1){
        int e = table_[h];
        if (e==-1){
          if (create){
            // Insert a new key and return the new id
            for( size_t i=0; i<key_size_; i++ )
              keys_[ filled_*key_size_+i ] = k[i];
            return table_[h] = static_cast<int> (filled_++);
          }
          else
            return -1;
        }
        // Check if the current key is The One
        bool good = true;
        for( size_t i=0; i<key_size_ && good; i++ )
          if (keys_[ e*key_size_+i ] != k[i])
            good = false;
        if (good)
          return e;
        // Continue searching
        h++;
        if (h==capacity_) h = 0;
      }
    }
    const short * getKey( int i ) const{
      return keys_+i*key_size_;
    }
  };
  /*
  class HashTable
  {
    public:
      HashTable ( int N ) : N_ (2 * N) {};

      find (const std::vector<short> &k, bool create = false;)
      {
        
        



      }
      

      
    protected:
      std::multimap<size_t, int> table_;
      
      std::vector<std::vector<short> > keys;
      //keys.reserve ( (d_+1) * N_ );
      // number of elements
      int N_;
  };*/

}
