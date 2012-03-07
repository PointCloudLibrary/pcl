/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012, Urban Robotics, Inc.
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
 */

/*
  This code defines the octree used for point storage at Urban Robotics. Please
  contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions.
  http://www.urbanrobotics.net/
*/
#ifndef PCL_OCTREE_DISK_CONTAINER_IMPL_H_
#define PCL_OCTREE_DISK_CONTAINER_IMPL_H_

// C++
#include <sstream>
#include <cassert>
#include <ctime>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>

// PCL (Urban Robotics)
#include "pcl/outofcore/octree_disk_container.h"

//allows operation on POSIX
#ifndef WIN32
#define _fseeki64 fseeko
#endif

namespace pcl
{
  namespace outofcore
  {

    template<typename PointT>
    boost::mutex octree_disk_container<PointT>::rng_mutex_;

    template<typename PointT> boost::mt19937
    octree_disk_container<PointT>::rand_gen_ ( static_cast<unsigned int> ( std::time( NULL ) ) );

    template<typename PointT>
    boost::uuids::random_generator octree_disk_container<PointT>::uuid_gen_ (&rand_gen_);

    template<typename PointT> void
    octree_disk_container<PointT>::getRandomUUIDString (std::string& s)
    {
      boost::uuids::uuid u;
      {
        boost::mutex::scoped_lock lock (rng_mutex_);
        u = uuid_gen_ ();
      }

      std::stringstream ss;
      ss << u;
      s = ss.str ();
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT>
    octree_disk_container<PointT>::octree_disk_container ()
    {
      std::string temp = getRandomUUIDString ();
      fileback_name_ = new std::string ();
      *fileback_name_ = temp;
      filelen_ = 0;
      //writebuff.reserve(writebuffmax);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT>
    octree_disk_container<PointT>::octree_disk_container (const boost::filesystem::path& path)
    {
      if (boost::filesystem::exists (path))
      {
        if (boost::filesystem::is_directory (path))
        {
          std::string uuid;
          getRandomUUIDString (uuid);
          boost::filesystem::path filename (uuid);
          boost::filesystem::path file = path / filename;

          fileback_name_ = new std::string (file.string ());

          filelen_ = 0;
        }
        else
        {
          boost::uint64_t len = boost::filesystem::file_size (path);

          fileback_name_ = new std::string (path.string ());

          filelen_ = len / sizeof(PointT);
        }
      }
      else
      {
        fileback_name_ = new std::string (path.string ());
        filelen_ = 0;
      }

      //writebuff.reserve(writebuffmax);
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT>
    octree_disk_container<PointT>::~octree_disk_container ()
    {
      flushWritebuff (true);
      //fileback.flush();
      //fileback.close();
      //std::remove(persistant->c_str());
      //boost::filesystem::remove(fileback_name_);//for testing!
      //std::cerr << "deleted file " << *persistant << std::endl;
      //std::cerr << "destruct container" << std::endl;
      delete fileback_name_;
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    octree_disk_container<PointT>::flushWritebuff (const bool force_cache_dealloc)
    {
      if (writebuff_.size () > 0)
      {
        FILE* f = fopen (fileback_name_->c_str (), "a+b");

        size_t len = writebuff_.size () * sizeof(PointT);
        /** \todo study and optimize the serialization to disk */
        char* loc = reinterpret_cast<char*> ( & (writebuff_.front ()) );
        size_t w = fwrite (loc, 1, len, f);
        (void)w;
        assert (w == len);

        //		int closeret = fclose(f);
        fclose (f);

        filelen_ += writebuff_.size ();
        writebuff_.clear ();
      }

      //if(forceCacheDeAlloc || (size() >= node<octree_disk_container<PointT>, PointT>::split_thresh))  //if told to dump cache, or if we have more nodes than the split threshold
      if (force_cache_dealloc)//if told to dump cache, or if we have more nodes than the split threshold
      {
        //don't reserve anymore -- lets us have more nodes and be fairly
        //lazy about dumping the cache. but once it is dumped for the last
        //time, this needs to be empty.
        writebuff_.resize (0);
      }
      else
      {
        //we are still accepting data, preallocate the storage
        //writebuff.reserve(writebuffmax);
      }

    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> PointT
    octree_disk_container<PointT>::operator[] (boost::uint64_t idx)
    {
      //if the index is on disk
      if (idx < filelen_)
      {

        PointT temp;
        //open our file
        FILE* f = fopen (fileback_name_->c_str (), "rb");
        assert (f != NULL);
        //seek the right length; 

        int seekret = _fseeki64 (f, idx * sizeof(PointT), SEEK_SET);
        assert (seekret == 0);
        size_t readlen = fread (&temp, 1, sizeof(PointT), f);
        assert (readlen == sizeof(PointT));
        int closeret = fclose (f);

        //fileback.open(fileback_name_->c_str(), std::fstream::in|std::fstream::out|std::fstream::binary);
        //fileback.seekp(idx*sizeof(PointT), std::ios_base::beg);
        //fileback.read((char*)&temp, sizeof(PointT));
        //fileback.close();
        return temp;
      }
      //otherwise if the index is still in the write buffer
      if (idx < (filelen_ + writebuff_.size ()))
      {
        idx -= filelen_;
        return writebuff_[idx];
      }

      //else, throw out of range exception
      /** \todo standardize the exceptions to PCL's */
      throw("out of range");
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    octree_disk_container<PointT>::readRange (const boost::uint64_t start, const boost::uint64_t count, std::vector<PointT, Eigen::aligned_allocator<PointT> >& v)
    {
      if ((start + count) > size ())
      {
        throw("out of range");
      }

      if (count == 0)
      {
        return;
      }

      boost::uint64_t filestart = 0;
      boost::uint64_t filecount = 0;

      boost::int64_t buffstart = -1;
      boost::int64_t buffcount = -1;

      if (start < filelen_)
      {
        filestart = start;
      }

      if ((start + count) <= filelen_)
      {
        filecount = count;
      }
      else
      {
        filecount = filelen_ - start;

        buffstart = 0;
        buffcount = count - filecount;
      }

      //resize
      PointT* loc = NULL;
      v.resize (static_cast<unsigned int>(count));
      loc = &(v.front ());

      //do the read
      FILE* f = fopen (fileback_name_->c_str (), "rb");
      assert (f != NULL);
      int seekret = _fseeki64 (f, filestart * static_cast<boost::uint64_t>(sizeof(PointT)), SEEK_SET);
      if (seekret != 0)
      {
        //suppressed warning. empty if statement?
      }
      // error out?
      assert (seekret == 0);

      //read at most 2 million elements at a time
      const static boost::uint32_t blocksize = boost::uint32_t (2e6);
      for (boost::uint64_t pos = 0; pos < filecount; pos += blocksize)
      {
        if ((pos + blocksize) < filecount)
        {
          size_t readlen = fread (loc, sizeof(PointT), blocksize, f);
          if (readlen != blocksize) 
          {
            //suppressed warning. empty if statement; error?
          }
          // error out?
          assert (readlen == blocksize);
          loc += blocksize;
        }
        else
        {
          size_t readlen = fread (loc, sizeof(PointT), static_cast<size_t>(filecount - pos), f);
          if (readlen != filecount - pos)
          {
            //suppressed warning. empty if statement; error?
          }
          // error out?
          assert (readlen == filecount - pos);
          loc += filecount - pos;
        }
      }

      //	int closeret = fclose(f);
      fclose (f);

      //copy the extra
      if (buffstart != -1)
      {
        typename std::vector<PointT, Eigen::aligned_allocator<PointT> >::const_iterator start = writebuff_.begin ();
        typename std::vector<PointT, Eigen::aligned_allocator<PointT> >::const_iterator end = writebuff_.begin ();

        std::advance (start, buffstart);
        std::advance (end, buffstart + buffcount);

        v.insert (v.end (), start, end);
      }

    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    octree_disk_container<PointT>::readRangeSubSample_bernoulli (const boost::uint64_t start, const boost::uint64_t count, const double percent, std::vector<PointT, Eigen::aligned_allocator<PointT> >& v)
    {
      if (count == 0)
      {
        return;
      }

      v.clear ();

      boost::uint64_t filestart = 0;
      boost::uint64_t filecount = 0;

      boost::int64_t buffstart = -1;
      boost::int64_t buffcount = -1;

      if (start < filelen_)
      {
        filestart = start;
      }

      if ((start + count) <= filelen_)
      {
        filecount = count;
      }
      else
      {
        filecount = filelen_ - start;

        buffstart = 0;
        buffcount = count - filecount;
      }

      if (buffcount > 0)
      {
        {
          boost::mutex::scoped_lock lock (rng_mutex_);
          boost::bernoulli_distribution<double> buffdist (percent);
          boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > buffcoin (rand_gen_, buffdist);

          for (boost::uint64_t i = buffstart; i < buffcount; i++)
          {
            if (buffcoin ())
            {
              v.push_back (writebuff_[i]);
            }
          }
        }
      }

      if (filecount > 0)
      {
        //pregen and then sort the offsets to reduce the amount of seek
        std::vector < boost::uint64_t > offsets;
        {
          boost::mutex::scoped_lock lock (rng_mutex_);

          boost::bernoulli_distribution<double> filedist (percent);
          boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > filecoin (rand_gen_, filedist);
          for (boost::uint64_t i = filestart; i < (filestart + filecount); i++)
          {
            if (filecoin ())
            {
              offsets.push_back (i);
            }
          }
        }
        std::sort (offsets.begin (), offsets.end ());

        FILE* f = fopen (fileback_name_->c_str (), "rb");
        assert (f != NULL);
        PointT p;
        char* loc = static_cast<char*> ( &p );
        
        boost::uint64_t filesamp = offsets.size ();
        for (boost::uint64_t i = 0; i < filesamp; i++)
        {
          int seekret = _fseeki64 (f, offsets[i] * static_cast<boost::uint64_t>(sizeof(PointT)), SEEK_SET);
          assert (seekret == 0);
          size_t readlen = fread (loc, sizeof(PointT), 1, f);
          assert (readlen == 1);

          v.push_back (p);
        }
        int closeret = fclose (f);
      }
    }
////////////////////////////////////////////////////////////////////////////////

//change this to use a weighted coin flip, to allow sparse sampling of small clouds (eg the bernoulli above)
    template<typename PointT> void
    octree_disk_container<PointT>::readRangeSubSample (const boost::uint64_t start, const boost::uint64_t count, const double percent, std::vector<PointT, Eigen::aligned_allocator<PointT> >& v)
    {
      if (count == 0)
      {
        return;
      }

      v.clear ();

      boost::uint64_t filestart = 0;
      boost::uint64_t filecount = 0;

      boost::int64_t buffstart = -1;
      boost::int64_t buffcount = -1;

      if (start < filelen_)
      {
        filestart = start;
      }

      if ((start + count) <= filelen_)
      {
        filecount = count;
      }
      else
      {
        filecount = filelen_ - start;

        buffstart = 0;
        buffcount = count - filecount;
      }

      boost::uint64_t filesamp = boost::uint64_t (percent * filecount);
      boost::uint64_t buffsamp = (buffcount > 0) ? (static_cast<boost::uint64_t > (percent * buffcount) ) : 0;

      if ((filesamp == 0) && (buffsamp == 0) && (size () > 0))
      {
        //std::cerr << "would not add points to LOD, falling back to bernoulli";
        readRangeSubSample_bernoulli (start, count, percent, v);
        return;
      }

      if (buffcount > 0)
      {
        {
          boost::mutex::scoped_lock lock (rng_mutex_);

          boost::uniform_int < boost::uint64_t > buffdist (0, buffcount - 1);
          boost::variate_generator<boost::mt19937&, boost::uniform_int<boost::uint64_t> > buffdie (rand_gen_, buffdist);

          for (boost::uint64_t i = 0; i < buffsamp; i++)
          {
            boost::uint64_t buffstart = buffdie ();
            v.push_back (writebuff_[buffstart]);
          }
        }
      }

      if (filesamp > 0)
      {
        //pregen and then sort the offsets to reduce the amount of seek
        std::vector < boost::uint64_t > offsets;
        {
          boost::mutex::scoped_lock lock (rng_mutex_);

          offsets.resize (filesamp);
          boost::uniform_int < boost::uint64_t > filedist (filestart, filestart + filecount - 1);
          boost::variate_generator<boost::mt19937&, boost::uniform_int<boost::uint64_t> > filedie (rand_gen_, filedist);
          for (boost::uint64_t i = 0; i < filesamp; i++)
          {
            boost::uint64_t filestart = filedie ();
            offsets[i] = filestart;
          }
        }
        std::sort (offsets.begin (), offsets.end ());

        FILE* f = fopen (fileback_name_->c_str (), "rb");
        assert (f != NULL);
        PointT p;
        char* loc = static_cast<char*> ( &p );
        for (boost::uint64_t i = 0; i < filesamp; i++)
        {
          int seekret = _fseeki64 (f, offsets[i] * static_cast<boost::uint64_t> (sizeof(PointT)), SEEK_SET);
          assert (seekret == 0);
          size_t readlen = fread (loc, sizeof(PointT), 1, f);
          assert (readlen == 1);

          v.push_back (p);
        }
        int closeret = fclose (f);
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> inline void
    octree_disk_container<PointT>::push_back (const PointT& p)
    {
      writebuff_.push_back (p);
      if (writebuff_.size () > writebuffmax)
      {
        flushWritebuff (false);
      }
    }
////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> inline void
    octree_disk_container<PointT>::insertRange (const PointT* start, const boost::uint64_t count)
    {
      FILE* f = fopen (fileback_name_->c_str (), "a+b");

      //write at most 2 million elements at a ime
      const static size_t blocksize = static_cast<size_t> ( 2e6 );

      for (boost::uint64_t pos = 0; pos < count; pos += blocksize)
      {
        const PointT* loc = start + pos;
        if ((pos + blocksize) < count)
        {
          if (loc)
          {
            //suppressed warning. empty if statement; error?
          }
          assert (fwrite (loc, sizeof(PointT), blocksize, f) == blocksize);
        }
        else
        {
          if (loc)
          {
            //suppressed warning. empty if statement; error?
          }
          
          assert (fwrite (loc, sizeof(PointT), static_cast<size_t> (count - pos), f) == count - pos);
        }
      }

      //	int closeret = fclose(f);
      fclose (f);

      filelen_ += count;
    }
////////////////////////////////////////////////////////////////////////////////
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OCTREE_DISK_CONTAINER_IMPL_H_
