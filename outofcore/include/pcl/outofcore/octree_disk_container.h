#pragma once

/*
 Copyright (c) 2012, Urban Robotics Inc
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of Urban Robotics Inc nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 This code defines the octree used for point storage at Urban Robotics. Please
 contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions.
 http://www.urbanrobotics.net/
 */

// C++
#include <vector>
#include <string>

// Boost
#include <boost/filesystem.hpp>
#pragma warning(push)
#pragma warning(disable: 4311 4312)
#include <boost/thread.hpp>
#pragma warning(pop)
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>

//allows operation on POSIX
#ifndef WIN32
#define _fseeki64 fseeko
#endif

/* todo (from original UR code): consider using per-node RNG (it is
 *  currently a shared static rng, which is mutexed. I did i this way
 *  to be sure that none of the nodes had RNGs seeded to the same
 *  value). the mutex could effect performance though 
 */

/** \class octree_disk_container 
 *  
 */

template<typename PointType>
class octree_disk_container
{
  
  public:
    octree_disk_container ();

    /** \brief Creates uuid named file or loads existing file
     * 
     * If dir is a directory, constructor will create new
     * uuid named file; if dir is an existing file, it will load the
     * file
     */
    octree_disk_container (const boost::filesystem::path& dir);
    ~octree_disk_container ();

    inline PointType
    operator[] (boost::uint64_t idx);

    inline void
    push_back (const PointType& p);

    void
    insertRange (const PointType* const * start, const boost::uint64_t count)
    {
      //copy the handles to a continous block
      PointType* arr = new PointType[count];
      for (size_t i = 0; i < count; i++)
      {
        arr[i] = *(start[i]);
      }
      insertRange (arr, count);
      delete[] arr;
    }

    void
    insertRange (const PointType* start, const boost::uint64_t count);

    void
    readRange (const boost::uint64_t start, const boost::uint64_t count, std::vector<PointType>& v);


    /** \brief  grab percent*count random points. points are _not_ guaranteed to be
     * unique (could have multiple identical points!)
     *
     * \param start
     * \param count
     * \param percent
     * \param v
     */
    void
    readRangeSubSample (const boost::uint64_t start, const boost::uint64_t count, const double percent,
                        std::vector<PointType>& v);

    /** \brief use bernoulli trials to select points. points are unique
     *
     * \param start
     * \param count
     * \param percent
     * \param v
     */
    void
    readRangeSubSample_bernoulli (const boost::uint64_t start, const boost::uint64_t count, 
                                  const double percent, std::vector<PointType>& v);

    boost::uint64_t
    size () const
    {
      return filelen + writebuff.size ();
    }

    bool
    empty ()
    {
      return ((filelen == 0) && writebuff.empty ());
    }

    void
    flush (const bool forceCacheDeAlloc)
    {
      flush_writebuff (forceCacheDeAlloc);
    }

    inline std::string&
    path ()
    {
      return *fileback_name;
    }

    inline void
    clear ()
    {
      writebuff.clear ();
      boost::filesystem::remove (boost::filesystem::path (fileback_name->c_str ()));
      filelen = 0;
    }

    void
    convertToXYZ (const boost::filesystem::path& path)
    {
      if (boost::filesystem::exists (*fileback_name))
      {
        FILE* fxyz = fopen (path.string ().c_str (), "w");

        FILE* f = fopen (fileback_name->c_str (), "rb");
        assert (f != NULL);

        boost::uint64_t num = size ();
        PointType p;
        char* loc = (char*)&p;
        for (boost::uint64_t i = 0; i < num; i++)
        {
          int seekret = _fseeki64 (f, i * sizeof(PointType), SEEK_SET);
          assert (seekret == 0);
          size_t readlen = fread (loc, sizeof(PointType), 1, f);
          assert (readlen == 1);

          //of << p.x << "\t" << p.y << "\t" << p.z << "\n";
          std::stringstream ss;
          ss << std::fixed;
          ss.precision (16);
          ss << p.x << "\t" << p.y << "\t" << p.z << "\n";

          fwrite (ss.str ().c_str (), 1, ss.str ().size (), fxyz);
        }

        int closeret = fclose (f);
        int closeretxyz = fclose (fxyz);
      }
    }

    /** \brief Generate a universally unique identifier (UUID)
     *
     * A mutex lock happens to ensure uniquness
     *
     * \todo Does this need to be on a templated class?  Seems like this could
     * be a general utility function.
     *
     */
    static void
    getRandomUUIDString (std::string& s);

  private:
    //no copy construction
    octree_disk_container (const octree_disk_container& rval) { }

    octree_disk_container&
    operator= (const octree_disk_container& rval) { }

    void
    flush_writebuff (const bool forceCacheDeAlloc);
    
    //elements [0,...,size()-1] map to [filelen, ..., filelen + size()-1]
    std::vector<PointType> writebuff;

    //std::fstream fileback;//elements [0,...,filelen-1]
    std::string *fileback_name;


    //number of elements in file
    boost::uint64_t filelen;

    //static const size_t writebuffmax = 100000;
    static const size_t writebuffmax = 50000;
    //static const size_t writebuffmax = 10000;
    //static const size_t writebuffmax = 1000;
    //static const size_t writebuffmax = 100;
    //static const size_t writebuffmax = 0;

    //boost::posix_time::ptime lastwrite;

    static boost::mutex rng_mutex;
    static boost::mt19937 rand_gen;
    static boost::uuids::random_generator uuid_gen;
};

