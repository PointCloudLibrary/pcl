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
#ifndef PCL_OUTOFCORE_OCTREE_DISK_CONTAINER_H_
#define PCL_OUTOFCORE_OCTREE_DISK_CONTAINER_H_

// C++
#include <vector>
#include <string>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/bernoulli_distribution.hpp>

#include <pcl/outofcore/octree_abstract_node_container.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>

//allows operation on POSIX
#ifndef WIN32
#define _fseeki64 fseeko
#endif

/* todo (from original UR code): consider using per-node RNG (it is
 *  currently a shared static rng, which is mutexed. I did i this way
 *  to be sure that none of the nodes had RNGs seeded to the same
 *  value). the mutex could effect performance though 
 *
 */

namespace pcl
{
  namespace outofcore
  {
/** \class octree_disk_container
 *  \brief Class responsible for serialization and deserialization of out of core point data
 */
    template<typename PointT>
    class octree_disk_container : public OutofcoreAbstractNodeContainer<PointT>
    {
  
      public:
        typedef typename OutofcoreAbstractNodeContainer<PointT>::AlignedPointTVector AlignedPointTVector;
        
        /** \brief Empty constructor creates disk container and sets filename from random uuid string*/
        octree_disk_container ();

        /** \brief Creates uuid named file or loads existing file
         * 
         * If dir is a directory, constructor will create new
         * uuid named file; if dir is an existing file, it will load the
         * file
         */
        octree_disk_container (const boost::filesystem::path& dir);

        /** \brief flushes write buffer, then frees memory */
        ~octree_disk_container ();

        /** \brief provides random access to points based on a linear index
         *  \todo benchmark this; compare to Julius' octree
         */
        inline PointT
        operator[] (uint64_t idx) const;

        inline void
        push_back (const PointT& p);

        void
        insertRange (const AlignedPointTVector& src);

        /** \todo standardize the interface for writing binary data to the files .oct_dat files */
        void
        insertRange (const PointT* const * start, const uint64_t count)
        {
          //copy the handles to a continuous block
          PointT* arr = new PointT[count];

          //copy from start of array, element by element
          for (size_t i = 0; i < count; i++)
          {
            arr[i] = *(start[i]);
          }

          insertRange (arr, count);
          delete[] arr;
        }
    
        /** \brief This is the primary method for serialization of
         * blocks of point data. This is called by the outofcore
         * octree interface, opens the binary file for appending data,
         * and writes it to disk.
         *
         * \todo With the current implementation of \b insertRange,
         * frequently the file is open and sometimes 1-3 points are
         * written to disk. This might be normal if the resolution is
         * low enough, but performance could be improved if points are
         * first sorted into buckets by locational code, then sent to
         * their appropriate container for being written to
         * disk. Furthermore, for OUTOFCORE_VERSION_ >= 3, this will
         * use PCD files (and possibly Julius's octree compression)
         * rather than an unorganized linear dump of the binary PointT
         * data.
         *
         * \param[in] start address of the first point to insert
         * \param[in] count offset from start of the last point to insert
         */
        void
        insertRange (const PointT* start, const uint64_t count);

        /** \brief Reads \b count points into memory from the disk container
         *
         * Reads \b count points into memory from the disk container, reading at most 2 million elements at a time
         * \todo Investigate most efficient ways to read in points v. block size
         *
         * \param[in] start index of first point to read from disk
         * \param[in] count offset of last point to read from disk
         * \param[out] v std::vector as destination for points read from disk into memory
         */
        void
        readRange (const uint64_t start, const uint64_t count, AlignedPointTVector& dst);

        void
        readRange (const uint64_t start, const uint64_t count, sensor_msgs::PointCloud2& dst)
        {
          if (count == 0)
          {
            PCL_DEBUG ( "[pcl::outofcore::octree_disk_container] No points requested for reading\n" );
            return;
          }

          if ((start + count) > size ())
          {
            PCL_ERROR ( "[pcl::outofcore::octree_disk_container] Indicies out of range; start + count exceeds the size of the stored points\n" );
            PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::octree_disk_container] Outofcore Octree Exception: Read indices exceed range");
          }

          uint64_t filestart = 0;
          uint64_t filecount = 0;

          pcl::PCDReader reader;

          Eigen::Vector4f  origin;
          Eigen::Quaternionf  orientation;
          int  pcd_version;

          //something wrong here ?
          assert ( reader.read ( *fileback_name_, dst, origin, orientation, pcd_version) == 0 );
        }


        /** \brief  grab percent*count random points. points are \b not guaranteed to be
         * unique (could have multiple identical points!)
         *
         * \todo improve the random sampling of octree points from the disk container
         *
         * \param[in] start The starting index of points to select
         * \param count[in] The length of the range of points from which to randomly sample 
         *  (i.e. from start to start+count)
         * \param percent[in] The percentage of count that is enough points to make up this random sample
         * \param dst[out] std::vector as destination for randomly sampled points; size will 
         * be percentage*count
         */
        void
        readRangeSubSample (const uint64_t start, const uint64_t count, const double percent,
                            AlignedPointTVector& dst);

        /** \brief Use bernoulli trials to select points. All points selected will be unique.
         *
         * \param[in] start The starting index of points to select
         * \param[in] count The length of the range of points from which to randomly sample 
         *  (i.e. from start to start+count)
         * \param[in] percent The percentage of count that is enough points to make up this random sample
         * \param[out] dst std::vector as destination for randomly sampled points; size will 
         * be percentage*count
         */
        void
        readRangeSubSample_bernoulli (const uint64_t start, const uint64_t count, 
                                      const double percent, AlignedPointTVector& dst);

        /** \brief Returns the total number of points for which this container is responsible, \ref filelen_ + points in \ref writebuff_ that have not yet been flushed to the disk
         */
        uint64_t
        size () const
        {
          return filelen_ + writebuff_.size ();
        }

        /** \brief STL-like empty test
         * \return true if container has no data on disk or waiting to be written in \ref writebuff_ */
        inline bool
        empty () const
        {
          return ((filelen_ == 0) && writebuff_.empty ());
        }

        void
        flush (const bool force_cache_dealloc)
        {
          flushWritebuff (force_cache_dealloc);
        }

        inline std::string&
        path ()
        {
          return *fileback_name_;
        }

        inline void
        clear ()
        {
          //clear elements that have not yet been written to disk
          writebuff_.clear ();
          //remove the binary data in the directory
          PCL_DEBUG ("[Octree Disk Container] Removing the point data from disk, in file %s\n",fileback_name_->c_str ());
          boost::filesystem::remove (boost::filesystem::path (fileback_name_->c_str ()));
          //reset the size-of-file counter
          filelen_ = 0;
        }

        /** \brief write points to disk as ascii
         *
         * \param[in] path
         */
        void
        convertToXYZ (const boost::filesystem::path& path)
        {
          if (boost::filesystem::exists (*fileback_name_))
          {
            FILE* fxyz = fopen (path.string ().c_str (), "w");

            FILE* f = fopen (fileback_name_->c_str (), "rb");
            assert (f != NULL);

            uint64_t num = size ();
            PointT p;
            char* loc = reinterpret_cast<char*> ( &p );

            for (uint64_t i = 0; i < num; i++)
            {
              int seekret = _fseeki64 (f, i * sizeof(PointT), SEEK_SET);
              assert (seekret == 0);
              size_t readlen = fread (loc, sizeof(PointT), 1, f);
              assert (readlen == 1);

              //of << p.x << "\t" << p.y << "\t" << p.z << "\n";
              std::stringstream ss;
              ss << std::fixed;
              ss.precision (16);
              ss << p.x << "\t" << p.y << "\t" << p.z << "\n";

              fwrite (ss.str ().c_str (), 1, ss.str ().size (), fxyz);
            }
            assert ( fclose (f) == 0 );
            assert ( fclose (fxyz) == 0);
          }
        }

        /** \brief Generate a universally unique identifier (UUID)
         *
         * A mutex lock happens to ensure uniquness
         *
         * \todo Does this need to be on a templated class?  Seems like this could
         * be a general utility function.
         * \todo Does this need to be random?
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
        flushWritebuff (const bool force_cache_dealloc);
    
        /** \brief elements [0,...,size()-1] map to [filelen, ..., filelen + size()-1] */
        AlignedPointTVector writebuff_;

        //std::fstream fileback;//elements [0,...,filelen-1]
        std::string *fileback_name_;

        //number of elements in file
        uint64_t filelen_;

        const static uint64_t READ_BLOCK_SIZE_;

        /** \todo Consult with the literature about optimizing out of core read/write */
        /** \todo this will be handled by the write method in pcl::FileWriter */
        /** \todo WRITE_BUFF_MAX_ is something of a misnomer; this is
         *  used in two different ways in the class which accounts for
         *  a bug. It should be the maximum number of points written
         *  to disk. However, in some places it is also used as the
         *  maximum number of points allowed to exist in \b writebuff_
         *  at any given time.
         */
        static const uint64_t
        WRITE_BUFF_MAX_;

        static boost::mutex rng_mutex_;
        static boost::mt19937 rand_gen_;
        static boost::uuids::random_generator uuid_gen_;
    };
  } //namespace outofcore
} //namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_DISK_CONTAINER_H_
