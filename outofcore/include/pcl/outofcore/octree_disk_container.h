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
 *  $Id: octree_disk_container.h 6927M 2012-08-24 13:26:40Z (local) $
 */

#pragma once

// C++
#include <mutex>
#include <string>

// Boost
#include <boost/uuid/random_generator.hpp>

#include <pcl/common/utils.h> // pcl::utils::ignore
#include <pcl/outofcore/octree_abstract_node_container.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

//allows operation on POSIX
#if !defined _WIN32
#define _fseeki64 fseeko
#elif defined __MINGW32__
#define _fseeki64 fseeko64
#endif

namespace pcl
{
  namespace outofcore
  {
  /** \class OutofcoreOctreeDiskContainer
   *  \note Code was adapted from the Urban Robotics out of core octree implementation. 
   *  Contact Jacob Schloss <jacob.schloss@urbanrobotics.net> with any questions. 
   *  http://www.urbanrobotics.net/
   *
   *  \brief Class responsible for serialization and deserialization of out of core point data
   *  \ingroup outofcore
   *  \author Jacob Schloss (jacob.schloss@urbanrobotics.net)
   */
    template<typename PointT = pcl::PointXYZ>
    class OutofcoreOctreeDiskContainer : public OutofcoreAbstractNodeContainer<PointT>
    {
  
      public:
        using AlignedPointTVector = typename OutofcoreAbstractNodeContainer<PointT>::AlignedPointTVector;
        
        /** \brief Empty constructor creates disk container and sets filename from random uuid string*/
        OutofcoreOctreeDiskContainer ();

        /** \brief Creates uuid named file or loads existing file
         * 
         * If \b dir is a directory, this constructor will create a new
         * uuid named file; if \b dir is an existing file, it will load the
         * file metadata for accessing the tree.
         *
         * \param[in] dir Path to the tree. If it is a directory, it
         * will create the metadata. If it is a file, it will load the metadata into memory.
         */
        OutofcoreOctreeDiskContainer (const boost::filesystem::path &dir);

        /** \brief flushes write buffer, then frees memory */
        ~OutofcoreOctreeDiskContainer () override;

        /** \brief provides random access to points based on a linear index
         */
        inline PointT
        operator[] (std::uint64_t idx) const override;

        /** \brief Adds a single point to the buffer to be written to disk when the buffer grows sufficiently large, the object is destroyed, or the write buffer is manually flushed */
        inline void
        push_back (const PointT& p);

        /** \brief Inserts a vector of points into the disk data structure */
        void
        insertRange (const AlignedPointTVector& src);

        /** \brief Inserts a PCLPointCloud2 object directly into the disk container */
        void
        insertRange (const pcl::PCLPointCloud2::Ptr &input_cloud);

        void
        insertRange (const PointT* const * start, const std::uint64_t count) override;
    
        /** \brief This is the primary method for serialization of
         * blocks of point data. This is called by the outofcore
         * octree interface, opens the binary file for appending data,
         * and writes it to disk.
         *
         * \param[in] start address of the first point to insert
         * \param[in] count offset from start of the last point to insert
         */
        void
        insertRange (const PointT* start, const std::uint64_t count) override;

        /** \brief Reads \b count points into memory from the disk container
         *
         * Reads \b count points into memory from the disk container, reading at most 2 million elements at a time
         *
         * \param[in] start index of first point to read from disk
         * \param[in] count offset of last point to read from disk
         * \param[out] dst std::vector as destination for points read from disk into memory
         */
        void
        readRange (const std::uint64_t start, const std::uint64_t count, AlignedPointTVector &dst) override;

        void
        readRange (const std::uint64_t, const std::uint64_t, pcl::PCLPointCloud2::Ptr &dst);

        /** \brief Reads the entire point contents from disk into \c output_cloud
         *  \param[out] output_cloud
         */
        int
        read (pcl::PCLPointCloud2::Ptr &output_cloud);

        /** \brief  grab percent*count random points. points are \b not guaranteed to be
         * unique (could have multiple identical points!)
         *
         * \param[in] start The starting index of points to select
         * \param[in] count The length of the range of points from which to randomly sample 
         *  (i.e. from start to start+count)
         * \param[in] percent The percentage of count that is enough points to make up this random sample
         * \param[out] dst std::vector as destination for randomly sampled points; size will 
         * be percentage*count
         */
        void
        readRangeSubSample (const std::uint64_t start, const std::uint64_t count, const double percent,
                            AlignedPointTVector &dst) override;

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
        readRangeSubSample_bernoulli (const std::uint64_t start, const std::uint64_t count, 
                                      const double percent, AlignedPointTVector& dst);

        /** \brief Returns the total number of points for which this container is responsible, \c filelen_ + points in \c writebuff_ that have not yet been flushed to the disk
         */
        std::uint64_t
        size () const override
        {
          return (filelen_ + writebuff_.size ());
        }

        /** \brief STL-like empty test
         * \return true if container has no data on disk or waiting to be written in \c writebuff_ */
        inline bool
        empty () const override
        {
          return ((filelen_ == 0) && writebuff_.empty ());
        }

        /** \brief Exposed functionality for manually flushing the write buffer during tree creation */
        void
        flush (const bool force_cache_dealloc)
        {
          flushWritebuff (force_cache_dealloc);
        }

        /** \brief Returns this objects path name */
        inline std::string&
        path ()
        {
          return (disk_storage_filename_);
        }

        inline void
        clear () override
        {
          //clear elements that have not yet been written to disk
          writebuff_.clear ();
          //remove the binary data in the directory
          PCL_DEBUG ("[Octree Disk Container] Removing the point data from disk, in file %s\n", disk_storage_filename_.c_str ());
          boost::filesystem::remove (boost::filesystem::path (disk_storage_filename_.c_str ()));
          //reset the size-of-file counter
          filelen_ = 0;
        }

        /** \brief write points to disk as ascii
         *
         * \param[in] path
         */
        void
        convertToXYZ (const boost::filesystem::path &path) override
        {
          if (boost::filesystem::exists (disk_storage_filename_))
          {
            FILE* fxyz = fopen (path.string ().c_str (), "we");

            FILE* f = fopen (disk_storage_filename_.c_str (), "rbe");
            assert (f != nullptr);

            std::uint64_t num = size ();
            PointT p;
            char* loc = reinterpret_cast<char*> ( &p );

            for (std::uint64_t i = 0; i < num; i++)
            {
              int seekret = _fseeki64 (f, i * sizeof (PointT), SEEK_SET);
              pcl::utils::ignore(seekret);
              assert (seekret == 0);
              std::size_t readlen = fread (loc, sizeof (PointT), 1, f);
              pcl::utils::ignore(readlen);
              assert (readlen == 1);

              //of << p.x << "\t" << p.y << "\t" << p.z << "\n";
              std::stringstream ss;
              ss << std::fixed;
              ss.precision (16);
              ss << p.x << "\t" << p.y << "\t" << p.z << "\n";

              fwrite (ss.str ().c_str (), 1, ss.str ().size (), fxyz);
            }
            int res = fclose (f);
            pcl::utils::ignore(res);
            assert (res == 0);
            res = fclose (fxyz);
            assert (res == 0);
          }
        }

        /** \brief Generate a universally unique identifier (UUID)
         *
         * A mutex lock happens to ensure uniqueness
         *
         */
        static void
        getRandomUUIDString (std::string &s);

        /** \brief Returns the number of points in the PCD file by reading the PCD header. */
        std::uint64_t
        getDataSize () const;
        
      private:
        //no copy construction
        OutofcoreOctreeDiskContainer (const OutofcoreOctreeDiskContainer& /*rval*/) { }


        OutofcoreOctreeDiskContainer&
        operator= (const OutofcoreOctreeDiskContainer& /*rval*/) { }

        void
        flushWritebuff (const bool force_cache_dealloc);
    
        /** \brief Name of the storage file on disk (i.e., the PCD file) */
        std::string disk_storage_filename_;

        //--- possibly deprecated parameter variables --//

        //number of elements in file
        std::uint64_t filelen_;

        /** \brief elements [0,...,size()-1] map to [filelen, ..., filelen + size()-1] */
        AlignedPointTVector writebuff_;

        const static std::uint64_t READ_BLOCK_SIZE_;

        static const std::uint64_t WRITE_BUFF_MAX_;

        static std::mutex rng_mutex_;
        static boost::mt19937 rand_gen_;
        static boost::uuids::basic_random_generator<boost::mt19937> uuid_gen_;

    };
  } //namespace outofcore
} //namespace pcl
