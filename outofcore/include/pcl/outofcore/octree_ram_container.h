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

#ifndef PCL_OUTOFCORE_OCTREE_RAM_CONTAINER_H_
#define PCL_OUTOFCORE_OCTREE_RAM_CONTAINER_H_

// C++
#include <vector>

// Boost
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/random/mersenne_twister.hpp>

//todo - Consider using per-node RNG (it is currently a shared static rng,
//       which is mutexed. I did i this way to be sure that node of the nodes
//       had RNGs seeded to the same value). the mutex could effect performance

/*  According to the Urban Robotics documentation, this class was not
 *  maintained with the rest of the disk_container code base. 
 */

namespace pcl
{
  namespace outofcore
  {
    template<typename PointT>
    class octree_ram_container
    {
      public:

        /** \brief empty contructor (with a path parameter?)
         */
        octree_ram_container (const boost::filesystem::path&) : container_ () { }
        
        /** \brief inserts "count" number of points into container; uses the container_ type's insert 
         *  function
         * \param[in] start - address of first point in array
         * \param[in] count - the maximum offset from start of points inserted 
         **/
        inline void
        insertRange (const PointT* start, const boost::uint64_t count);

        /** \brief inserts count points into container 
         * \param[in] start - address of first point in array
         * \param[in] count - the maximum offset from start of points inserted 
         **/
        inline void
        insertRange (const PointT* const * start, const boost::uint64_t count);

        /** \brief 
         * \param[in] start Index of first point to return from container
         * \param[in] count Offset (start + count) of the last point to return from container
         * \param[out] v Array of points read from the input range
         */
        void
        readRange (const boost::uint64_t start, const boost::uint64_t count, std::vector<PointT, Eigen::aligned_allocator<PointT> >& v);

        /** \brief grab percent*count random points. points are NOT
         *   guaranteed to be unique (could have multiple identical points!)
         *
         * \param[in] start Index of first point in range to subsample
         * \param[in] count Offset (start+count) of last point in range to subsample
         * \param[in] percent Percentage of range to return
         * \param[out] v Vector with percent*count uniformly random sampled 
         * points from given input rangerange
         */
        void
        readRangeSubSample (const boost::uint64_t start, const boost::uint64_t count, const double percent,
                            std::vector<PointT, Eigen::aligned_allocator<PointT> >& v);

        /** \brief returns the size of the vector of points stored in this class */
        inline boost::uint64_t
        size () const
        {
          return container_.size ();
        }

        /** \brief NOT IMPLEMENTED*/ //commented out to suppress warnings
        inline void
        flush (const bool force_cache_dealloc) 
        { 
          if(force_cache_dealloc)
          {
            //not implemented. what is this for?
          }
        } //???

        /** \brief clears the vector of points in this class */
        inline void
        clear ()
        {
          container_.clear ();
        }

        /** \brief Writes ascii x,y,z point data to path.string().c_str()
         *  \param path The path/filename destination of the ascii xyz data
         */
        void
        convertToXYZ (const boost::filesystem::path& path);

      private:
        //no copy construction
        octree_ram_container (const octree_ram_container& rval) { }

        octree_ram_container&
        operator= (const octree_ram_container& rval) { }

        //the actual container
        //std::deque<PointT> container;

        /** \brief linear container to hold the points */
        std::vector<PointT, Eigen::aligned_allocator<PointT> > container_;

        static boost::mutex rng_mutex_;
        static boost::mt19937 rand_gen_;
    };
  }
}


#endif //PCL_OUTOFCORE_OCTREE_RAM_CONTAINER_H_
