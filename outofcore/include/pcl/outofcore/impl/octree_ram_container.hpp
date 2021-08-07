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
 *  $Id: octree_ram_container.hpp 6927 2012-08-23 02:34:54Z stfox88 $
 */

#ifndef PCL_OUTOFCORE_RAM_CONTAINER_IMPL_H_
#define PCL_OUTOFCORE_RAM_CONTAINER_IMPL_H_

// C++
#include <sstream>

// PCL (Urban Robotics)
#include <pcl/outofcore/octree_ram_container.h>

namespace pcl
{
  namespace outofcore
  {
    template<typename PointT>
    std::mutex OutofcoreOctreeRamContainer<PointT>::rng_mutex_;

    template<typename PointT> 
    std::mt19937 OutofcoreOctreeRamContainer<PointT>::rng_ ([] {std::random_device rd; return rd(); } ());

    template<typename PointT> void
    OutofcoreOctreeRamContainer<PointT>::convertToXYZ (const boost::filesystem::path& path)
    {
      if (!container_.empty ())
      {
        FILE* fxyz = fopen (path.string ().c_str (), "we");

        std::uint64_t num = size ();
        for (std::uint64_t i = 0; i < num; i++)
        {
          const PointT& p = container_[i];

          std::stringstream ss;
          ss << std::fixed;
          ss.precision (16);
          ss << p.x << "\t" << p.y << "\t" << p.z << "\n";

          fwrite (ss.str ().c_str (), 1, ss.str ().size (), fxyz);
        }

        assert ( fclose (fxyz) == 0 );
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeRamContainer<PointT>::insertRange (const PointT* start, const std::uint64_t count)
    {
      container_.insert (container_.end (), start, start + count);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeRamContainer<PointT>::insertRange (const PointT* const * start, const std::uint64_t count)
    {
      AlignedPointTVector temp;
      temp.resize (count);
      for (std::uint64_t i = 0; i < count; i++)
      {
        temp[i] = *start[i];
      }
      container_.insert (container_.end (), temp.begin (), temp.end ());
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeRamContainer<PointT>::readRange (const std::uint64_t start, const std::uint64_t count,
                                             AlignedPointTVector& v)
    {
      v.resize (count);
      memcpy (v.data (), container_.data () + start, count * sizeof(PointT));
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeRamContainer<PointT>::readRangeSubSample (const std::uint64_t start, 
                                                      const std::uint64_t count,
                                                      const double percent, 
                                                      AlignedPointTVector& v)
    {
      std::uint64_t samplesize = static_cast<std::uint64_t> (percent * static_cast<double> (count));

      std::lock_guard<std::mutex> lock (rng_mutex_);

      std::uniform_int_distribution < std::uint64_t > buffdist (start, start + count);

      for (std::uint64_t i = 0; i < samplesize; i++)
      {
        std::uint64_t buffstart = buffdist (rng_);
        v.push_back (container_[buffstart]);
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_RAM_CONTAINER_IMPL_H_
