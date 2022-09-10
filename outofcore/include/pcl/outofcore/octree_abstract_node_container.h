/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *  $Id: octree_abstract_node_container.h 6802M 2012-08-25 00:11:05Z (local) $
 */

#pragma once

#include <boost/filesystem.hpp>

#include <mutex>
#include <vector>

namespace pcl
{
  namespace outofcore
  {
    template<typename PointT>
    class OutofcoreAbstractNodeContainer 
    {

      public:
        using AlignedPointTVector = std::vector<PointT, Eigen::aligned_allocator<PointT> >;

        OutofcoreAbstractNodeContainer () 
          : container_ ()
        {}

        OutofcoreAbstractNodeContainer (const boost::filesystem::path&) {}

        virtual 
        ~OutofcoreAbstractNodeContainer () = default;        

        virtual void
        insertRange (const PointT* start, const std::uint64_t count)=0;
        
        virtual void
        insertRange (const PointT* const* start, const std::uint64_t count)=0;

        virtual void
        readRange (const std::uint64_t start, const std::uint64_t count, AlignedPointTVector& v)=0;
        
        virtual void
        readRangeSubSample (const std::uint64_t start, const std::uint64_t count, const double percent, AlignedPointTVector& v) =0;

        virtual bool
        empty () const=0;
        
        virtual std::uint64_t
        size () const =0;
        
        virtual void
        clear ()=0;

        virtual void
        convertToXYZ (const boost::filesystem::path& path)=0;

        virtual PointT
        operator[] (std::uint64_t idx) const=0;

      protected:
        OutofcoreAbstractNodeContainer (const OutofcoreAbstractNodeContainer& rval);

        AlignedPointTVector container_;
        
        static std::mutex rng_mutex_;
    };
  }//namespace outofcore
}//namespace pcl
