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
 *  $Id: octree_disk_container.hpp 6927M 2012-08-24 17:01:57Z (local) $
 */

#ifndef PCL_OUTOFCORE_OCTREE_DISK_CONTAINER_IMPL_H_
#define PCL_OUTOFCORE_OCTREE_DISK_CONTAINER_IMPL_H_

// C++
#include <sstream>
#include <cassert>
#include <ctime>

// Boost
#include <pcl/outofcore/boost.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

// PCL (Urban Robotics)
#include <pcl/outofcore/octree_disk_container.h>

//allows operation on POSIX
#if !defined WIN32
#define _fseeki64 fseeko
#elif defined __MINGW32__
#define _fseeki64 fseeko64
#endif

namespace pcl
{
  namespace outofcore
  {
    template<typename PointT>
    boost::mutex OutofcoreOctreeDiskContainer<PointT>::rng_mutex_;

    template<typename PointT> boost::mt19937
    OutofcoreOctreeDiskContainer<PointT>::rand_gen_ (static_cast<unsigned int> (std::time(NULL)));

    template<typename PointT>
    boost::uuids::random_generator OutofcoreOctreeDiskContainer<PointT>::uuid_gen_ (&rand_gen_);

    template<typename PointT>
    const uint64_t OutofcoreOctreeDiskContainer<PointT>::READ_BLOCK_SIZE_ = static_cast<uint64_t> (2e12);
    template<typename PointT>
    const uint64_t OutofcoreOctreeDiskContainer<PointT>::WRITE_BUFF_MAX_ = static_cast<uint64_t> (2e12);

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::getRandomUUIDString (std::string& s)
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
    OutofcoreOctreeDiskContainer<PointT>::OutofcoreOctreeDiskContainer () 
      : disk_storage_filename_ ()
      , filelen_ (0)
      , writebuff_ (0)
    {
      std::string temp;
      getRandomUUIDString (temp);
      disk_storage_filename_ = boost::shared_ptr<std::string> (new std::string (temp));
      filelen_ = 0;
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT>
    OutofcoreOctreeDiskContainer<PointT>::OutofcoreOctreeDiskContainer (const boost::filesystem::path& path)
      : disk_storage_filename_ ()
      , filelen_ (0)
      , writebuff_ (0)
    {
      if (boost::filesystem::exists (path))
      {
        if (boost::filesystem::is_directory (path))
        {
          std::string uuid;
          getRandomUUIDString (uuid);
          boost::filesystem::path filename (uuid);
          boost::filesystem::path file = path / filename;

          disk_storage_filename_ = boost::shared_ptr<std::string> (new std::string (file.string ()));
        }
        else
        {
          uint64_t len = boost::filesystem::file_size (path);

          disk_storage_filename_ = boost::shared_ptr<std::string> (new std::string (path.string ()));

          filelen_ = len / sizeof(PointT);

          pcl::PCLPointCloud2 cloud_info;
          Eigen::Vector4f origin;
          Eigen::Quaternionf orientation;
          int pcd_version;
          int data_type;
          unsigned int data_index;
          
          //read the header of the pcd file and get the number of points
          PCDReader reader;
          reader.readHeader (*disk_storage_filename_, cloud_info, origin, orientation, pcd_version, data_type, data_index, 0);
          
          filelen_ = cloud_info.width * cloud_info.height;
        }
      }
      else //path doesn't exist
      {
        disk_storage_filename_ = boost::shared_ptr<std::string> (new std::string (path.string ()));
        filelen_ = 0;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT>
    OutofcoreOctreeDiskContainer<PointT>::~OutofcoreOctreeDiskContainer ()
    {
      flushWritebuff (true);
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::flushWritebuff (const bool force_cache_dealloc)
    {
      if (writebuff_.size () > 0)
      {
        //construct the point cloud for this node
        typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        
        cloud->width = static_cast<uint32_t> (writebuff_.size ());
        cloud->height = 1;

        cloud->points = writebuff_;

        //write data to a pcd file
        pcl::PCDWriter writer;


        PCL_WARN ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] Flushing writebuffer in a dangerous way to file %s. This might overwrite data in destination file\n", __FUNCTION__, disk_storage_filename_->c_str ());
        
        // Write ascii for now to debug
        int res = writer.writeBinaryCompressed (*disk_storage_filename_, *cloud);
        (void)res;
        assert (res == 0);
        if (force_cache_dealloc)
        {
          writebuff_.resize (0);
        }
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> PointT
    OutofcoreOctreeDiskContainer<PointT>::operator[] (uint64_t idx) const
    {
      PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeDiskContainer] Not implemented for use with PCL library\n");
      
      //if the index is on disk
      if (idx < filelen_)
      {

        PointT temp;
        //open our file
        FILE* f = fopen (disk_storage_filename_->c_str (), "rb");
        assert (f != NULL);

        //seek the right length; 
        int seekret = _fseeki64 (f, idx * sizeof(PointT), SEEK_SET);
        (void)seekret;
        assert (seekret == 0);

        size_t readlen = fread (&temp, 1, sizeof(PointT), f);
        (void)readlen;
        assert (readlen == sizeof (PointT));

        int res = fclose (f);
        (void)res;
        assert (res == 0);

        return (temp);
      }
      //otherwise if the index is still in the write buffer
      if (idx < (filelen_ + writebuff_.size ()))
      {
        idx -= filelen_;
        return (writebuff_[idx]);
      }

      //else, throw out of range exception
      PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore:OutofcoreOctreeDiskContainer] Index is out of range");
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::readRange (const uint64_t start, const uint64_t count, AlignedPointTVector& dst)
    {
      if (count == 0)
      {
        return;
      }

      if ((start + count) > size ())
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] Indicies out of range; start + count exceeds the size of the stored points\n", __FUNCTION__);
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeDiskContainer] Outofcore Octree Exception: Read indices exceed range");
      }

      pcl::PCDReader reader;
      typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> ());
      
      int res = reader.read (*disk_storage_filename_, *cloud);
      (void)res;
      assert (res == 0);
      
      for (size_t i=0; i < cloud->points.size (); i++)
        dst.push_back (cloud->points[i]);
      
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::readRangeSubSample_bernoulli (const uint64_t start, const uint64_t count, const double percent, AlignedPointTVector& dst)
    {
      if (count == 0)
      {
        return;
      }

      dst.clear ();

      uint64_t filestart = 0;
      uint64_t filecount = 0;

      int64_t buffstart = -1;
      int64_t buffcount = -1;

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

          for (size_t i = buffstart; i < static_cast<uint64_t> (buffcount); i++)
          {
            if (buffcoin ())
            {
              dst.push_back (writebuff_[i]);
            }
          }
        }
      }

      if (filecount > 0)
      {
        //pregen and then sort the offsets to reduce the amount of seek
        std::vector < uint64_t > offsets;
        {
          boost::mutex::scoped_lock lock (rng_mutex_);

          boost::bernoulli_distribution<double> filedist (percent);
          boost::variate_generator<boost::mt19937&, boost::bernoulli_distribution<double> > filecoin (rand_gen_, filedist);
          for (uint64_t i = filestart; i < (filestart + filecount); i++)
          {
            if (filecoin ())
            {
              offsets.push_back (i);
            }
          }
        }
        std::sort (offsets.begin (), offsets.end ());

        FILE* f = fopen (disk_storage_filename_->c_str (), "rb");
        assert (f != NULL);
        PointT p;
        char* loc = reinterpret_cast<char*> (&p);
        
        uint64_t filesamp = offsets.size ();
        for (uint64_t i = 0; i < filesamp; i++)
        {
          int seekret = _fseeki64 (f, offsets[i] * static_cast<uint64_t> (sizeof(PointT)), SEEK_SET);
          (void)seekret;
          assert (seekret == 0);
          size_t readlen = fread (loc, sizeof(PointT), 1, f);
          (void)readlen;
          assert (readlen == 1);

          dst.push_back (p);
        }

        fclose (f);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

//change this to use a weighted coin flip, to allow sparse sampling of small clouds (eg the bernoulli above)
    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::readRangeSubSample (const uint64_t start, const uint64_t count, const double percent, AlignedPointTVector& dst)
    {
      if (count == 0)
      {
        return;
      }

      dst.clear ();

      uint64_t filestart = 0;
      uint64_t filecount = 0;

      int64_t buffcount = -1;

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
        buffcount = count - filecount;
      }

      uint64_t filesamp = static_cast<uint64_t> (percent * static_cast<double> (filecount));
      
      uint64_t buffsamp = (buffcount > 0) ? (static_cast<uint64_t > (percent * static_cast<double> (buffcount))) : 0;

      if ((filesamp == 0) && (buffsamp == 0) && (size () > 0))
      {
        //std::cerr << "would not add points to LOD, falling back to bernoulli";
        readRangeSubSample_bernoulli (start, count, percent, dst);
        return;
      }

      if (buffcount > 0)
      {
        {
          boost::mutex::scoped_lock lock (rng_mutex_);

          boost::uniform_int < uint64_t > buffdist (0, buffcount - 1);
          boost::variate_generator<boost::mt19937&, boost::uniform_int<uint64_t> > buffdie (rand_gen_, buffdist);

          for (uint64_t i = 0; i < buffsamp; i++)
          {
            uint64_t buffstart = buffdie ();
            dst.push_back (writebuff_[buffstart]);
          }
        }
      }

      if (filesamp > 0)
      {
        //pregen and then sort the offsets to reduce the amount of seek
        std::vector < uint64_t > offsets;
        {
          boost::mutex::scoped_lock lock (rng_mutex_);

          offsets.resize (filesamp);
          boost::uniform_int < uint64_t > filedist (filestart, filestart + filecount - 1);
          boost::variate_generator<boost::mt19937&, boost::uniform_int<uint64_t> > filedie (rand_gen_, filedist);
          for (uint64_t i = 0; i < filesamp; i++)
          {
            uint64_t _filestart = filedie ();
            offsets[i] = _filestart;
          }
        }
        std::sort (offsets.begin (), offsets.end ());

        FILE* f = fopen (disk_storage_filename_->c_str (), "rb");
        assert (f != NULL);
        PointT p;
        char* loc = reinterpret_cast<char*> (&p);
        for (uint64_t i = 0; i < filesamp; i++)
        {
          int seekret = _fseeki64 (f, offsets[i] * static_cast<uint64_t> (sizeof(PointT)), SEEK_SET);
          (void)seekret;
          assert (seekret == 0);
          size_t readlen = fread (loc, sizeof(PointT), 1, f);
          (void)readlen;
          assert (readlen == 1);

          dst.push_back (p);
        }
        int res = fclose (f);
        (void)res;
        assert (res == 0);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::push_back (const PointT& p)
    {
      writebuff_.push_back (p);
      if (writebuff_.size () > WRITE_BUFF_MAX_)
      {
        flushWritebuff (false);
      }
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::insertRange (const AlignedPointTVector& src)
    {
      const uint64_t count = src.size ();
      
      typename pcl::PointCloud<PointT>::Ptr tmp_cloud (new pcl::PointCloud<PointT> ());
      
      // If there's a pcd file with data          
      if (boost::filesystem::exists (*disk_storage_filename_))
      {
        // Open the existing file
        pcl::PCDReader reader;
        int res = reader.read (*disk_storage_filename_, *tmp_cloud);
        (void)res;
        assert (res == 0);
      }
      // Otherwise create the point cloud which will be saved to the pcd file for the first time
      else 
      {
        tmp_cloud->width = static_cast<uint32_t> (count + writebuff_.size ());
        tmp_cloud->height = 1;
      }            

      for (size_t i = 0; i < src.size (); i++)
        tmp_cloud->points.push_back (src[i]);
      
      // If there are any points in the write cache writebuff_, a different write cache than this one, concatenate
      for (size_t i = 0; i < writebuff_.size (); i++)
      {
        tmp_cloud->points.push_back (writebuff_[i]);
      }

      //assume unorganized point cloud
      tmp_cloud->width = static_cast<uint32_t> (tmp_cloud->points.size ());
            
      //save and close
      PCDWriter writer;
      
      int res = writer.writeBinaryCompressed (*disk_storage_filename_, *tmp_cloud);
      (void)res;
      assert (res == 0);
    }
  
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::insertRange (const pcl::PCLPointCloud2::Ptr& input_cloud)
    {
      pcl::PCLPointCloud2::Ptr tmp_cloud (new pcl::PCLPointCloud2 ());
          
      //if there's a pcd file with data associated with this node, read the data, concatenate, and resave
      if (boost::filesystem::exists (*disk_storage_filename_))
      {
        //open the existing file
        pcl::PCDReader reader;
        int res = reader.read (*disk_storage_filename_, *tmp_cloud);
        (void)res;
        assert (res == 0);
        pcl::PCDWriter writer;
        PCL_DEBUG ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] Concatenating point cloud from %s to new cloud\n", __FUNCTION__, disk_storage_filename_->c_str ());
        
        size_t previous_num_pts = tmp_cloud->width*tmp_cloud->height + input_cloud->width*input_cloud->height;
        //Concatenate will fail if the fields in input_cloud do not match the fields in the PCD file.
        pcl::concatenatePointCloud (*tmp_cloud, *input_cloud, *tmp_cloud);
        size_t res_pts = tmp_cloud->width*tmp_cloud->height;
        
        (void)previous_num_pts;
        (void)res_pts;
        
        assert (previous_num_pts == res_pts);
        
        writer.writeBinaryCompressed (*disk_storage_filename_, *tmp_cloud);
            
      }
      else //otherwise create the point cloud which will be saved to the pcd file for the first time
      {
        pcl::PCDWriter writer;
        int res = writer.writeBinaryCompressed (*disk_storage_filename_, *input_cloud);
        (void)res;
        assert (res == 0);
      }            

    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::readRange (const uint64_t, const uint64_t, pcl::PCLPointCloud2::Ptr& dst)
    {
      pcl::PCDReader reader;

      Eigen::Vector4f  origin;
      Eigen::Quaternionf  orientation;
      int  pcd_version;
          
      if (boost::filesystem::exists (*disk_storage_filename_))
      {
//            PCL_INFO ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] Reading points from disk from %s.\n", __FUNCTION__ , disk_storage_filename_->c_str ());
        int res = reader.read (*disk_storage_filename_, *dst, origin, orientation, pcd_version);
        (void)res;
        assert (res != -1);
      }
      else
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] File %s does not exist in node.\n", __FUNCTION__, disk_storage_filename_->c_str ());
      }
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> int
    OutofcoreOctreeDiskContainer<PointT>::read (pcl::PCLPointCloud2::Ptr& output_cloud)
    {
      pcl::PCLPointCloud2::Ptr temp_output_cloud (new pcl::PCLPointCloud2 ());

      if (boost::filesystem::exists (*disk_storage_filename_))
      {
//            PCL_INFO ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] Reading points from disk from %s.\n", __FUNCTION__ , disk_storage_filename_->c_str ());
        int res = pcl::io::loadPCDFile (*disk_storage_filename_, *temp_output_cloud);
        (void)res;
        assert (res != -1);
        if(res == -1)
          return (-1);
      }
      else
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeDiskContainer::%s] File %s does not exist in node.\n", __FUNCTION__, disk_storage_filename_->c_str ()); 
        return (-1);
      }

      if(output_cloud.get () != 0)
      {
        pcl::concatenatePointCloud (*output_cloud, *temp_output_cloud, *output_cloud);
      }
      else
      {
        output_cloud = temp_output_cloud;
      }
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::insertRange (const PointT* const * start, const uint64_t count)
    {
//      PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeDiskContainer] Deprecated\n");
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
    
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> void
    OutofcoreOctreeDiskContainer<PointT>::insertRange (const PointT* start, const uint64_t count)
    {
      typename pcl::PointCloud<PointT>::Ptr tmp_cloud (new pcl::PointCloud<PointT> ());

      // If there's a pcd file with data, read it in from disk for appending
      if (boost::filesystem::exists (*disk_storage_filename_))
      {
        pcl::PCDReader reader;
        // Open it
        int res = reader.read (disk_storage_filename_->c_str (), *tmp_cloud);
        (void)res; 
        assert (res == 0);
      }
      else //otherwise create the pcd file
      {
        tmp_cloud->width = static_cast<uint32_t> (count) + static_cast<uint32_t> (writebuff_.size ());
        tmp_cloud->height = 1;
      }            

      // Add any points in the cache
      for (size_t i = 0; i < writebuff_.size (); i++)
      {
        tmp_cloud->points.push_back (writebuff_ [i]);
      }

      //add the new points passed with this function
      for (size_t i = 0; i < count; i++)
      {
        tmp_cloud->points.push_back (*(start + i));
      }

      tmp_cloud->width = static_cast<uint32_t> (tmp_cloud->points.size ());
      tmp_cloud->height = 1;
            
      //save and close
      PCDWriter writer;

      int res = writer.writeBinaryCompressed (*disk_storage_filename_, *tmp_cloud);
      (void)res;
      assert (res == 0);
    }
    ////////////////////////////////////////////////////////////////////////////////

    template<typename PointT> boost::uint64_t
    OutofcoreOctreeDiskContainer<PointT>::getDataSize () const
    {
      pcl::PCLPointCloud2 cloud_info;
      Eigen::Vector4f origin;
      Eigen::Quaternionf orientation;
      int pcd_version;
      int data_type;
      unsigned int data_index;
      
      //read the header of the pcd file and get the number of points
      PCDReader reader;
      reader.readHeader (*disk_storage_filename_, cloud_info, origin, orientation, pcd_version, data_type, data_index, 0);
      
      boost::uint64_t total_points = cloud_info.width * cloud_info.height + writebuff_.size ();

      return (total_points);
    }
    ////////////////////////////////////////////////////////////////////////////////

  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_OCTREE_DISK_CONTAINER_IMPL_H_
