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
 *  $Id$
 *
 */

#include <pcl/outofcore/outofcore_node_data.h>

#include <pcl/console/print.h>

#include <pcl/pcl_macros.h>
#include <pcl/exceptions.h> // for PCL_THROW_EXCEPTION, PCLException

#include <iostream>
#include <fstream>
#include <memory>

namespace pcl
{
  namespace outofcore
  {
    
    OutofcoreOctreeNodeMetadata::OutofcoreOctreeNodeMetadata () 
      : outofcore_version_ ()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////

    OutofcoreOctreeNodeMetadata::~OutofcoreOctreeNodeMetadata () = default;

    ////////////////////////////////////////////////////////////////////////////////

    OutofcoreOctreeNodeMetadata::OutofcoreOctreeNodeMetadata (const OutofcoreOctreeNodeMetadata& orig)
    {
      this->min_bb_ = orig.min_bb_;
      this->max_bb_ = orig.max_bb_;
      this->binary_point_filename_ = orig.binary_point_filename_;
      this->midpoint_xyz_ = orig.midpoint_xyz_;
      this->directory_ = orig.directory_;
      this->metadata_filename_ = orig.metadata_filename_;
      this->outofcore_version_ = orig.outofcore_version_;

      this->updateVoxelCenter ();
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    const Eigen::Vector3d&
    OutofcoreOctreeNodeMetadata::getBoundingBoxMin () const
    {
      return (min_bb_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::setBoundingBoxMin (const Eigen::Vector3d& min_bb)
    {
      min_bb_ = min_bb;
      this->updateVoxelCenter ();
    }

    ////////////////////////////////////////////////////////////////////////////////

    const Eigen::Vector3d&
    OutofcoreOctreeNodeMetadata::getBoundingBoxMax () const
    {
      return (max_bb_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::setBoundingBoxMax (const Eigen::Vector3d& max_bb)
    {
      max_bb_ = max_bb;
      this->updateVoxelCenter ();
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::getBoundingBox (Eigen::Vector3d &min_bb, Eigen::Vector3d &max_bb) const
    {
      min_bb = min_bb_;
      max_bb = max_bb_;
    }

    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::setBoundingBox (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb)
    {
      min_bb_ = min_bb;
      max_bb_ = max_bb;
      this->updateVoxelCenter ();
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    const boost::filesystem::path&
    OutofcoreOctreeNodeMetadata::getDirectoryPathname () const
    {
      return (directory_);
    }

    ////////////////////////////////////////////////////////////////////////////////
    void
    OutofcoreOctreeNodeMetadata::setDirectoryPathname (const boost::filesystem::path& directory_pathname)
    {
      directory_ = directory_pathname;
    }

    ////////////////////////////////////////////////////////////////////////////////

    const boost::filesystem::path&
    OutofcoreOctreeNodeMetadata::getPCDFilename () const
    {
      return (binary_point_filename_);
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::setPCDFilename (const boost::filesystem::path& point_filename)
    {
      binary_point_filename_ = point_filename;
    }
    

    ////////////////////////////////////////////////////////////////////////////////

    int 
    OutofcoreOctreeNodeMetadata::getOutofcoreVersion () const
    {
      return (outofcore_version_);
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::setOutofcoreVersion (const int version)
    {
      outofcore_version_ = version;
    }

    ////////////////////////////////////////////////////////////////////////////////

    const boost::filesystem::path&
    OutofcoreOctreeNodeMetadata::getMetadataFilename () const
    {
      return (metadata_filename_);
    }
    
    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::setMetadataFilename (const boost::filesystem::path& path_to_metadata)
    {
      directory_ = path_to_metadata.parent_path ();
      metadata_filename_ = path_to_metadata;
    }

    ////////////////////////////////////////////////////////////////////////////////

    const Eigen::Vector3d&
    OutofcoreOctreeNodeMetadata::getVoxelCenter () const
    {
      return (midpoint_xyz_);
    }

    ////////////////////////////////////////////////////////////////////////////////

    void 
    OutofcoreOctreeNodeMetadata::serializeMetadataToDisk ()
    {
      std::shared_ptr<cJSON> idx (cJSON_CreateObject (), cJSON_Delete);

      cJSON* cjson_outofcore_version = cJSON_CreateNumber (outofcore_version_);
  
      double min_array[3];
      double max_array[3];

      for(int i=0; i<3; i++)
      {
        min_array[i] = min_bb_[i];
        max_array[i] = max_bb_[i];
      }
      
      cJSON* cjson_bb_min = cJSON_CreateDoubleArray (min_array, 3);
      cJSON* cjson_bb_max = cJSON_CreateDoubleArray (max_array, 3);

      std::string binary_point_filename_string = binary_point_filename_.filename ().generic_string ();
      cJSON* cjson_bin_point_filename = cJSON_CreateString (binary_point_filename_string.c_str ());

      cJSON_AddItemToObject (idx.get (), "version", cjson_outofcore_version);
      cJSON_AddItemToObject (idx.get (), "bb_min", cjson_bb_min);
      cJSON_AddItemToObject (idx.get (), "bb_max", cjson_bb_max);
      cJSON_AddItemToObject (idx.get (), "bin", cjson_bin_point_filename);

      char* idx_txt = cJSON_Print (idx.get ());

      std::ofstream f (metadata_filename_.string ().c_str (), std::ios::out | std::ios::trunc);
      f << idx_txt;
      f.close ();

      free (idx_txt);
    }

    ////////////////////////////////////////////////////////////////////////////////

    int 
    OutofcoreOctreeNodeMetadata::loadMetadataFromDisk ()
    {
      if(directory_ != metadata_filename_.parent_path ())
      {
        PCL_ERROR ("directory_ is not set correctly\n");
      }
      
      //if the file to load doesn't exist, return failure
      if (!boost::filesystem::exists (metadata_filename_))
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeNodeMetadata] Can not find index metadata at %s.\n", metadata_filename_.c_str ());
        return (0);
      }
      if(boost::filesystem::is_directory (metadata_filename_))
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeNodeMetadata] Got a directory, but no oct_idx metadata?\n");
        return (0);
      }

      //load CJSON
      std::vector<char> idx_input;
      std::uintmax_t len = boost::filesystem::file_size (metadata_filename_);
      idx_input.resize (len + 1);
      
      std::ifstream f (metadata_filename_.string ().c_str (), std::ios::in);
      f.read (&(idx_input.front ()), len);
      idx_input.back () = '\0';
      
      //Parse
      std::shared_ptr<cJSON> idx (cJSON_Parse (&(idx_input.front ())), cJSON_Delete);

      cJSON* cjson_outofcore_version = cJSON_GetObjectItem (idx.get (), "version");
      cJSON* cjson_bb_min = cJSON_GetObjectItem (idx.get (), "bb_min");
      cJSON* cjson_bb_max = cJSON_GetObjectItem (idx.get (), "bb_max");
      cJSON* cjson_bin_point_filename = cJSON_GetObjectItem (idx.get (), "bin");

      bool parse_failure = false;
      
      //Sanitize
      if (!cjson_outofcore_version)
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeNodeMetadata::%s] Failed to parse \"version\" field of node metadata %s\n", __FUNCTION__, metadata_filename_.c_str ());
        parse_failure = true;
      }
      if (!cjson_bb_min)
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeNodeMetadata::%s] Failed to parse \"bb_min\" field of node metadata %s\n", __FUNCTION__, metadata_filename_.c_str ());        
        parse_failure = true;
      }
      if (!cjson_bb_max)
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeNodeMetadata::%s] Failed to parse \"bb_max\" field of node metadata %s\n", __FUNCTION__, metadata_filename_.c_str ());
        parse_failure = true;
      }
      if (!cjson_bin_point_filename)
      {
        PCL_ERROR ("[pcl::outofcore::OutofcoreOctreeNodeMetadata::%s] Failed to parse \"bin\" field of node metadata %s\n", __FUNCTION__, metadata_filename_.c_str ());
        parse_failure = true;
      }
      
      if (parse_failure)
      {
        PCL_THROW_EXCEPTION (PCLException, "[pcl::outofcore::OutofcoreOctreeNodeMetadata::%s] Outofcore node metadata parse error\n");
      }
      
      for (int i = 0; i < 3; i++)
      {
        min_bb_[i] = cJSON_GetArrayItem (cjson_bb_min, i)->valuedouble;
        max_bb_[i] = cJSON_GetArrayItem (cjson_bb_max, i)->valuedouble;
      }
      outofcore_version_ = cjson_outofcore_version->valueint;

      binary_point_filename_= directory_ / cjson_bin_point_filename->valuestring;
      midpoint_xyz_ = (max_bb_+min_bb_)/static_cast<double>(2.0);
      
      //return success
      return (1);
    }
    ////////////////////////////////////////////////////////////////////////////////

    int 
    OutofcoreOctreeNodeMetadata::loadMetadataFromDisk (const boost::filesystem::path& path_to_metadata)
    {
      this->setMetadataFilename (path_to_metadata);
      this->setDirectoryPathname (path_to_metadata.parent_path ());
      return (this->loadMetadataFromDisk ());
    }

    ////////////////////////////////////////////////////////////////////////////////    

    std::ostream& 
    operator<<(std::ostream&, const OutofcoreOctreeNodeMetadata&)
    {
      //todo: implement me
      PCL_THROW_EXCEPTION (PCLException, "Not implemented\n");
    }
    
    ////////////////////////////////////////////////////////////////////////////////


  }//namespace outofcore
}//namespace pcl

  
    
    
