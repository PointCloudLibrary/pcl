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
 *  $Id: outofcore_node_data.h 6915 2012-08-22 10:54:21Z stfox88 $
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/outofcore/cJSON.h>

#include <pcl/common/eigen.h>

#include <boost/filesystem.hpp>

#include <ostream>

namespace pcl
{
  namespace outofcore
  {
    /** \class OutofcoreOctreeNodeMetadata 
     *
     *  \brief Encapsulated class to read JSON metadata into memory, and write the JSON metadata for each
     *  node. 
     *
     *  This class encapsulates the outofcore node metadata
     *  serialization/deserialization. At the time it was written,
     *  this depended on cJSON to write JSON objects to disk. This
     *  class can be extended to have arbitrary ascii metadata fields
     *  saved to the metadata object file on disk.
     *
     *  The JSON file is formatted in the following way:
     *  \verbatim
     {
       "version": 3,
       "bb_min":  [xxx,yyy,zzz],
       "bb_max":  [xxx,yyy,zzz],
       "bin":     "path_to_data.pcd"
     }
     \endverbatim
     *
     *  Any properties not stored in the metadata file are computed
     *  when the file is loaded (e.g. \ref midpoint_xyz_). By
     *  convention, the JSON files are stored on disk with .oct_idx
     *  extension.
     *
     *  \ingroup outofcore
     *  \author Stephen Fox (foxstephend@gmail.com)
     */
    class PCL_EXPORTS OutofcoreOctreeNodeMetadata
    {

      public:
        //public typedefs
        using Ptr = shared_ptr<OutofcoreOctreeNodeMetadata>;
        using ConstPtr = shared_ptr<const OutofcoreOctreeNodeMetadata>;
  
        /** \brief Empty constructor */
        OutofcoreOctreeNodeMetadata ();
        ~OutofcoreOctreeNodeMetadata ();

        /** \brief Copy constructor */
        OutofcoreOctreeNodeMetadata (const OutofcoreOctreeNodeMetadata& orig);
        
        /** \brief Get the lower bounding box corner */
        const Eigen::Vector3d&
        getBoundingBoxMin () const;
        /** \brief Set the lower bounding box corner */
        void 
        setBoundingBoxMin (const Eigen::Vector3d& min_bb);
        /** \brief Get the upper bounding box corner */
        const Eigen::Vector3d&
        getBoundingBoxMax () const;
        /** \brief Set the upper bounding box corner */
        void 
        setBoundingBoxMax (const Eigen::Vector3d& max_bb);

        /** \brief Get the lower and upper corners of the bounding box enclosing this node */
        void 
        getBoundingBox (Eigen::Vector3d &min_bb, Eigen::Vector3d &max_bb) const;
        /** \brief Set the lower and upper corners of the bounding box */
        void 
        setBoundingBox (const Eigen::Vector3d& min_bb, const Eigen::Vector3d& max_bb);
        
        /** \brief Get the directory path name; this is the parent_path of  */
        const boost::filesystem::path&
        getDirectoryPathname () const;
        /** \brief Set the directory path name */
        void 
        setDirectoryPathname (const boost::filesystem::path& directory_pathname);

        /** \brief Get the path to the PCD file */
        const boost::filesystem::path&
        getPCDFilename () const;
        /** \brief Set the point filename; extension .pcd */
        void 
        setPCDFilename (const boost::filesystem::path& point_filename);

        /** \brief et the outofcore version read from the "version" field of the JSON object */
        int 
        getOutofcoreVersion () const;
        /** \brief Set the outofcore version stored in the "version" field of the JSON object */
        void 
        setOutofcoreVersion (const int version);

        /** \brief Sets the name of the JSON file */
        const boost::filesystem::path&
        getMetadataFilename () const;
        /** \brief Gets the name of the JSON file */
        void 
        setMetadataFilename (const boost::filesystem::path& path_to_metadata);
        
        /** \brief Get the midpoint of this node's bounding box */
        const Eigen::Vector3d&
        getVoxelCenter () const;
        
        /** \brief Writes the data to a JSON file located at \ref metadata_filename_ */
        void 
        serializeMetadataToDisk ();

        /** \brief Loads the data from a JSON file located at \ref metadata_filename_ */
        int 
        loadMetadataFromDisk ();
        /** \brief Loads the data from a JSON file located at \ref metadata_filename_ */
        int 
        loadMetadataFromDisk (const boost::filesystem::path& path_to_metadata);

        friend
        std::ostream& operator<<(std::ostream& os, const OutofcoreOctreeNodeMetadata& metadata_arg);
        
      protected:
        /** \brief The X,Y,Z axes-aligned minimum corner for the bounding box */
        Eigen::Vector3d min_bb_;
        /** \brief The X,Y,Z axes-aligned maximum corner for the bounding box */
        Eigen::Vector3d max_bb_;
        /** \brief Path to PCD file (i.e. "bin"ary point data) */
        boost::filesystem::path binary_point_filename_;
        /** \brief Voxel center; not stored on disk */
        Eigen::Vector3d midpoint_xyz_;
        /** \brief Directory this metadata belongs in */
        boost::filesystem::path directory_;
        /** \brief Metadata (JSON) file pathname (oct_idx extension JSON file) */
        boost::filesystem::path metadata_filename_;
        /** \brief Outofcore library version identifier */
        int outofcore_version_;

        /** \brief Computes the midpoint; used when bounding box is changed */
        inline void 
        updateVoxelCenter ()
        {
          midpoint_xyz_ = (this->max_bb_ + this->min_bb_)/static_cast<double>(2.0);
        }
    };
  }//namespace outofcore
}//namespace pcl
