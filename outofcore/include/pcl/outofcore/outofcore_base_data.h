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
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/outofcore/boost.h>
#include <pcl/outofcore/cJSON.h>

#include <pcl/outofcore/metadata.h>

//standard library
#include <string>

namespace pcl
{
  namespace outofcore
  {
    /** \class OutofcoreOctreeBaseMetadata 
     *
     *  \brief Encapsulated class to read JSON metadata into memory,
     *  and write the JSON metadata associated with the octree root
     *  node. This is global information that is not the same as the
     *  metadata for the root node. Inherits OutofcoreAbstractMetadata
     *  interface for metadata in \b pcl_outofcore.

     *
     *  This class encapsulates the outofcore base metadata
     *  serialization/deserialization. At the time it was written,
     *  this depended on cJSON to write JSON objects to disk. This
     *  class can be extended to have arbitrary JSON ascii metadata
     *  fields saved to the metadata object file on disk. The class
     *  has been encapuslated to abstract the detailso of the on-disk
     *  format from the outofcore implementation. For example, the
     *  format could be changed to XML/YAML, or any dynamic format at
     *  some point.
     *
     *  The JSON file is formatted in the following way:
     *  \verbatim
     {
       "name": "nameoftree",
       "version": 3,
       "pointtype": "urp",               #(needs to be changed*)
       "lod": 3,                         #(depth of the tree
       "numpts":  [X0, X1, X2, ..., XD], #total number of points at each LOD
       "coord_system": "ECEF"            #the tree is not affected by this value
     }
     \endverbatim
     *
     *  Any properties not stored in the metadata file are computed
     *  when the file is loaded. By convention, and for historical
     *  reasons from the original Urban Robotics implementation, the
     *  JSON file representing the overall tree is a JSON file named
     *  with the ".octree" extension.
     *
     *  \ingroup outofcore
     *  \author Stephen Fox (foxstephend@gmail.com)
     */
    class PCL_EXPORTS OutofcoreOctreeBaseMetadata : public OutofcoreAbstractMetadata
    {
      public:
        using Ptr = shared_ptr<OutofcoreOctreeBaseMetadata>;
        using ConstPtr = shared_ptr<const OutofcoreOctreeBaseMetadata>;

        /** \brief Empty constructor */
        OutofcoreOctreeBaseMetadata ();
        /** \brief Load metadata from disk 
         *
         *  \param[in] path_arg Location of JSON metadata file to load from disk
         */
        OutofcoreOctreeBaseMetadata (const boost::filesystem::path& path_arg);
        /** \brief Default destructor*/
        ~OutofcoreOctreeBaseMetadata ();

        /** \brief Copy constructor */
        OutofcoreOctreeBaseMetadata (const OutofcoreOctreeBaseMetadata& orig);

        /** \brief et the outofcore version read from the "version" field of the JSON object */
        int 
        getOutofcoreVersion () const;
        /** \brief Set the outofcore version stored in the "version" field of the JSON object */
        void 
        setOutofcoreVersion (const int version);

        /** \brief Gets the name of the JSON file */
        boost::filesystem::path 
        getMetadataFilename () const;
        /** \brief Sets the name of the JSON file */
        void 
        setMetadataFilename (const boost::filesystem::path& path_to_metadata);
                
        /** \brief Writes the data to a JSON file located at \ref metadata_filename_ */
        void 
        serializeMetadataToDisk () override;

        /** \brief Loads the data from a JSON file located at \ref metadata_filename_ */
        virtual int
        loadMetadataFromDisk ();
        /** \brief Loads the data from a JSON file located at \ref metadata_filename_ */
        
        int
        loadMetadataFromDisk (const boost::filesystem::path& path_to_metadata) override;

        /** \brief Returns the name of the tree; this is not the same as the filename */
        virtual std::string
        getOctreeName ();
        /** \brief Sets the name of the tree */
        virtual void
        setOctreeName (const std::string& name_arg);

        virtual std::string
        getPointType ();
        /** \brief Sets a single string identifying the point type of this tree */
        virtual void
        setPointType (const std::string& point_type_arg);

        virtual std::vector<std::uint64_t>&
        getLODPoints ();
        virtual std::vector<std::uint64_t>
        getLODPoints () const;
        /** \brief Get the number of points at the given depth */
        virtual std::uint64_t
        getLODPoints (const std::uint64_t& depth_index) const;
        
        /** \brief Initialize the LOD vector with points all 0 */
        virtual void
        setLODPoints (const std::uint64_t& depth);
        /** \brief Copy a vector of LOD points into this metadata (dangerous!)*/
        virtual void
        setLODPoints (std::vector<std::uint64_t>& lod_points_arg);

        /** \brief Set the number of points at lod_index_arg manually 
         *  \param[in] lod_index_arg the depth at which this increments the number of LOD points
         *  \param[in] num_points_arg The number of points to store at that LOD
         *  \param[in] increment If true, increments the number of points at the LOD rather than overwriting the number of points
         */
        virtual void
        setLODPoints (const std::uint64_t& lod_index_arg, const std::uint64_t& num_points_arg, const bool increment=true);
        
        /** \brief Set information about the coordinate system */
        virtual void
        setCoordinateSystem (const std::string& coordinate_system);
        /** \brief Get metadata information about the coordinate system */
        virtual std::string
        getCoordinateSystem () const;

        /** \brief Set the depth of the tree corresponding to JSON "lod:number". This should always be equal to LOD_num_points_.size()-1 */
        virtual void
        setDepth (const std::uint64_t& depth_arg);
        virtual std::uint64_t
        getDepth () const;

        /** \brief Provide operator overload to stream ascii file data*/
        friend std::ostream& 
        operator<<(std::ostream& os, const OutofcoreOctreeBaseMetadata& metadata_arg);

      protected:
        /** \brief Metadata (JSON) file pathname (octree extension JSON file) */
        boost::filesystem::path metadata_filename_;

        /** \brief Outofcore library version identifier; maps to JSON "version":int */
        int outofcore_version_;

        /** \brief Coordinate system; maps to JSON "coord_sys":string */
        std::string coordinate_system_;

        /** \brief Name of the tree (which could be used, for example, as the name of a layer); maps to JSON "name":string*/
        std::string tree_name_;

        /** \brief Delineates the point types of the field; maps to JSON "pointtype":string:
         *  \note This is inconsistent with "point type" fields used in PCLPointCloud2 and in other places in PCL
         */
        std::string point_type_;
        
        /** \brief Depth of the tree (which is the number of levels of depth); maps to JSON "lod":int*/
        std::uint64_t levels_of_depth_;
        
        /** \brief Vector of number of points at each LOD. For a tree with no LOD, all fields will be zero except for the field indexed by LOD_points_[levels_of_depth]; maps to JSON "numpts":int array*/
        std::vector<std::uint64_t> LOD_num_points_;

        /** \brief Writes the JSON metadata to a string */
        void
        writeMetadataString (std::vector<char>& buf) override;
    };
  }//namespace outofcore
}//namespace pcl
