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


#ifndef PCL_OUTOFCORE_METADATA_H_
#define PCL_OUTOFCORE_METADATA_H_

#include <pcl/outofcore/boost.h>
#include <vector>
#include <ostream>

namespace pcl
{
  namespace outofcore
  {
    
    /** \class AbstractMetadata
     *
     *  \brief Abstract interface for outofcore metadata file types
     *
     *  \ingroup outofcore
     *  \author Stephen Fox (foxstephend@gmail.com)
     */

    class PCL_EXPORTS OutofcoreAbstractMetadata
    {
    public:
      
      /** \brief Empty constructor */
      OutofcoreAbstractMetadata ()
      {
      }
      
      virtual
      ~OutofcoreAbstractMetadata ()
      {}
      
      /** \brief Write the metadata in the on-disk format, e.g. JSON. */
      virtual void
      serializeMetadataToDisk () = 0;

      /** \brief Method which should read and parse metadata and store
       *  it in variables that have public getters and setters*/
      virtual int
      loadMetadataFromDisk (const boost::filesystem::path& path_to_metadata) = 0;
      
      /** \brief Should write the same ascii metadata that is saved on
       *   disk, or a human readable format of the metadata in case a binary format is being used */
      friend std::ostream& 
      operator<<(std::ostream& os, const OutofcoreAbstractMetadata& metadata_arg);
      
    protected:
      
      /** \brief Constructs the metadata ascii which can be written to disk or piped to stdout */
      virtual void
      writeMetadataString (std::vector<char>& buf) =0;
      
    };
    
  }//namespace outofcore
}//namespace pcl

#endif //PCL_OUTOFCORE_METADATA_H_
