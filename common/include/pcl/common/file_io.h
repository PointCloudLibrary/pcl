/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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

#ifndef PCL_COMMON_FILE_IO_H_
#define PCL_COMMON_FILE_IO_H_

#include <iostream>
#ifndef _WIN32
  #include <dirent.h>
#endif
#include <vector>
#include <algorithm>

/** 
  * \file pcl/common/file_io.h
  * Define some helper functions for reading and writing files
  * \ingroup common
  * \todo move this to pcl::console
  */

/*@{*/
namespace pcl
{
  /** \brief Find all *.pcd files in the directory and return them sorted
    * \param directory the directory to be searched
    * \param file_names the resulting (sorted) list of .pcd files
    */
  inline void 
  getAllPcdFilesInDirectory (const std::string& directory, std::vector<std::string>& file_names);
  
  /** \brief Remove the path from the given string and return only the filename (the remaining string after the 
    * last '/')
    * \param input the input filename (with full path)
    * \return the resulting filename, stripped of the path
    */
  inline std::string 
  getFilenameWithoutPath (const std::string& input);

  /** \brief Remove the extension from the given string and return only the filename (everything before the last '.')
    * \param input the input filename (with the file extension)
    * \return the resulting filename, stripped of its extension
    */
  inline std::string 
  getFilenameWithoutExtension (const std::string& input);

  /** \brief Get the file extension from the given string (the remaining string after the last '.')
    * \param input the input filename
    * \return \a input 's file extension
    */
  inline std::string 
  getFileExtension (const std::string& input);
}  // namespace end
/*@}*/
#include <pcl/common/impl/file_io.hpp>

#endif  //#ifndef PCL_FILE_IO_H_
