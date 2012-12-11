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
 *  FOR a PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#ifndef PCL_COMMON_FILE_IO_IMPL_HPP_
#define PCL_COMMON_FILE_IO_IMPL_HPP_

namespace pcl
{

#ifndef _WIN32
  void getAllPcdFilesInDirectory(const std::string& directory, std::vector<std::string>& file_names)
  {
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(directory.c_str())) == NULL) {
      std::cerr << "Could not open directory.\n";
      return;
    }
    while ((dirp = readdir(dp)) != NULL) {
      if (dirp->d_type == DT_REG)  // Only regular files
      {
        std::string file_name = dirp->d_name;
        if (file_name.substr(file_name.size()-4, 4)==".pcd")
          file_names.push_back(dirp->d_name);
      }
    }
    closedir(dp);
    std::sort(file_names.begin(), file_names.end());
    //for (unsigned int i=0; i<file_names.size(); ++i)
      //cout << file_names[i]<<"\n";
  }
#endif

std::string getFilenameWithoutPath(const std::string& input)
{
  size_t filename_start = input.find_last_of('/', static_cast<size_t>(-1)) + 1;
  return input.substr(filename_start, input.size()-filename_start);
}

std::string getFilenameWithoutExtension(const std::string& input)
{
  size_t dot_position = input.find_last_of('.', input.size());
  return input.substr(0, dot_position);
}

std::string getFileExtension(const std::string& input)
{
  size_t dot_position = input.find_last_of('.', input.size());
  return input.substr(dot_position+1, input.size());
}

}  // namespace end

#endif

