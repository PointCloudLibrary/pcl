/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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

#include <pcl/common/io.h>  // for getFieldSize
#include <pcl/common/utils.h> // pcl::utils::ignore
#include <pcl/io/ascii_io.h>
#include <istream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp> // for lexical_cast
#include <boost/algorithm/string.hpp> // for split
#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
pcl::ASCIIReader::ASCIIReader ()
{
  {
    pcl::PCLPointField f;
    f.datatype = pcl::PCLPointField::FLOAT32;
    f.count    = 1;
    f.name     = "x";
    f.offset   = 0;
    fields_.push_back (f);
  }

  {
    pcl::PCLPointField f;
    f.datatype = pcl::PCLPointField::FLOAT32;
    f.count    = 1;
    f.name     = "y";
    f.offset   = 4;
    fields_.push_back (f);
  }

  {
    pcl::PCLPointField f;
    f.datatype = pcl::PCLPointField::FLOAT32;
    f.count    = 1;
    f.name     = "z";
    f.offset   = 8;
    fields_.push_back (f);
  }
}

//////////////////////////////////////////////////////////////////////////////
pcl::ASCIIReader::~ASCIIReader () = default;

//////////////////////////////////////////////////////////////////////////////
int
pcl::ASCIIReader::readHeader (const std::string& file_name,
  pcl::PCLPointCloud2& cloud, Eigen::Vector4f& origin,
  Eigen::Quaternionf& orientation, int& file_version, int& data_type,
  unsigned int& data_idx, const int offset)
{
  pcl::utils::ignore(offset); //offset is not used for ascii file implementation
	
  boost::filesystem::path fpath = file_name;

  if (!boost::filesystem::exists (fpath))
  {
    PCL_ERROR ("[%s] File %s does not exist.\n", name_.c_str (), file_name.c_str ());
    return (-1);
  }
  if (boost::filesystem::extension (fpath) != extension_)
  {
    PCL_ERROR ("[%s] File does not have %s extension. \n", name_.c_str(), extension_.c_str());
    return -1;
  }

  cloud.fields = fields_;
  cloud.point_step = 0;
  for (std::size_t i = 0; i < fields_.size (); i++) 
    cloud.point_step += typeSize (cloud.fields[i].datatype);

  std::fstream ifile (file_name.c_str (), std::fstream::in);
  std::string line;
  int total = 0;
  while (std::getline (ifile, line))
    total++;

  origin = Eigen::Vector4f::Zero ();
  orientation = Eigen::Quaternionf::Identity ();
  cloud.width = total;
  cloud.height = 1;
  cloud.is_dense = true;
  file_version = -1;
  data_type = 0;
  data_idx  = 0;
  return (total);
}

//////////////////////////////////////////////////////////////////////////////
int
pcl::ASCIIReader::read (
    const std::string& file_name,
    pcl::PCLPointCloud2& cloud,
    Eigen::Vector4f& origin,
    Eigen::Quaternionf& orientation, int& file_version, const int offset)
{

  int  data_type;
  unsigned int data_idx;
  if (this->readHeader (file_name, cloud, origin, orientation, file_version, data_type, data_idx, offset) < 0) 
    return (-1);
  cloud.data.resize (cloud.height * cloud.width * cloud.point_step);

  std::string line;
  std::fstream ifile (file_name.c_str (), std::fstream::in);

  int total=0;

  std::uint8_t* data = &cloud.data[0];
  while (std::getline (ifile, line))
  {
    boost::algorithm::trim (line);
    if (line.find_first_not_of ('#') != 0) 
      continue;   //skip comment lines

   std::vector<std::string> tokens;
   boost::algorithm::split (tokens, line,boost::algorithm::is_any_of (sep_chars_), boost::algorithm::token_compress_on);

   if (tokens.size () != fields_.size ()) 
     continue;

   std::uint32_t offset = 0;
   try
   {
     for (std::size_t i = 0; i < fields_.size (); i++) 
       offset += parse (tokens[i], fields_[i], data + offset);
   }
   catch (std::exception& /*e*/)
   {
     continue;
   }
   data += offset;
   total++;
  }
  cloud.data.resize (total * cloud.point_step);
  return (cloud.width * cloud.height);
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::ASCIIReader::setInputFields (const std::vector<pcl::PCLPointField>& fields)
{
	fields_ = fields;
}

//////////////////////////////////////////////////////////////////////////////
void
pcl::ASCIIReader::setSepChars (const std::string &chars)
{
	sep_chars_ = chars;
}

//////////////////////////////////////////////////////////////////////////////
int
pcl::ASCIIReader::parse (
    const std::string& token,
    const pcl::PCLPointField& field,
    std::uint8_t* data_target)
{
#define ASSIGN_TOKEN(CASE_LABEL)                                                       \
  case CASE_LABEL: {                                                                   \
    *(reinterpret_cast<pcl::traits::asType_t<CASE_LABEL>*>(data_target)) =             \
        boost::lexical_cast<pcl::traits::asType_t<CASE_LABEL>>(token);                 \
    return sizeof(pcl::traits::asType_t<CASE_LABEL>);                                  \
  }
  switch (field.datatype)
  {
    ASSIGN_TOKEN(pcl::PCLPointField::BOOL)
    ASSIGN_TOKEN(pcl::PCLPointField::INT8)
    ASSIGN_TOKEN(pcl::PCLPointField::UINT8)
    ASSIGN_TOKEN(pcl::PCLPointField::INT16)
    ASSIGN_TOKEN(pcl::PCLPointField::UINT16)
    ASSIGN_TOKEN(pcl::PCLPointField::INT32)
    ASSIGN_TOKEN(pcl::PCLPointField::UINT32)
    ASSIGN_TOKEN(pcl::PCLPointField::INT64)
    ASSIGN_TOKEN(pcl::PCLPointField::UINT64)
    ASSIGN_TOKEN(pcl::PCLPointField::FLOAT32)
    ASSIGN_TOKEN(pcl::PCLPointField::FLOAT64)
  }
#undef ASSIGN_TOKEN
  return 0;
}

//////////////////////////////////////////////////////////////////////////////
std::uint32_t
pcl::ASCIIReader::typeSize (int type)
{
  return getFieldSize(type);
}

