/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id$
 *
 */

#include <fstream>
#include <fcntl.h>
#include <string>
#include <map>
#include <stdlib.h>
#include <boost/algorithm/string.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include "pcl/io/io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/ply.h"
#include <pcl/point_types.h>

int
pcl::PLYReader::readHeader (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                            Eigen::Vector4f &origin, Eigen::Quaternionf &orientation,
                            int &ply_version, bool &binary_data, int &data_idx)
{
  // Default values
  data_idx = 0;
  binary_data = false;
  cloud.width = cloud.height = cloud.point_step = cloud.row_step = 0;
  cloud.data.clear ();

  // By default, assume that there are _no_ invalid (e.g., NaN) points
  cloud.is_dense = true;

  std::ifstream fs;
  std::string line;

  // Open file in binary mode to avoid problem of 
  // std::getline() corrupting the result of ifstream::tellg()
  fs.open (file_name.c_str (), std::ios::binary);
  if (!fs.is_open () || fs.fail ())
  {
    PCL_ERROR ("[pcl::PLYReader::readHeader] Could not open file %s.\n", file_name.c_str ());
    return (-1);
  }

  std::vector<std::string> st;
  // Read the header and fill it in with wonderful values
  try
  {
    getline (fs, line);
    boost::trim (line);
    boost::split (st, line, boost::is_any_of ( std::string ("\t\r ")), boost::token_compress_on);

    // PLY file always start with magic line "ply"
    if (st.at (0) !=  "ply")
    {
      PCL_ERROR ("[pcl::PLYReader::readHeader] %s is not a valid ply file\n", st[0].c_str());
      return(-1);
    }

    while (!fs.eof ())
    {
      getline (fs, line);
      // Ignore empty lines
      if (line == "")
        continue;

      // Tokenize the line
      boost::trim (line);
      boost::split (st, line, boost::is_any_of (std::string ( "\t\r ")), boost::token_compress_on);

      std::string line_type = st.at (0);

      // read format
      if (line_type.substr (0, 6) == "format")
      {
        float version =  atof(st.at(2).c_str());
        //check version number
        if(version != 1.0)
        {
          PCL_ERROR ("[pcl::PLYReader::readHeader] can't handle this PLY format version %f\n", version);
          return (-1);
        }
        //check format
        if("ascii" == st.at(1))
          binary_data = false;
        else
        {
          if ("binary_big_endian" == st.at(1) || "binary_little_endian" == st.at(1))
          {
            binary_data = true;
            pcl::io::ply::Format format = pcl::io::ply::getEndianess();
            if ((("binary_big_endian" == st.at(1)) && 
                 (format == pcl::io::ply::LITTLE_ENDIAN_FORMAT)) ||
                (("binary_little_endian" == st.at(1)) && 
                 (format == pcl::io::ply::BIG_ENDIAN_FORMAT)))
              swap_bytes_ = true;
          }
          else
          {
            PCL_ERROR ("[pcl::PLYReader::readHeader] unknown format %f\n", st[1].c_str());
            return (-1);
          }
        }
        continue;
      }
      // ignore comments
      if (line_type.substr (0, 7) == "comment")
        continue;
      // read element
      if (line_type.substr (0, 7) == "element") 
      {
        if(st.size() == 3)
          parser_.push_element(st.at(1), atoi(st.at(2).c_str()));
        else
          parser_.push_element(st.at(1), 1);
        continue;
      }
      // read property
      if (line_type.substr (0, 8) == "property")
      {
        // list property
        if(st.at(1) == "list")
        {
          int size_type = pcl::io::ply::getTypeFromTypeName(st.at(2));
          int data_type = pcl::io::ply::getTypeFromTypeName(st.at(3));
          if(data_type < -1 || size_type < -1)
          {
            PCL_ERROR ("[pcl::PLYReader] parse error property list %s %s %s.\n", st[2].c_str(), st[3].c_str (), st[4].c_str ());
            return -1;
          }
          else
          {
            size_t capacity = pcl::io::ply::getMaximumCapacity(size_type);
            if(capacity == 0)
            {
              PCL_ERROR ("[pcl::PLYReader] unhandled size type for property list %s %s %s.\n", st[2].c_str(), st[3].c_str (), st[4].c_str ());
              return -1;
            }
            parser_.push_property(st.at(4), size_type, data_type);
          }
        }
        // scalar property
        else
        {
          int type = pcl::io::ply::getTypeFromTypeName(st.at(1));
          if(type < -1)
          {
            PCL_ERROR ("[pcl::PLYReader] parse error property %s %s.\n", st[1].c_str (), st[2].c_str ());
            return -1;
          }
          else
            parser_.push_property (st.at(2), type);
        }
        continue;
      }
      // end of header
      if (line_type.substr (0, 10) == "end_header") 
        data_idx = fs.tellg ();
      break;
    }
  }
  catch (const char *exception)
  {
    PCL_ERROR ("[pcl::PLYReader::readHeader] %s\n", exception);
    return (-1);
  }
  fs.close();

  pcl::io::ply::element* vertex = NULL;
  pcl::io::ply::element* camera = NULL;
  // this is the most painful part in a PLY file: 
  // you need to peek up the list property size, for now we don't deal with them
  for(pcl::io::ply::parser::iterator elements_it = parser_.begin();
      elements_it != parser_.end();
      ++elements_it)
  {
    if((*elements_it)->has_list_properties ())
    {
      PCL_ERROR ("[pcl::PLYReader::readHeader] list properties are not handled\n");
      return (-1);
    }
    if("vertex" == (*elements_it)->name_)
      vertex = *elements_it;
    if("camera" == (*elements_it)->name_)
      camera = *elements_it;
  }
  // Ensure we have some elements named vertex
  if(!vertex)
  {
    PCL_ERROR ("[pcl::PLYReader::readHeader] no element named vertex found!\n");
    return -1;
  }
  cloud.point_step = vertex->offset_;
  cloud.data.resize (vertex->count_ * vertex->offset_);
  cloud.width = vertex->count_;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.height = 1;
  cloud.fields.resize (vertex->properties_.size());
  size_t counter = 0;
  for(pcl::io::ply::element::const_iterator properties_it = vertex->properties_.begin();
      properties_it != vertex->properties_.end();
      ++properties_it, counter++)
  {
    cloud.fields.at(counter).name = (*properties_it)->name_;
    cloud.fields.at(counter).offset = counter * (*properties_it)->offset_;
    cloud.fields.at(counter).datatype = (*properties_it)->data_type_;
    if(!vertex->is_list_property(properties_it))
      cloud.fields.at(counter).count = 1;
    else
    {
      pcl::io::ply::list_property *lp = (pcl::io::ply::list_property *) (*properties_it);
      cloud.fields.at(counter).count = lp->offset_/getFieldSize(lp->data_type_);
    }
  }
  // Check if optional camera element is there
  if(camera)
    ply_version = pcl::PLYReader::PLY_V1;

  return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////

int
pcl::PLYReader::read (const std::string &file_name, sensor_msgs::PointCloud2 &cloud,
                      Eigen::Vector4f &origin, Eigen::Quaternionf &orientation, int &ply_version)
{
  using namespace pcl::io;

  bool binary_data;
  int data_idx;
  int res = readHeader (file_name, cloud, origin, orientation, ply_version, binary_data, data_idx);
  if (res < 0)
    return (res);
  
  int line_count = 0;
  assert(parser_["vertex"] != NULL);
  // Get the number of points the cloud should have
  int nr_points = cloud.width * cloud.height;

  // if ascii
  if (!binary_data)
  {
    // Re-open the file (readHeader closes it)
    std::ifstream fs;
    fs.open (file_name.c_str ());
    if (!fs.is_open () || fs.fail ())
    {
      PCL_ERROR ("[pcl::PLYReader::read] Could not open file %s.", file_name.c_str ());
      return (-1);
    }
    
    fs.seekg (data_idx, std::ios_base::beg);
    std::string line;
    std::vector<std::string> st;
    std::map<std::string, void*> associated_data;
    associated_data["vertex"] = &cloud.data[0];
    pcl::io::ply::camera sensor;
    associated_data["camera"] = &sensor;
    // Read the rest of the file
    try
    {
      while (!fs.eof ())
      {
        for(pcl::io::ply::parser::iterator elements_it = parser_.begin();
            elements_it != parser_.end();
            ++elements_it)
        {
          for(size_t counter = 0 ; counter < (*elements_it)->count_; counter++)
          {
            getline (fs, line);
            // Ignore empty lines
            if (line == "")
              continue;
              
            // Tokenize the line
            boost::trim (line);
            boost::split (st, line, boost::is_any_of (std::string ("\t\r ")), boost::token_compress_on);
        
            size_t prop_counter = 0;
            size_t offset_before = 0;
            for(pcl::io::ply::element::iterator properties_it = (*elements_it)->properties_.begin();
                properties_it != (*elements_it)->properties_.end();
                ++properties_it, prop_counter++)
            {
              if((*elements_it)->is_list_property(properties_it))
              {
                size_t list_length = atoi ((st.at(0)).c_str ());
                pcl::io::ply::list_property* lp = (pcl::io::ply::list_property*) (*properties_it);
                lp->set_size((st.at(0)).c_str ());
                for(size_t i = 0; i < list_length; i++)
                {
                  switch (lp->data_type_)
                  {
                  case sensor_msgs::PointField::INT8:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::INT8>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::UINT8:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::UINT8>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::INT16:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::INT16>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::UINT16:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::UINT16>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::INT32:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::INT32>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::UINT32:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::UINT32>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::FLOAT32:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT32>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  case sensor_msgs::PointField::FLOAT64:
                    copyStringValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT64>::type> (st.at(prop_counter+i), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before + i * pcl::getFieldSize(lp->data_type_));
                    break;
                  default:
                    PCL_WARN ("[pcl::PCDReader::read] Incorrect data type specified for list element (%d)!\n",lp->data_type_);
                    break;
                  }
                }
              }
              else
              {
                switch ((*properties_it)->data_type_)
                {
                case sensor_msgs::PointField::INT8:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::INT8>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::UINT8:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::UINT8>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::INT16:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::INT16>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::UINT16:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::UINT16>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::INT32:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::INT32>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::UINT32:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::UINT32>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::FLOAT32:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT32>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                case sensor_msgs::PointField::FLOAT64:
                  copyStringValue<pcl::traits::asType<sensor_msgs::PointField::FLOAT64>::type> (st.at(prop_counter), associated_data[(*elements_it)->name_], counter * (*elements_it)->offset_ + offset_before);
                  break;
                default:
                  PCL_WARN ("[pcl::PCDReader::read] Incorrect data type specified (%d)!\n",(*properties_it)->data_type_);
                  break;
                }
              }
              offset_before+= (*properties_it)->offset_;
            }
          }
        }
        line_count++;
      }
    }
    catch (const char *exception)
    {
      PCL_ERROR ("[pcl::PLYReader::read] %s", exception);
      return (-1);
    }

  // Close file
    fs.close ();
    if(parser_["camera"])
    {
      sensor.ext_to_eigen(origin, orientation);
    }
  }
  /// ---[ Binary mode only
  /// We must re-open the file and read with mmap () for binary
  else
  {
    // Set the is_dense mode to false -- otherwise we would have to iterate over all points and check them 1 by 1
    cloud.is_dense = false;
    // Open for reading
    boost::iostreams::mapped_file_source mapped_file_;
    mapped_file_.open (file_name, cloud.data.size () + data_idx + sizeof(pcl::io::ply::camera), 0);
    if (!mapped_file_.is_open())
      return (-1);
    
    const char *map = mapped_file_.data();

    // Copy the data
    memcpy (&cloud.data[0], &map[0] + data_idx, cloud.data.size ());

    // Copy the sensor data if available
    if(parser_["camera"] != NULL)
    {
      pcl::io::ply::camera sensor;
      memcpy (&sensor, &map[0] + data_idx + cloud.data.size (), sizeof(pcl::io::ply::camera));
      sensor.ext_to_eigen(origin, orientation);
    }
    // Unmap the pages of memory
    mapped_file_.close();
  }

  // if ( (line_count != nr_points) && (!binary_data) )
  // {
  //   PCL_ERROR ("[pcl::PLYReader::read] Number of points read (%d) is different than expected (%d)", line_count, nr_points);
  //   return (-1);
  // }
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////

void pcl::PLYWriter::setMaskFromFieldsList(const std::string& fields_list)
{
  // Find coordinates mandatory
  size_t xyz_found = fields_list.find("x y z", 0);
  if(xyz_found == std::string::npos)
  {
    // Look for XY only
    size_t xy_found = fields_list.find("x y", 0);
    if(xy_found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_XY;
    else
    {
      // Look for normal
      size_t n_found = fields_list.find("normal_x normal_y normal_z", xyz_found);
      if(n_found != std::string::npos)
        mask_ |= pcl::io::ply::VERTEX_NORMAL;
      else
        // There is no XYZ
        PCL_ERROR ("[pcl::PLYWriter] PLY file format doesn't handle this kind of data: %s!\n", fields_list.c_str());
    }
  }
  else 
  {
    mask_ |= pcl::io::ply::VERTEX_XYZ;
    xyz_found+=5;
    // Find intensity optional
    size_t found = fields_list.find("intensity", xyz_found);
    if(found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_INTENSITY;
    // Find colors optional
    found = fields_list.find("rgb", xyz_found);
    if(found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_COLOR;
    // Find range optional
    found = fields_list.find("range", xyz_found);
    if(found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_RANGE;
    // Find strength optional
    found = fields_list.find("strength", xyz_found);
    if(found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_STRENGTH;
    // Find viewpoint optional
    found = fields_list.find("vp_x vp_y vp_z", xyz_found);
    if(found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_VIEWPOINT;
    // Find normal optional
    found = fields_list.find("normal_x normal_y normal_z", xyz_found);
    if(found != std::string::npos)
      mask_ |= pcl::io::ply::VERTEX_NORMAL;    
  }
}

/////////////////////////////////////////////////////////////////////////////////////////

std::string
pcl::PLYWriter::generateHeader (const sensor_msgs::PointCloud2 &cloud, 
                                bool binary)
{
  std::ostringstream oss;
  // Begin header
  oss << "ply";
  if(!binary)
    oss << "\nformat ascii 1.0";
  else
  {
    if(cloud.is_bigendian)
      oss << "\nformat binary_big_endian 1.0";
    else
      oss << "\nformat binary_little_endian 1.0";
  } 
  oss << "\ncomment PCL generated";
  
  // Set mask from fields list
  setMaskFromFieldsList (getFieldsList (cloud));

  // If mask can not be determined
  if(mask_ == 0)
    return "";

  if(mask_ & pcl::io::ply::VERTEX_XYZ)
  {
    oss << "\nelement vertex "<< cloud.width * cloud.height;
    oss << "\nproperty float x"
      "\nproperty float y"
      "\nproperty float z";

    if(mask_ & pcl::io::ply::VERTEX_INTENSITY)
      oss << "\nproperty float intensity";

    if(mask_ & pcl::io::ply::VERTEX_NORMAL)
      oss << "\nproperty float nx"
        "\nproperty float ny"
        "\nproperty float nz"
        "\nproperty float curvature";
    
    if(mask_ & pcl::io::ply::VERTEX_COLOR)
      oss << "\nproperty uchar red"
        "\nproperty uchar green"
        "\nproperty uchar blue";
    
    if(mask_ & pcl::io::ply::VERTEX_RADIUS)
      oss << "\nproperty float radius";

    if(mask_ & pcl::io::ply::VERTEX_VIEWPOINT)
      oss << "\nproperty float vp_x"
        "\nproperty float vp_y"
        "\nproperty float vp_z";
    if(mask_ & pcl::io::ply::VERTEX_RANGE)
      oss << "\nproperty float range";
    
    if(mask_ & pcl::io::ply::VERTEX_STRENGTH)
      oss << "\nproperty float strength";
  } 
  else 
  {
    if(mask_ & pcl::io::ply::VERTEX_NORMAL)
    {
      oss << "\nelement vertex "<< cloud.width * cloud.height;
      oss << "\nproperty float nx"
        "\nproperty float ny"
        "\nproperty float nz"
        "\nproperty float curvature";
    }
    else
    {
      if(mask_ & pcl::io::ply::VERTEX_XY)
      {
        oss << "\nelement vertex "<< cloud.width * cloud.height;
        oss << "\nproperty float x"
          "\nproperty float y";
      }
    }
  }

  oss << "\nelement camera"
    "\nproperty float view_px"
    "\nproperty float view_py"
    "\nproperty float view_pz"
    "\nproperty float x_axisx"
    "\nproperty float x_axisy"
    "\nproperty float x_axisz"
    "\nproperty float y_axisx"
    "\nproperty float y_axisy"
    "\nproperty float y_axisz"
    "\nproperty float z_axisx"
    "\nproperty float z_axisy"
    "\nproperty float z_axisz"
    "\nproperty float focal"
    "\nproperty float scalex"
    "\nproperty float scaley"
    "\nproperty float centerx"
    "\nproperty float centery"
    "\nproperty int viewportx"
    "\nproperty int viewporty"
    "\nproperty float k1"
    "\nproperty float k2";

  // End header
  oss << "\nend_header\n";
  return (oss.str ());
}

/////////////////////////////////////////////////////////////////////////////////////////

int
pcl::PLYWriter::writeASCII (const std::string &file_name, 
                            const sensor_msgs::PointCloud2 &cloud, 
                            const Eigen::Vector4f &origin, 
                            const Eigen::Quaternionf &orientation,
                            int precision)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::PLYWriter::writeASCII] Input point cloud has no data!\n");
    return (-1);
  }

  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());      // Open file

  int nr_points  = cloud.width * cloud.height;
  int point_size = cloud.data.size () / nr_points;

  // Write the header information if available
  fs << generateHeader (cloud, false);
  // Iterate through the points
  for (int i = 0; i < nr_points; ++i)
  {
    for (size_t d = 0; d < cloud.fields.size (); ++d)
    {
      int count = cloud.fields[d].count;
      if (count == 0) 
        count = 1; //workaround

      for (int c = 0; c < count; ++c)
      {
        switch (cloud.fields[d].datatype)
        {
          case sensor_msgs::PointField::INT8:
          {
            char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (char)], sizeof (char));
            fs << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::UINT8:
          {
            unsigned char value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned char)], sizeof (unsigned char));
            fs << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::INT16:
          {
            short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (short)], sizeof (short));
            fs << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::UINT16:
          {
            unsigned short value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned short)], sizeof (unsigned short));
            fs << boost::numeric_cast<int>(value);
            break;
          }
          case sensor_msgs::PointField::INT32:
          {
            int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (int)], sizeof (int));
            fs << value;
            break;
          }
          case sensor_msgs::PointField::UINT32:
          {
            unsigned int value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (unsigned int)], sizeof (unsigned int));
            fs << value;
            break;
          }
          case sensor_msgs::PointField::FLOAT32:
          {
            float value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
            fs << value;
            break;
          }
          case sensor_msgs::PointField::FLOAT64:
          {
            double value;
            memcpy (&value, &cloud.data[i * point_size + cloud.fields[d].offset + c * sizeof (double)], sizeof (double));
            fs << value;
            break;
          }
          default:
            PCL_WARN ("[pcl::PLYWriter::writeASCII] Incorrect field data type specified (%d)!\n", cloud.fields[d].datatype);
            break;
        }

        if (d < cloud.fields.size () - 1 || c < (int)cloud.fields[d].count - 1)
          fs << " ";
      }
    }
    fs << std::endl;
  }
  // Append sensor information
  fs << origin[0] << " " << origin[1] << " " << origin[2] << " ";
  Eigen::Matrix3f R = orientation.toRotationMatrix ();
  fs << R(0,0) << " " << R(0,1) << " " << R(0,2) << " ";
  fs << R(1,0) << " " << R(1,1) << " " << R(1,2) << " ";
  fs << R(2,0) << " " << R(2,1) << " " << R(2,2) << " ";
  // No focal
  fs << 0 << " ";
  // No scale
  fs << 0 << " " << 0 << " ";
  // No center
  fs << 0 << " " << 0 << " ";
  // No viewport
  fs << 0 << " " << 0 << " ";
  // No corrections
  fs << 0 << " " << 0;
  fs << std::endl;
  fs.flush();
  // Close file
  fs.close ();              
  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PLYWriter::writeBinary (const std::string &file_name, 
                             const sensor_msgs::PointCloud2 &cloud,
                             const Eigen::Vector4f &origin, 
                             const Eigen::Quaternionf &orientation)
{
  if (cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::FileWriter::writeBinary] Input point cloud has no data!");
    return (-1);
  }
  int data_idx = 0;
  std::ofstream fs;

  // Write the header information
  std::ostringstream oss;
  oss << generateHeader (cloud, true);
  std::string header(oss.str());
  if("" == header)
  {
    std::cout << "unvalid header" << std::endl;
    return (-1);
  }
  oss.flush();
  data_idx = oss.tellp ();

  // Open for writing
   boost::iostreams::mapped_file_params file_params_(file_name);
   file_params_.mode = std::ios_base::out | std::ios_base::binary;
   file_params_.offset = 0;
   file_params_.new_file_size = sizeof(pcl::io::ply::camera) + cloud.data.size ()+ getpagesize() - 1;
  // mapped_file_.open(file_params_);
  boost::iostreams::mapped_file_sink mapped_file_;
//  mapped_file_.open (file_name, sizeof(pcl::io::ply::camera) + cloud.data.size ()+ getpagesize() - 1, 0);
  mapped_file_.open(file_params_);
  if (!mapped_file_.is_open())
  {
    PCL_ERROR ("[pcl::FileWriter::writeBinary] Error during open () %s!\n", file_name.c_str());
    return (-1);
  }

  // Stretch the file size to the size of the data
  mapped_file_.resize(getpagesize () + cloud.data.size () + sizeof(pcl::io::ply::camera)  - 1);
  if (mapped_file_.size() != getpagesize () + cloud.data.size () + sizeof(pcl::io::ply::camera) - 1)
  {
    mapped_file_.close();
    PCL_ERROR ("[pcl::FileWriter::writeBinary] Error during resize ()!\n");
    return (-1);
  }

  // Prepare the map
  char *map = mapped_file_.data();

  // Copy the header
  memcpy (&map[0], oss.str().c_str(), data_idx);

  // Copy the data
  memcpy (&map[0] + data_idx, &cloud.data[0], cloud.data.size ());

  // Wrap sensor data
  pcl::io::ply::camera sensor(origin, orientation);

  // Copy the sensor data
  memcpy (&map[0] + data_idx + cloud.data.size (), &sensor, sizeof(pcl::io::ply::camera));

  // Close file
  mapped_file_.close();

//   // Append sensor information

//   std::ofstream fpout(file_name.c_str(), std::ios::app | std::ios::binary);
//   if(!fpout)
//   {
//     PCL_ERROR ("[pcl::PLYWriter::writeBinary] Error during reopening (%s)!\n", file_name.c_str());
//     return (-1);
//   }

//   float t;
//   for(int i = 0; i < 3; i++)
//   {
//     t = origin[i];
//     fpout.write((const char *) &t,sizeof(float));
//   }
//   Eigen::Matrix3f R = orientation.toRotationMatrix ();
//   for(int i = 0; i < 3; i++)
//     for(int j = 0; j < 3; j++)
//   {
//     t = R(i,j);
//     fpout.write((const char *) &t,sizeof(float));
//   }


// /////////////////////////////////////////////////////
// // Append those properties directly.               //
// // They are for perspective cameras so just put 0  //
// //                                                 //
// // property float focal                            //
// // property float scalex                           //
// // property float scaley                           //
// // property float centerx                          //
// // property float centery                          //
// // property int viewportx                          //
// // property int viewporty                          //
// // property float k1                               //
// // property float k2                               //
// /////////////////////////////////////////////////////

//   float zerof = 0;
//   for(int i = 0; i < 5; i++)
//     fpout.write((const char *) &zerof,sizeof(float));
//   int zeroi = 0;
//   for(int i = 0; i < 2; i++)
//     fpout.write((const char *) &zeroi,sizeof(float));
//   for(int i = 0; i < 2; i++)
//     fpout.write((const char *) &zerof,sizeof(float));

//   fpout.close();

  return (0);
}
