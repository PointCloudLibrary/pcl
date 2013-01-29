/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 * @author: Koen Buys, Anatoly Baksheev
 */

#include <pcl/gpu/people/person_attribs.h>
#include <pcl/gpu/people/label_common.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pcl/console/print.h>

#include <sstream>

pcl::gpu::people::PersonAttribs::PersonAttribs()
{
  PCL_DEBUG("[pcl::gpu::people::PersonAttribs] : (D) : Constructor called\n");

  // INIT
  max_part_size_.resize(pcl::gpu::people::NUM_PARTS);

  part_ideal_length_.resize(pcl::gpu::people::NUM_PARTS);
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
    part_ideal_length_[i].resize(pcl::gpu::people::MAX_CHILD);

  max_length_offset_.resize(pcl::gpu::people::NUM_PARTS);
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
    max_length_offset_[i].resize(pcl::gpu::people::MAX_CHILD);

  nr_of_children_.resize(pcl::gpu::people::NUM_PARTS);

  name_ = "generic";
}

int
pcl::gpu::people::PersonAttribs::readPersonXMLConfig (std::istream& is)
{
  PCL_DEBUG("[pcl::gpu::people::PersonAttribs::readPersonXMLConfig] : (D) : called\n");
  // Read in the property tree
  boost::property_tree::ptree pt;
  read_xml(is,pt);

  // Check file version
  int version = pt.get<int>("version");
  if(version != pcl::gpu::people::XML_VERSION)
  {
    PCL_ERROR("[pcl::gpu::people::PersonAttribs::readPersonXMLConfig] : (E) : Incompatible XML_VERSIONS\n");
    return -1;
  }

  // Check num_parts
  int num_parts = pt.get<int>("num_parts");
  if(num_parts != pcl::gpu::people::NUM_PARTS)
  {
    PCL_ERROR("[pcl::gpu::people::PersonAttribs::readPersonXMLConfig] : (E) : num_parts doesn't match\n");
    return -1;
  }

  // Check num_labels
  int num_labels = pt.get<int>("num_labels");
  if(num_labels != pcl::gpu::people::NUM_LABELS)
  {
    PCL_ERROR("[pcl::gpu::people::PersonAttribs::readPersonXMLConfig] : (E) : num_labels doesn't match\n");
    return -1;
  }

  name_ = pt.get<std::string>("person.name");

  PCL_DEBUG("[pcl::gpu::people::PersonAttribs::readPersonXMLConfig] : (D) : loaded %s\n", name_.c_str());

  // Get max_part_size_
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    std::stringstream path;
    path << "person.max_part_size.value_" << i;
    max_part_size_[i] = pt.get<float>(path.str());
  }

  // Get part_ideal_length
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    for(int j = 0; j < pcl::gpu::people::MAX_CHILD; j++)
    {
      std::stringstream path;
      path << "person.part_ideal_length.value_" << i << ".child_" << j;
      part_ideal_length_[i][j] = pt.get<float>(path.str());
    }
  }

  // Get max_length_offset_
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    for(int j = 0; j < pcl::gpu::people::MAX_CHILD; j++)
    {
      std::stringstream path;
      path << "person.max_length_offset.value_" << i << ".child_" << j;
      max_length_offset_[i][j] = pt.get<float>(path.str());
    }
  }

  // Get nr_of_children
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    std::stringstream path;
    path << "person.nr_of_children.value_" << i;
    nr_of_children_[i] = pt.get<float>(path.str());
  }

  return 0;
}

void
pcl::gpu::people::PersonAttribs::writePersonXMLConfig (std::ostream& os)
{
  PCL_DEBUG("[pcl::gpu::people::PersonAttribs::writePersonXMLConfig] : (D) : called\n");
  boost::property_tree::ptree pt;

  // Write global information which is not person specific
  pt.add("version", static_cast<int> (pcl::gpu::people::XML_VERSION));
  pt.add("num_parts", static_cast<int> (pcl::gpu::people::NUM_PARTS));
  pt.add("num_labels", static_cast<int> (NUM_LABELS));

//  boost::property_tree::ptree& node = pt.add("person", "");
//  node.put("name", name_);

  // FROM HERE PERSON SPECIFIC STUFF
  pt.add("person.name", name_);

  // Add max_part_size_
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    std::stringstream path;
    path << "person.max_part_size.value_" << i;
    pt.add(path.str(), max_part_size_[i]);
  }

  // Add part_ideal_length
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    for(int j = 0; j < pcl::gpu::people::MAX_CHILD; j++)
    {
      std::stringstream path;
      path << "person.part_ideal_length.value_" << i << ".child_" << j;
      pt.add(path.str(), part_ideal_length_[i][j]);
    }
  }

  // Add max_length_offset_
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    for(int j = 0; j < pcl::gpu::people::MAX_CHILD; j++)
    {
      std::stringstream path;
      path << "person.max_length_offset.value_" << i << ".child_" << j;
      pt.add(path.str(), max_length_offset_[i][j]);
    }
  }

  // Add nr_of_children_
  for(int i = 0; i < pcl::gpu::people::NUM_PARTS; i++)
  {
    std::stringstream path;
    path << "person.nr_of_children.value_" << i;
    pt.add(path.str(), nr_of_children_[i]);
  }

  write_xml(os,pt);
}
