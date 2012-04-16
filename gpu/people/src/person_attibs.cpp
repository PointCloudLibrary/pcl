#include <pcl/gpu/people/person_attribs.h>
#include <pcl/gpu/people/label_common.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>


void
pcl::gpu::people::PersonAttribs::readPersonXMLConfig (std::istream& is)
{
  boost::property_tree::ptree pt;
  read_xml(is,pt);
}

void
pcl::gpu::people::PersonAttribs::writePersonXMLConfig (std::ostream& os)
{
  boost::property_tree::ptree pt;
  pt.add("version", XML_VERSION);
//  boost::property_tree::ptree& node = pt.add("person", "");
//  node.put("name", name_);
  pt.add("person.name", name_);

  write_xml(os,pt);
}




