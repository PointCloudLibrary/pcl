#pragma once

#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include <pcl/PCLHeader.h>

namespace pcl
{
  struct ModelCoefficients
  {
    ModelCoefficients ()
    {
    }

    ::pcl::PCLHeader header;

    std::vector<float> values;

  public:
    using Ptr = boost::shared_ptr< ::pcl::ModelCoefficients>;
    using ConstPtr = boost::shared_ptr<const ::pcl::ModelCoefficients>;
  }; // struct ModelCoefficients

  using ModelCoefficientsPtr = ModelCoefficients::Ptr;
  using ModelCoefficientsConstPtr = ModelCoefficients::ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::ModelCoefficients & v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "values[]" << std::endl;
    for (std::size_t i = 0; i < v.values.size (); ++i)
    {
      s << "  values[" << i << "]: ";
      s << "  " << v.values[i] << std::endl;
    }
    return (s);
  }

} // namespace pcl
