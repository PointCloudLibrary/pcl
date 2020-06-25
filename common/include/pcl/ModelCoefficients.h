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
    PCLHeader header;

    std::vector<float> values;

  public:
    using Ptr = shared_ptr<ModelCoefficients>;
    using ConstPtr = shared_ptr<const ModelCoefficients>;
  }; // struct ModelCoefficients

  using ModelCoefficientsPtr = ModelCoefficients::Ptr;
  using ModelCoefficientsConstPtr = ModelCoefficients::ConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const ModelCoefficients& v)
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
