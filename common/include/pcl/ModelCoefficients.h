#ifndef PCL_MESSAGE_MODELCOEFFICIENTS_H
#define PCL_MESSAGE_MODELCOEFFICIENTS_H
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include <pcl/PCLHeader.h>

namespace pcl
{
  struct ModelCoefficients
  {
    ModelCoefficients () : header (), values ()
    {
    }

    ::pcl::PCLHeader header;

    std::vector<float> values;

  public:
    typedef boost::shared_ptr< ::pcl::ModelCoefficients> Ptr;
    typedef boost::shared_ptr< ::pcl::ModelCoefficients  const> ConstPtr;
  }; // struct ModelCoefficients

  typedef boost::shared_ptr< ::pcl::ModelCoefficients> ModelCoefficientsPtr;
  typedef boost::shared_ptr< ::pcl::ModelCoefficients const> ModelCoefficientsConstPtr;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::ModelCoefficients & v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "values[]" << std::endl;
    for (size_t i = 0; i < v.values.size (); ++i)
    {
      s << "  values[" << i << "]: ";
      s << "  " << v.values[i] << std::endl;
    }
    return (s);
  }

} // namespace pcl

#endif // PCL_MESSAGE_MODELCOEFFICIENTS_H

