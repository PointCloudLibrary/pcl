#ifndef PCL_MESSAGE_CAMERAINFO_H
#define PCL_MESSAGE_CAMERAINFO_H
#include <string>
#include <vector>
#include <ostream>
#include <boost/array.hpp>

// Include the correct Header path here
#include "std_msgs/Header.h"
#include "sensor_msgs/RegionOfInterest.h"

namespace sensor_msgs
{
  template <class ContainerAllocator>
  struct CameraInfo_
  {
    typedef CameraInfo_<ContainerAllocator> Type;

    CameraInfo_()
    : header()
    , height(0)
    , width(0)
    , roi()
    , D()
    , K()
    , R()
    , P()
    {
      D.assign(0.0);
      K.assign(0.0);
      R.assign(0.0);
      P.assign(0.0);
    }

    CameraInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , height(0)
    , width(0)
    , roi(_alloc)
    , D()
    , K()
    , R()
    , P()
    {
      D.assign(0.0);
      K.assign(0.0);
      R.assign(0.0);
      P.assign(0.0);
    }

    typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
     ::std_msgs::Header_<ContainerAllocator>  header;

    typedef uint32_t _height_type;
    uint32_t height;

    typedef uint32_t _width_type;
    uint32_t width;

    typedef  ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  _roi_type;
     ::sensor_msgs::RegionOfInterest_<ContainerAllocator>  roi;

    typedef boost::array<double, 5>  _D_type;
    boost::array<double, 5>  D;

    typedef boost::array<double, 9>  _K_type;
    boost::array<double, 9>  K;

    typedef boost::array<double, 9>  _R_type;
    boost::array<double, 9>  R;

    typedef boost::array<double, 12>  _P_type;
    boost::array<double, 12>  P;

    typedef boost::shared_ptr< ::sensor_msgs::CameraInfo_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::sensor_msgs::CameraInfo_<ContainerAllocator>  const> ConstPtr;
  }; // struct CameraInfo
  typedef  ::sensor_msgs::CameraInfo_<std::allocator<void> > CameraInfo;

  typedef boost::shared_ptr< ::sensor_msgs::CameraInfo> CameraInfoPtr;
  typedef boost::shared_ptr< ::sensor_msgs::CameraInfo const> CameraInfoConstPtr;

  template<typename ContainerAllocator>
  std::ostream& stream_with_indentation (std::ostream& s, const std::string& indent, 
                                         const ::sensor_msgs::CameraInfo_<ContainerAllocator> & v)
  {
    s << indent << "header: " << std::endl;
    stream_with_indentation (s, indent + "  ", v.header);
    s << indent << "height: ";
    s << indent << "  " << v.height << std::endl;
    s << indent << "width: ";
    s << indent << "  " << v.width << std::endl;
    //s << indent << "distortion_model: ";
    //s << indent << "  " << v.distortion_model << std::endl;
    s << indent << "D[]" << std::endl;
    for (size_t i = 0; i < v.D.size (); ++i)
    {
      s << indent << "  D[" << i << "]: ";
      s << indent << "  " << v.D[i] << std::endl;
    }
    s << indent << "K[]" << std::endl;
    for (size_t i = 0; i < v.K.size (); ++i)
    {
      s << indent << "  K[" << i << "]: ";
      s << indent << "  " << v.K[i] << std::endl;
    }
    s << indent << "R[]" << std::endl;
    for (size_t i = 0; i < v.R.size (); ++i)
    {
      s << indent << "  R[" << i << "]: ";
      s << indent << "  " << v.R[i] << std::endl;
    }
    s << indent << "P[]" << std::endl;
    for (size_t i = 0; i < v.P.size (); ++i)
    {
      s << indent << "  P[" << i << "]: ";
      s << indent << "  " << v.P[i] << std::endl;
    }
    //s << indent << "binning_x: ";
    //s << indent << "  " << v.binning_x << std::endl;
    //s << indent << "binning_y: ";
    //s << indent << "  " << v.binning_y << std::endl;
    s << indent << "roi: ";
    s << std::endl;
    stream_with_indentation (s, indent + "  ", v.roi);
    return (s);
  }

  template<typename ContainerAllocator>
  std::ostream& operator<<(std::ostream& s, const  ::sensor_msgs::CameraInfo_<ContainerAllocator> & v)
  {
    stream_with_indentation (s, "", v);
    return (s);
  }

} // namespace sensor_msgs

#endif // PCL_MESSAGE_CAMERAINFO_H

