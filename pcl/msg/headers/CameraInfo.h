#ifndef PCL_MESSAGE_CAMERAINFO_H
#define PCL_MESSAGE_CAMERAINFO_H
#include <string>
#include <vector>
#include <ostream>
#include <boost/array.hpp>

// Include the correct Header path here
#include "Header.h"
#include "RegionOfInterest.h"

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

    typedef  ::roslib::Header_<ContainerAllocator>  _header_type;
     ::roslib::Header_<ContainerAllocator>  header;

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
} // namespace sensor_msgs

#endif // PCL_MESSAGE_CAMERAINFO_H

