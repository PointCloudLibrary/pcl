#pragma once

#ifdef USE_ROS
   #error USE_ROS setup requires PCL to compile against ROS message headers, which is now deprecated
#endif

#include <ostream>
#include <vector>

#include <boost/predef/other/endian.h>

// Include the correct Header path here
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>

namespace pcl
{

  struct PCL_EXPORTS PCLPointCloud2
  {
    PCLPointCloud2 () : height (0), width (0), 
                     is_bigendian (false), point_step (0), row_step (0),
                     is_dense (false)
    {
#if BOOST_ENDIAN_BIG_BYTE
      is_bigendian = true;
#elif BOOST_ENDIAN_LITTLE_BYTE
      is_bigendian = false;
#else
#error "unable to determine system endianness"
#endif
    }

    ::pcl::PCLHeader header;

    pcl::uint32_t height;
    pcl::uint32_t width;

    std::vector< ::pcl::PCLPointField>  fields;

    pcl::uint8_t is_bigendian;
    pcl::uint32_t point_step;
    pcl::uint32_t row_step;

    std::vector<pcl::uint8_t> data;

    pcl::uint8_t is_dense;

  public:
    using Ptr = boost::shared_ptr< ::pcl::PCLPointCloud2>;
    using ConstPtr = boost::shared_ptr<const ::pcl::PCLPointCloud2>;

    //////////////////////////////////////////////////////////////////////////
    /** \brief Inplace concatenate two pcl::PCLPointCloud2
      *
      * IFF the layout of all the fields in both the clouds is the same, this command
      * doesn't remove any fields named "_" (aka marked as skip). For comparison of field
      * names, "rgb" and "rgba" are considered equivalent
      * However, if the order and/or number of non-skip fields is different, the skip fields
      * are dropped and non-skip fields copied selectively.
      * This function returns an error if
      *   * the total number of non-skip fields is different
      *   * the non-skip field names are named differently (excluding "rbg{a}") in serial order
      *   * the endian-ness of both clouds is different
      * \param[in,out] cloud1 the first input and output point cloud dataset
      * \param[in] cloud2 the second input point cloud dataset
      * \return true if successful, false if failed (e.g., name/number of fields differs)
      */
    static bool
    concatenate (pcl::PCLPointCloud2 &cloud1, const pcl::PCLPointCloud2 &cloud2);

    /** \brief Concatenate two pcl::PCLPointCloud2
      * \param[in] cloud1 the first input point cloud dataset
      * \param[in] cloud2 the second input point cloud dataset
      * \param[out] cloud_out the resultant output point cloud dataset
      * \return true if successful, false if failed (e.g., name/number of fields differs)
      */
    static bool
    concatenate (const PCLPointCloud2 &cloud1,
                 const PCLPointCloud2 &cloud2,
                 PCLPointCloud2 &cloud_out)
    {
      cloud_out = cloud1;
      return concatenate(cloud_out, cloud2);
    }

    /** \brief Add a point cloud to the current cloud.
      * \param[in] rhs the cloud to add to the current cloud
      * \return the new cloud as a concatenation of the current cloud and the new given cloud
      */
    PCLPointCloud2&
    operator += (const PCLPointCloud2& rhs);

    /** \brief Add a point cloud to another cloud.
      * \param[in] rhs the cloud to add to the current cloud
      * \return the new cloud as a concatenation of the current cloud and the new given cloud
      */
    inline PCLPointCloud2
    operator + (const PCLPointCloud2& rhs)
    {
      return (PCLPointCloud2 (*this) += rhs);
    }
  }; // struct PCLPointCloud2

  using PCLPointCloud2Ptr = boost::shared_ptr< ::pcl::PCLPointCloud2>;
  using PCLPointCloud2ConstPtr = boost::shared_ptr<const ::pcl::PCLPointCloud2>;

  inline std::ostream& operator<<(std::ostream& s, const  ::pcl::PCLPointCloud2 &v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "height: ";
    s << "  " << v.height << std::endl;
    s << "width: ";
    s << "  " << v.width << std::endl;
    s << "fields[]" << std::endl;
    for (std::size_t i = 0; i < v.fields.size (); ++i)
    {
      s << "  fields[" << i << "]: ";
      s << std::endl;
      s << "    " << v.fields[i] << std::endl;
    }
    s << "is_bigendian: ";
    s << "  " << v.is_bigendian << std::endl;
    s << "point_step: ";
    s << "  " << v.point_step << std::endl;
    s << "row_step: ";
    s << "  " << v.row_step << std::endl;
    s << "data[]" << std::endl;
    for (std::size_t i = 0; i < v.data.size (); ++i)
    {
      s << "  data[" << i << "]: ";
      s << "  " << v.data[i] << std::endl;
    }
    s << "is_dense: ";
    s << "  " << v.is_dense << std::endl;
    
    return (s);
  }

} // namespace pcl
