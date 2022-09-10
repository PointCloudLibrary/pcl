#pragma once

#include <ostream>
#include <vector>

#include <boost/predef/other/endian.h>

#include <pcl/pcl_macros.h>  // for PCL_EXPORTS
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>
#include <pcl/types.h>

namespace pcl
{

  struct PCL_EXPORTS PCLPointCloud2
  {
    ::pcl::PCLHeader header;

    uindex_t height = 0;
    uindex_t width = 0;

    std::vector<::pcl::PCLPointField>  fields;

    static_assert(BOOST_ENDIAN_BIG_BYTE || BOOST_ENDIAN_LITTLE_BYTE, "unable to determine system endianness");
    std::uint8_t is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    uindex_t point_step = 0;
    uindex_t row_step = 0;

    std::vector<std::uint8_t> data;

    std::uint8_t is_dense = 0;

  public:
    using Ptr = shared_ptr< ::pcl::PCLPointCloud2>;
    using ConstPtr = shared_ptr<const ::pcl::PCLPointCloud2>;

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

    /** \brief Get value at specified offset.
      * \param[in] point_index point index.
      * \param[in] field_offset offset.
      * \return value at the given offset.
      */
    template<typename T> inline
    const T& at(const pcl::uindex_t& point_index, const pcl::uindex_t& field_offset) const {
      const auto position = point_index * point_step + field_offset;
      if (data.size () >= (position + sizeof(T)))
        return reinterpret_cast<const T&>(data[position]);
      else
        throw std::out_of_range("PCLPointCloud2::at");
    }

    /** \brief Get value at specified offset.
      * \param[in] point_index point index.
      * \param[in] field_offset offset.
      * \return value at the given offset.
      */
    template<typename T> inline
    T& at(const pcl::uindex_t& point_index, const pcl::uindex_t& field_offset) {
      const auto position = point_index * point_step + field_offset;
      if (data.size () >= (position + sizeof(T)))
        return reinterpret_cast<T&>(data[position]);
      else
        throw std::out_of_range("PCLPointCloud2::at");
    }
  }; // struct PCLPointCloud2

  using PCLPointCloud2Ptr = PCLPointCloud2::Ptr;
  using PCLPointCloud2ConstPtr = PCLPointCloud2::ConstPtr;

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
