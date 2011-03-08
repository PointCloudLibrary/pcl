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
 * $Id: io.hpp 34756 2010-12-15 00:40:22Z mihelich $
 *
 */

namespace pcl
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Helper functor structure for concatenate. */
  template<typename PointInT, typename PointOutT>
    struct NdConcatenateFunctor
  {
    typedef typename traits::POD<PointInT>::type PodIn;
    typedef typename traits::POD<PointOutT>::type PodOut;
    
    NdConcatenateFunctor (const PointInT &p1, PointOutT &p2)
      : p1_ (reinterpret_cast<const PodIn&>(p1)), p2_ (reinterpret_cast<PodOut&>(p2)) { }

    template<typename Key> inline void operator() ()
    {
      // This sucks without Fusion :(
      //boost::fusion::at_key<Key> (p2_) = boost::fusion::at_key<Key> (p1_);
      typedef typename pcl::traits::datatype<PointInT, Key>::type InT;
      typedef typename pcl::traits::datatype<PointOutT, Key>::type OutT;
      // Note: don't currently support different types for the same field (e.g. converting double to float)
      BOOST_MPL_ASSERT_MSG((boost::is_same<InT, OutT>::value),
                           POINT_IN_AND_POINT_OUT_HAVE_DIFFERENT_TYPES_FOR_FIELD,
                           (Key, PointInT, InT, PointOutT, OutT));
      memcpy(reinterpret_cast<uint8_t*>(&p2_) + pcl::traits::offset<PointOutT, Key>::value,
             reinterpret_cast<const uint8_t*>(&p1_) + pcl::traits::offset<PointInT, Key>::value,
             sizeof(InT));
    }

    private:
      const PodIn &p1_;
      PodOut &p2_;
  };
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> int
pcl::getFieldIndex (const pcl::PointCloud<PointT> &cloud, const std::string &field_name, std::vector<sensor_msgs::PointField> &fields)
{
  fields.clear ();
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
  for (size_t d = 0; d < fields.size (); ++d)
    if (fields[d].name == field_name)
      return (d);
  return (-1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::getFields (const pcl::PointCloud<PointT> &cloud, std::vector<sensor_msgs::PointField> &fields)
{
  fields.clear ();
  // Get the fields list
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> std::string
pcl::getFieldsList (const pcl::PointCloud<PointT> &cloud)
{
  // Get the fields list
  std::vector<sensor_msgs::PointField> fields;
  pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));
  std::string result;
  for (size_t i = 0; i < fields.size () - 1; ++i)
    result += fields[i].name + " ";
  result += fields[fields.size () - 1].name;
  return (result);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::copyPointCloud (const pcl::PointCloud<PointInT> &cloud_in, pcl::PointCloud<PointOutT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.points.resize (cloud_in.points.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = cloud_in.points.size ();
  cloud_out.height   = 1;
  cloud_out.is_dense = cloud_in.is_dense;

  // Copy all the data fields from the input cloud to the output one
  typedef typename pcl::traits::fieldList<PointInT>::type FieldList;
  // Iterate over each point
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (cloud_in.points[i], cloud_out.points[i]));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, const std::vector<int> &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = indices.size ();
  cloud_out.height   = 1;
  if (cloud_in.is_dense)
    cloud_out.is_dense = true;
  else
    // It's not necessarily true that is_dense is false if cloud_in.is_dense is false
    // To verify this, we would need to iterate over all points and check for NaNs
    cloud_out.is_dense = false;

  // Copy all the data fields from the input cloud to the output one
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  // Iterate over each point
  for (size_t i = 0; i < indices.size (); ++i)
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointT> (cloud_in.points[indices[i]], cloud_out.points[i]));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, const pcl::PointIndices &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  // Allocate enough space and copy the basics
  cloud_out.points.resize (indices.indices.size ());
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = indices.indices.size ();
  cloud_out.height   = 1;
  if (cloud_in.is_dense)
    cloud_out.is_dense = true;
  else
    // It's not necessarily true that is_dense is false if cloud_in.is_dense is false
    // To verify this, we would need to iterate over all points and check for NaNs
    cloud_out.is_dense = false;

  // Copy all the data fields from the input cloud to the output one
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  // Iterate over each point
  for (size_t i = 0; i < indices.indices.size (); ++i)
    // Iterate over each dimension
    pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointT> (cloud_in.points[indices.indices[i]], cloud_out.points[i]));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::copyPointCloud (const pcl::PointCloud<PointT> &cloud_in, const std::vector<pcl::PointIndices> &indices,
                     pcl::PointCloud<PointT> &cloud_out)
{
  int nr_p = 0;
  for (size_t i = 0; i < indices.size (); ++i)
    nr_p += indices[i].indices.size ();

  // Allocate enough space and copy the basics
  cloud_out.points.resize (nr_p);
  cloud_out.header   = cloud_in.header;
  cloud_out.width    = nr_p;
  cloud_out.height   = 1;
  if (cloud_in.is_dense)
    cloud_out.is_dense = true;
  else
    // It's not necessarily true that is_dense is false if cloud_in.is_dense is false
    // To verify this, we would need to iterate over all points and check for NaNs
    cloud_out.is_dense = false;

  // Copy all the data fields from the input cloud to the output one
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;
  // Iterate over each cluster
  int cp = 0;
  for (size_t cc = 0; cc < indices.size (); ++cc)
  {
    // Iterate over each idx
    for (size_t i = 0; i < indices[cc].indices.size (); ++i)
    {
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointT, PointT> (cloud_in.points[indices[cc].indices[i]], cloud_out.points[cp]));
      cp++;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointIn1T, typename PointIn2T, typename PointOutT> void
pcl::concatenateFields (const pcl::PointCloud<PointIn1T> &cloud1_in,
                        const pcl::PointCloud<PointIn2T> &cloud2_in,
                        pcl::PointCloud<PointOutT> &cloud_out)
{
  typedef typename pcl::traits::fieldList<PointIn1T>::type FieldList1;
  typedef typename pcl::traits::fieldList<PointIn2T>::type FieldList2;

  if (cloud1_in.points.size () != cloud2_in.points.size ())
  {
    ROS_ERROR ("[pcl::concatenateFields] The number of points in the two input datasets differs!");
    return;
  }

  // Resize the output dataset
  cloud_out.points.resize (cloud1_in.points.size ());
  cloud_out.header   = cloud1_in.header;
  cloud_out.width    = cloud1_in.width;
  cloud_out.height   = cloud1_in.height;
  if (!cloud1_in.is_dense || !cloud2_in.is_dense)
    cloud_out.is_dense = false;
  else
    cloud_out.is_dense = true;

  // Iterate over each point
  for (size_t i = 0; i < cloud_out.points.size (); ++i)
  {
    // Iterate over each dimension
    pcl::for_each_type <FieldList1> (pcl::NdConcatenateFunctor <PointIn1T, PointOutT> (cloud1_in.points[i], cloud_out.points[i]));
    pcl::for_each_type <FieldList2> (pcl::NdConcatenateFunctor <PointIn2T, PointOutT> (cloud2_in.points[i], cloud_out.points[i]));
  }
}
