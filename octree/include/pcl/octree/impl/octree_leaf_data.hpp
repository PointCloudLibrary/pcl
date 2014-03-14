
#ifndef PCL_OCTREE_LEAF_DATA_HPP_
#define PCL_OCTREE_LEAF_DATA_HPP_


namespace pcl
{
  namespace detail
  {
    /* The accumulator structure and the ones that derive/specialize it are
    * supposed to accumulate some particular piece of information that is
    * present in a point type on which it is templated.
    *
    * New points are added with `add()`, and where accumulator picks up the
    * relevant piece of data from the point and stores it internally. When all
    * the points have been inserted, `get()` should be called to compute the
    * average and retrieve it. */
    
    /* xyz_accumulator computes the sum of x, y, and z fields of the points that
    * are added to it. It has two versions, one for the point types that
    * actually have x, y, and z fields, and the other one for the types that do
    * not. The latter simply does nothing. */
    
    struct xyz_accumulator
    {
      size_t n;
      Eigen::Vector3f xyz;
      xyz_accumulator () : n (0), xyz (Eigen::Vector3f::Zero ()) { }
      // No-op add/get for point without xyz
      template <typename T> typename boost::disable_if<pcl::traits::has_xyz<T>, void>::type
      add (const T& t) { }
      template <typename T> typename boost::disable_if<pcl::traits::has_xyz<T>, void>::type
      get (T& t) const { }
      // Functional add/get for points with xyz
      template <typename T> typename boost::enable_if<pcl::traits::has_xyz<T>, void>::type
      add (const T& t) { xyz += t.getVector3fMap (); ++n; }
      template <typename T> typename boost::enable_if<pcl::traits::has_xyz<T>, void>::type
      get (T& t) const { t.getVector3fMap () = xyz / n; }
    };

    /* Computes the average of all normal vectors and normalizes it. Also
    * computes the average curvature. */
    
    struct normal_accumulator
    {
      size_t n;
      Eigen::Vector4f normal;
      float curvature;
      normal_accumulator () : n (0), normal (Eigen::Vector4f::Zero ()), curvature (0) { }
      // No-op add/get for points without normal
      template <typename T> typename boost::disable_if<pcl::traits::has_normal<T>, void>::type
      add (const T& t) { }
      template <typename T> typename boost::disable_if<pcl::traits::has_normal<T>, void>::type
      get (T& t) const { }
      // Functional add/get for points with normal
      template <typename T> typename boost::enable_if<pcl::traits::has_normal<T>, void>::type
      add (const T& t) { normal += t.getNormalVector4fMap (); curvature += t.curvature; ++n; }
      template <typename T> typename boost::enable_if<pcl::traits::has_normal<T>, void>::type
      get (T& t) const
      {
        t.getNormalVector4fMap () = normal;
        t.getNormalVector4fMap ().normalize ();
        t.curvature = curvature / n;
      }
    };

    /* Computes the average for each of the RGB channels separately. */
    
    struct color_accumulator
    {
      size_t n;
      float r, g, b;
      color_accumulator () : n (0), r (0), g (0), b (0) { }
      // No-op add/get for points without color
      template <typename T> typename boost::disable_if<pcl::traits::has_color<T>, void>::type
      add (const T& t) { }
      template <typename T> typename boost::disable_if<pcl::traits::has_color<T>, void>::type
      get (T& t) const { }
      // Functional add/get for points with color
      template <typename T> typename boost::enable_if<pcl::traits::has_color<T>, void>::type
      add (const T& t)
      {
        r += static_cast<float> (t.r);
        g += static_cast<float> (t.g);
        b += static_cast<float> (t.b);
        ++n;
      }
      template <typename T> typename boost::enable_if<pcl::traits::has_color<T>, void>::type
      get (T& t) const
      {
        t.rgba = static_cast<uint32_t> (r / n) << 16 |
                 static_cast<uint32_t> (g / n) <<  8 |
                 static_cast<uint32_t> (b / n);
      }
    };

  }
}


#endif
