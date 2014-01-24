
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
    
    template <typename T>
    struct accumulator { void add (const T&) { }; void get (T&, size_t) const { }; };
    
    /* xyz_accumulator computes the sum of x, y, and z fields of the points that
    * are added to it. It has two versions, one for the point types that
    * actually have x, y, and z fields, and the other one for the types that do
    * not. The latter simply does nothing. */
    
    template <typename T, typename Enable = void>
    struct xyz_accumulator : accumulator<T> { };
    
    template <typename T>
    struct xyz_accumulator<T, typename boost::enable_if<pcl::traits::has_xyz<T> >::type>
    {
      xyz_accumulator () : xyz (Eigen::Vector3f::Zero ()) { }
      void add (const T& t) { xyz += t.getVector3fMap (); }
      void get (T& t, size_t n) const { t.getVector3fMap () = xyz / n; } 
      Eigen::Vector3f xyz;
    };
    
    /* Computes the average of all normal vectors and normalizes it. Also
    * computes the average curvature. */
    
    template <typename T, typename Enable = void>
    struct normal_accumulator : accumulator<T> { };
    
    template <typename T>
    struct normal_accumulator<T, typename boost::enable_if<pcl::traits::has_normal<T> >::type>
    {
      normal_accumulator () : normal (Eigen::Vector4f::Zero ()), curvature (0) { }
      void add (const T& t)
      {
        normal += t.getNormalVector4fMap ();
        curvature += t.curvature;
      }
      void get (T& t, size_t n) const
      {
        t.getNormalVector4fMap () = normal / n;
        t.getNormalVector4fMap ().normalize ();
        t.curvature = curvature / n;
      }
      Eigen::Vector4f normal;
      float curvature;
    };
    
    /* Computes the average for each of the RGB channels separately. */
    
    template <typename T, typename Enable = void>
    struct color_accumulator : accumulator<T> { };
    
    template <typename T>
    struct color_accumulator<T, typename boost::enable_if<pcl::traits::has_color<T> >::type>
    {
      color_accumulator () : r (0), g (0), b (0) { }
      void add (const T& t)
      {
        r += static_cast<float> (t.r);
        g += static_cast<float> (t.g);
        b += static_cast<float> (t.b);
      }
      void get (T& t, size_t n) const
      {
        t.rgba = static_cast<uint32_t> (r / n) << 16 |
        static_cast<uint32_t> (g / n) <<  8 |
        static_cast<uint32_t> (b / n);
      }
      float r, g, b;
    };
    
  }
}


#endif