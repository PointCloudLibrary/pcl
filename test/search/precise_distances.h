namespace pcl_tests {
// Here we want very precise distance functions, speed is less important. So we use
// double precision, unlike euclideanDistance() in pcl/common/distances and distance()
// in pcl/common/geometry which use float (single precision) and possibly vectorization

template <typename PointT> inline double
squared_point_distance(const PointT& p1, const PointT& p2)
{
  const double x_diff = (static_cast<double>(p1.x) - static_cast<double>(p2.x)),
               y_diff = (static_cast<double>(p1.y) - static_cast<double>(p2.y)),
               z_diff = (static_cast<double>(p1.z) - static_cast<double>(p2.z));
  return (x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

template <typename PointT> inline double
point_distance(const PointT& p1, const PointT& p2)
{
  const double x_diff = (static_cast<double>(p1.x) - static_cast<double>(p2.x)),
               y_diff = (static_cast<double>(p1.y) - static_cast<double>(p2.y)),
               z_diff = (static_cast<double>(p1.z) - static_cast<double>(p2.z));
  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}
} // namespace pcl_tests
